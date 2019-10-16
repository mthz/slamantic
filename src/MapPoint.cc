/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>
#include <slamantic/LabelHistogram.hpp>
#include <slamantic/DynamicsFactor.hpp>

namespace ORB_SLAM2
{

  bool MapPoint::mDisableDynamicFactor = false;

  long unsigned int MapPoint::nNextId = 0;
  mutex             MapPoint::mGlobalMutex;

  MapPoint::MapPoint(const cv::Mat& Pos, KeyFrame *pRefKF, Map *pMap)
    : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
      mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
      mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
  {
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
    mDynamicsFactor=0;
  }

  MapPoint::MapPoint(const cv::Mat& Pos, Map *pMap, Frame *pFrame, const int& idxF)
    : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
      mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0), mnCorrectedReference(0),
      mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
      mpReplaced(NULL), mpMap(pMap)
  {
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / cv::norm(mNormalVector);

    cv::Mat     PC               = Pos - Ow;
    const float dist             = cv::norm(PC);
    const int   level            = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int   nLevels          = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);
    mDynamicsFactor=0;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
  }

  void MapPoint::SetWorldPos(const cv::Mat& Pos)
  {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
  }

  cv::Mat MapPoint::GetWorldPos()
  {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
  }

  cv::Mat MapPoint::GetNormal()
  {
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
  }

  KeyFrame *MapPoint::GetReferenceKeyFrame()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
  }

  void MapPoint::AddObservation(KeyFrame *pKF, size_t idx)
  {
    {
      unique_lock<mutex> lock(mMutexFeatures);

      if(mObservations.count(pKF))
        return;

      mObservations[pKF] = idx;

      if(pKF->mvuRight[idx] >= 0)
        nObs += 2;
      else
      {
        nObsMono++;
        nObs++;
      }
    }

    mpLatestKF = pKF;

    if(mDynamicOutlier)
    {
      handleDynamicOutlier();
    }
  }

  void MapPoint::EraseObservation(KeyFrame *pKF)
  {
    bool bBad = false;
    {
      unique_lock<mutex> lock(mMutexFeatures);
      if(mObservations.count(pKF))
      {
        int idx = mObservations[pKF];
        if(pKF->mvuRight[idx] >= 0)
          nObs -= 2;
        else
        {
          nObsMono--;
          nObs--;
        }

        mObservations.erase(pKF);

        if(mpRefKF == pKF)
          mpRefKF = mObservations.begin()->first;

        if(mpLatestKF == pKF)
        {
          mpLatestKF = mpRefKF;
        }

//        if(!mDynamicOutlier){
        // If only 2 observations or less, discard point
        if(nObs <= 2)
          bBad = true;
//        }
      }
    }

    if(bBad)
      SetBadFlag();
  }

  map<KeyFrame *, size_t> MapPoint::GetObservations()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
  }

  int MapPoint::Observations()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
  }

  void MapPoint::SetBadFlag()
  {
    map<KeyFrame *, size_t> obs;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      unique_lock<mutex> lock2(mMutexPos);
      mbBad = true;
      obs   = mObservations;
      mObservations.clear();
    }

    for(map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
      KeyFrame *pKF = mit->first;
      pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
  }

  MapPoint *MapPoint::GetReplaced()
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
  }

  void MapPoint::Replace(MapPoint *pMP)
  {
    if(pMP->mnId == this->mnId)
      return;

    int                     nvisible, nfound;
    map<KeyFrame *, size_t> obs;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      unique_lock<mutex> lock2(mMutexPos);
      obs = mObservations;
      mObservations.clear();
      mbBad      = true;
      nvisible   = mnVisible;
      nfound     = mnFound;
      mpReplaced = pMP;
    }

    for(map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
      // Replace measurement in keyframe
      KeyFrame *pKF = mit->first;

      if(!pMP->IsInKeyFrame(pKF))
      {
        pKF->ReplaceMapPointMatch(mit->second, pMP);
        pMP->AddObservation(pKF, mit->second);
      }
      else
      {
        pKF->EraseMapPointMatch(mit->second);
      }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
  }

  bool MapPoint::isBad()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
  }

  void MapPoint::IncreaseVisible(int n)
  {
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible += n;
  }

  void MapPoint::IncreaseFound(int n)
  {
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound += n;
  }

  float MapPoint::GetFoundRatio()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
  }

  void MapPoint::ComputeDistinctiveDescriptors()
  {
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame *, size_t> observations;

    {
      unique_lock<mutex> lock1(mMutexFeatures);
      if(mbBad)
        return;
      observations = mObservations;
    }

    if(observations.empty())
      return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
    {
      KeyFrame *pKF = mit->first;

      if(!pKF->isBad())
        vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
      return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float      Distances[N][N];
    for(size_t i   = 0; i < N; i++)
    {
      Distances[i][i] = 0;
      for(size_t j = i + 1; j < N; j++)
      {
        int distij      = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
        Distances[i][j] = distij;
        Distances[j][i] = distij;
      }
    }

    // Take the descriptor with least median distance to the rest
    int        BestMedian = INT_MAX;
    int        BestIdx    = 0;
    for(size_t i          = 0; i < N; i++)
    {
      vector<int> vDists(Distances[i], Distances[i] + N);
      sort(vDists.begin(), vDists.end());
      int median = vDists[0.5 * (N - 1)];

      if(median < BestMedian)
      {
        BestMedian = median;
        BestIdx    = i;
      }
    }

    {
      unique_lock<mutex> lock(mMutexFeatures);
      mDescriptor = vDescriptors[BestIdx].clone();
    }
  }

  cv::Mat MapPoint::GetDescriptor()
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
  }
  int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
  {
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
      return mObservations[pKF];
    else
      return -1;
  }

  bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
  {
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
  }

  void MapPoint::UpdateNormalAndDepth()
  {
    map<KeyFrame *, size_t> observations;
    KeyFrame                *pRefKF;
    cv::Mat                 Pos;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      unique_lock<mutex> lock2(mMutexPos);
      if(mbBad)
        return;
      observations = mObservations;
      pRefKF       = mpRefKF;
      Pos          = mWorldPos.clone();
    }

    if(observations.empty())
      return;

    cv::Mat                               normal = cv::Mat::zeros(3, 1, CV_32F);
    int                                   n      = 0;
    for(map<KeyFrame *, size_t>::iterator mit    = observations.begin(), mend = observations.end(); mit != mend; mit++)
    {
      KeyFrame *pKF    = mit->first;
      cv::Mat  Owi     = pKF->GetCameraCenter();
      cv::Mat  normali = mWorldPos - Owi;
      normal = normal + normali / cv::norm(normali);
      n++;
    }

    cv::Mat     PC               = Pos - pRefKF->GetCameraCenter();
    const float dist             = cv::norm(PC);
    const int   level            = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->mvScaleFactors[level];
    const int   nLevels          = pRefKF->mnScaleLevels;

    {
      unique_lock<mutex> lock3(mMutexPos);
      mfMaxDistance = dist * levelScaleFactor;
      mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
      mNormalVector = normal / n;
    }

    {
      computeLabelProbability();
      computeDynamicsFactor();
    }

  }

  float MapPoint::GetMinDistanceInvariance()
  {
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
  }

  float MapPoint::GetMaxDistanceInvariance()
  {
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
  }

  int MapPoint::PredictScale(const float& currentDist, KeyFrame *pKF)
  {
    float ratio;
    {
      unique_lock<mutex> lock(mMutexPos);
      ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
    if(nScale < 0)
      nScale = 0;
    else if(nScale >= pKF->mnScaleLevels)
      nScale = pKF->mnScaleLevels - 1;

    return nScale;
  }

  int MapPoint::PredictScale(const float& currentDist, Frame *pF)
  {
    float ratio;
    {
      unique_lock<mutex> lock(mMutexPos);
      ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
    if(nScale < 0)
      nScale = 0;
    else if(nScale >= pF->mnScaleLevels)
      nScale = pF->mnScaleLevels - 1;

    return nScale;
  }


  void MapPoint::computeDynamicsFactor()
  {
    if(MapPoint::mDisableDynamicFactor)
    {
      mDynamicsFactor = 0;
      return;
    }

    if(mDynamicOutlier)
    {
      mDynamicsFactor = slamantic::DYNAMICS_FACTOR_DYNAMIC;
      return;
    }

    if(mObservations.empty())
    {
      mDynamicsFactor = 0;
      return;
    }

    // compute number of observations on different points in time
    // nObs is geometric observations
    int nObsStereo = (nObs - nObsMono) / 2;
    int nObsDf     = nObsStereo + nObsMono;
    if(nObsStereo == 0)
    {
      // if no stereo observation present, point was triangulated from mono thus reduce point in time
      nObsDf--;
    }

    double const k = 20;
    mDynamicsFactor = slamantic::computeDynamicsFactor(nObsDf,
                                                       mSemanticDescriptor.labelDynamicsFactor,
                                                       mSemanticDescriptor.probability,
                                                       k);
  }


  void MapPoint::computeLabelProbability()
  {
    if(mDisableDynamicFactor){
      return;
    }

    // if label is mature do not change label
    double const matureLabelProbability = 0.9;
    size_t const matureLabelCnt         = 20;
    if(mSemanticDescriptor.probability >= matureLabelProbability && mSemanticDescriptor.totalLabelCnt >= matureLabelCnt)
    {
      return;
    }

    // if the label probability is very low after 5 label measurements; try to relabel from observations()
    // or if it is the initial labeling
    size_t const relabelMinCnt    = 5;
    double const relabelThreshold = 0.6;

    if((mSemanticDescriptor.probability < relabelThreshold && mSemanticDescriptor.totalLabelCnt > relabelMinCnt)
       || mSemanticDescriptor.totalLabelCnt == 1)
    {
      // build histogram of labels
      slamantic::LabelHistogram labelHist;
      slamantic::SemanticPtr    pSemantic;
      float                     obsLabelProbability = 1.0;

      for(auto const& pObs : mObservations)
      {
        const KeyFrame *pFrame = pObs.first;
        if(!pFrame->mpSemanticImage)
        {
          continue;
        }
        pSemantic = pFrame->mpSemanticImage->getSemantic();
        slamantic::LabelId labelId = pFrame->mpSemanticImage->getLabelIdAt(pFrame->mvKeys[pObs.second].pt);
        labelHist.insert(labelId);
        obsLabelProbability = pFrame->mpSemanticImage->getProbabilityAt(pFrame->mvKeys[pObs.second].pt);
      }

      auto const& labelMax = labelHist.max_element();
      if(labelHist.size() == 0)
      {
        return;
      }

      DCHECK(pSemantic);

      // store dynamcis factor of label
      mSemanticDescriptor.labelDynamicsFactor = pSemantic->getLabel(labelMax.first).dynamicsFactor;
      // retreive values from histogram
      mSemanticDescriptor.init(labelMax.first, labelMax.second, labelHist.size_total());
      // in case of single lableing, use provided probability
      if(mSemanticDescriptor.totalLabelCnt == 1)
      {
        DCHECK(obsLabelProbability > 0.0);
        mSemanticDescriptor.probability = obsLabelProbability;
      }
    }
  }
  void MapPoint::handleDynamicOutlier()
  {
    // set as permanent outlier
    mDynamicOutlier = true;

    std::map<KeyFrame *, size_t> obs;
    {
      unique_lock<mutex> lock1(mMutexFeatures);
      obs = mObservations;
    }
    // delete all observations except the reference and the latest one
    for(auto const& ob : obs)
    {
      ob.first->setDynamic(ob.second);
      if(ob.first != mpLatestKF && ob.first != mpRefKF)
      {
        EraseObservation(ob.first);
        ob.first->EraseMapPointMatch(ob.second);
      }
    }

    // latest becomes new reference
    {
      unique_lock<mutex> lock(mMutexFeatures);
      mpRefKF = mpLatestKF;
    }
  }

} //namespace ORB_SLAM
