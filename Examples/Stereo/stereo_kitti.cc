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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <zconf.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP


void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimestamps);


slamantic::SemanticImagePtr LoadSemanticImage(std::string const& path_to_sequence,
                                              std::string const& frameName,
                                              cv::Mat const& imLeft,
                                              slamantic::SemanticPtr const& pSemantic)
{

  slamantic::SemanticImagePtr pSemanticImage;

  std::string semantic_dir       = path_to_sequence + "/labelIds/";
  std::string semantic_prop_dir  = path_to_sequence + "/labelProbabilities/";
  std::string semantic_file      = semantic_dir + frameName + ".png";
  std::string semantic_prob_file = semantic_prop_dir + frameName + ".png";

  cv::Mat imLabelIdSrc;
  cv::Mat imProbFileSrcU8;
  cv::Mat imProbFileU8;
  cv::Mat imLabelId;
  cv::Mat imProbFile;

  imLabelIdSrc    = cv::imread(semantic_file, cv::IMREAD_UNCHANGED);
  imProbFileSrcU8 = cv::imread(semantic_prob_file, cv::IMREAD_UNCHANGED);

  if(imLabelIdSrc.empty())
  {
    LOG(ERROR) << "Failed to load image: " << semantic_file;
    return pSemanticImage;
  }

  cv::resize(imLabelIdSrc, imLabelId, cv::Size(imLeft.cols, imLeft.rows), 0, 0, cv::INTER_NEAREST);
  cv::resize(imProbFileSrcU8, imProbFileU8, cv::Size(imLeft.cols, imLeft.rows), 0, 0, cv::INTER_NEAREST);
  imProbFileU8.convertTo(imProbFile, CV_32FC1, 1. / 254);

//      cv::imshow("prob", imProbFile);
//      cv::imshow("mask", imLabelId);

  pSemanticImage = std::make_shared<slamantic::SemanticImageId>(imLabelId, pSemantic);
  if(!imProbFile.empty())
  {
    pSemanticImage->setProbabilities(imProbFile);
  }

  return pSemanticImage;
}


int main(int argc, char **argv)
{
  if(argc != 5)
  {
    cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_semantic_labels path_to_sequence"
         << endl;
    return 1;
  }

  std::string path_to_vocabulary(argv[1]);
  std::string path_to_settings(argv[2]);
  std::string path_to_semantic_labels(argv[3]);
  std::string path_to_sequence(argv[4]);

  CHECK(std::ifstream(path_to_vocabulary).good()) << "could not open " << path_to_vocabulary;
  CHECK(std::ifstream(path_to_settings).good()) << "could not open " << path_to_settings;
  CHECK(std::ifstream(path_to_semantic_labels).good()) << "could not open " << path_to_semantic_labels;

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(string(path_to_sequence), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::STEREO, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;


  slamantic::SemanticPtr pSemantic;
  std::string            semantic_dir;
  if(SLAM.isSemanticEnabled())
  {
    pSemantic = std::make_shared<slamantic::Semantic>();
    pSemantic->loadFromYaml(path_to_semantic_labels);
    semantic_dir = string(path_to_sequence) + "/" + pSemantic->getOptions().algorithm + "/";
  }

  // Main loop
  cv::Mat imLeft, imRight, imLabelId;
  for(int ni        = 0; ni < nImages; ni++)
  {
    // Read left and right images from file
    imLeft  = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni];

    if(imLeft.empty())
    {
      cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

    slamantic::SemanticImagePtr pSemanticImage;
    // if semantic is enabled
    if(SLAM.isSemanticEnabled())
    {
      stringstream ss;
      ss << setfill('0') << setw(6) << ni;
      pSemanticImage = LoadSemanticImage(semantic_dir, ss.str(), imLeft, pSemantic);
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, tframe, "", pSemanticImage);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if(ni < nImages - 1)
      T = vTimestamps[ni + 1] - tframe;
    else if(ni > 0)
      T = tframe - vTimestamps[ni - 1];

    if(ttrack < T)
      usleep((T - ttrack) * 1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float   totaltime = 0;
  for(int ni        = 0; ni < nImages; ni++)
  {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
  SLAM.SaveTrajectoryTUM("poses.txt");

  return 0;
}

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimestamps)
{
  ifstream fTimes;
  string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while(!fTimes.eof())
  {
    string s;
    getline(fTimes, s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft  = strPathToSequence + "/image_0/";
  string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for(int i = 0; i < nTimes; i++)
  {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}
