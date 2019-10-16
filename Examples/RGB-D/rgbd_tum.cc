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
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <zconf.h>

INITIALIZE_EASYLOGGINGPP

using namespace std;

void LoadImages(const string& strAssociationFilename,
                vector<string>& vstrImageFilenamesRGB,
                vector<string>& vstrImageFilenamesD,
                vector<double>& vTimestamps);

int main(int argc, char **argv)
{
  if(argc != 6)
  {
    cerr << endl
         << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_semantic_labels path_to_sequence path_to_association "
         << endl;
    return 1;
  }

  std::string path_to_vocabulary(argv[1]);
  std::string path_to_settings(argv[2]);
  std::string path_to_semantic_labels(argv[3]);
  std::string path_to_sequence(argv[4]);
  std::string path_to_associations(argv[5]);

  CHECK(std::ifstream(path_to_vocabulary).good()) << "could not open " << path_to_vocabulary;
  CHECK(std::ifstream(path_to_settings).good()) << "could not open " << path_to_settings;
  CHECK(std::ifstream(path_to_semantic_labels).good()) << "could not open " << path_to_semantic_labels;
  CHECK(std::ifstream(path_to_associations).good()) << "could not open " << path_to_associations;

  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  string         strAssociationFilename = string(path_to_associations);
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if(vstrImageFilenamesRGB.empty())
  {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  }
  else if(vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
  {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::RGBD, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  slamantic::SemanticPtr pSemantic;
  std::string semantic_dir;
  if(SLAM.isSemanticEnabled())
  {
    pSemantic = std::make_shared<slamantic::Semantic>();
    pSemantic->loadFromYaml(path_to_semantic_labels);
    semantic_dir = std::string(path_to_sequence) + "/" + pSemantic->getOptions().algorithm + "/";
  }

  // Main loop
  cv::Mat imRGB, imD, imLabelId;
  for(int ni               = 0; ni < nImages; ni++)
  {
    // Read image and depthmap from file
    imRGB = cv::imread(string(path_to_sequence) + "/" + vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
    imD   = cv::imread(string(path_to_sequence) + "/" + vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni];

    if(imRGB.empty() || imD.empty())
    {
      cerr << endl << "Failed to load image at: " << string(path_to_sequence) << "/" << vstrImageFilenamesRGB[ni]
           << endl;
      return 1;
    }

    slamantic::SemanticImagePtr pSemanticImage;
    if(pSemantic)
    {
      std::string semantic_file = semantic_dir + vstrImageFilenamesRGB[ni];
      imLabelId = cv::imread(semantic_file, cv::IMREAD_UNCHANGED);
      LOG_IF(imLabelId.empty(), WARNING) << "failed to load semantic image " << semantic_file;
      pSemanticImage = std::make_shared<slamantic::SemanticImageId>(imLabelId, pSemantic);
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the image to the SLAM system
    SLAM.TrackRGBD(imRGB, imD, tframe, pSemanticImage);

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
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void LoadImages(const string& strAssociationFilename,
                vector<string>& vstrImageFilenamesRGB,
                vector<string>& vstrImageFilenamesD,
                vector<double>& vTimestamps)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());

  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation, s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);

    }
  }
}
