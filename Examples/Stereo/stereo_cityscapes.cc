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

#include <unistd.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>

using namespace std;

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimestamps);

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  std::string   path    = argv[3];
  auto          sub     = path.rfind("/");
  std::string   seqname = path.substr(sub + 1, sub + 6);
  std::ofstream of("poses_" + seqname + ".txt  ");

  // Main loop
  cv::Mat imLeft, imRight;
  for(int ni            = 0; ni < nImages; ni++)
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

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    cv::Mat Tcw = SLAM.TrackStereo(imLeft, imRight, tframe);

    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    of << setprecision(6) << vTimestamps[ni] << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1)
       << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;


#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

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

  return 0;
}

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimestamps)
{
  ifstream fTimes;
  string   strPathTimeFile = strPathToSequence + "/frames.txt";
  fTimes.open(strPathTimeFile.c_str());


  string strPrefixLeft  = strPathToSequence + "/imageResized/";
  string strPrefixRight = strPathToSequence + "/imageRightResized/";

  double t = 0;
  while(!fTimes.eof())
  {

    string s;
    getline(fTimes, s);
    vstrImageLeft.push_back(strPrefixLeft + s + "_leftImg8bit.png");
    vstrImageRight.push_back(strPrefixRight + s + "_rightImg8bit.png");

    std::ifstream fTimestamp(strPathToSequence + "/timestamp/" + s + "_timestamp.txt");
    fTimestamp >> t;
    vTimestamps.push_back(t / 1E9);
//    t += 0.02;
  }
}
