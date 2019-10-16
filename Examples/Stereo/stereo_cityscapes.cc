//
// copyright Matthias Schoerghuber (AIT)
//

#include <unistd.h>

#include<iostream>
#include <iomanip>
#include <algorithm>
#include <chrono>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <Converter.h>

#include <easylogging++.h>

INITIALIZE_EASYLOGGINGPP

using namespace std;

slamantic::SemanticImagePtr LoadSemanticImage(std::string const& path_to_sequence,
                                              std::string const& frameName,
                                              cv::Mat const& imLeft,
                                              slamantic::SemanticPtr const& pSemantic)
{

  slamantic::SemanticImagePtr pSemanticImage;

  std::string semantic_dir       = path_to_sequence + "/labelIds/";
  std::string semantic_prop_dir  = path_to_sequence + "/labelProbabilities/";
  std::string semantic_file      = semantic_dir + frameName + "_leftImg8bit.png";
  std::string semantic_prob_file = semantic_prop_dir + frameName + "_leftImg8bit.png";

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

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<string>& vstrNames,
                vector<double>& vTimestamps);

int main(int argc, char **argv)
{
  if(argc < 4)
  {
    cerr << endl
         << "Usage: ./stereo_cityscapes path_to_vocabulary path_to_settings path_to_semantic_labels path_to_sequence [startn nframes [record]]"
         << endl;
    return 1;
  }

  std::string path_to_vocabulary(argv[1]);
  std::string path_to_settings(argv[2]);
  std::string path_to_semantic_labels(argv[3]);
  std::string path_to_sequence(argv[4]);

  int  startn = argc > 5 ? stoi(argv[5]) : 0;
  int  stopn  = argc > 6 ? startn + stoi(argv[6]) : 0;
  bool record = argc > 7 && std::string(argv[7]) == "true";

  CHECK(std::ifstream(path_to_vocabulary).good()) << "could not open " << path_to_vocabulary;
  CHECK(std::ifstream(path_to_settings).good()) << "could not open " << path_to_settings;
  CHECK(std::ifstream(path_to_semantic_labels).good()) << "could not open " << path_to_semantic_labels;
  LOG_IF(record, INFO) << "recording activated from command line";
  LOG_IF(!record && argc > 7, INFO) << "recording NOT activated, set argument to >true< to activate";


  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<string> vstrNames;
  vector<double> vTimestamps;
  LoadImages(string(path_to_sequence), vstrImageLeft, vstrImageRight, vstrNames, vTimestamps);

  const int nImages = vstrImageLeft.size();
  CHECK(nImages > 0) << "no images in sequence";

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::STEREO, true);
  //  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack(nImages);

  LOG(INFO) << std::endl << "--------------" << std::endl << "Start processing sequence ..." << std::endl
            << "Images in the sequence: " << nImages;

  // debug output
  std::ofstream oOdometryTUM("poses_odometry.txt");


  if(record)
  {
    SLAM.StartRecording();
  }

  slamantic::SemanticPtr pSemantic;
  std::string            semantic_dir;
  if(SLAM.isSemanticEnabled())
  {
    pSemantic = std::make_shared<slamantic::Semantic>();
    pSemantic->loadFromYaml(path_to_semantic_labels);
    semantic_dir = string(path_to_sequence) + "/" + pSemantic->getOptions().algorithm + "/";
  }

  if(stopn == 0)
  {
    stopn = nImages;
  }

  // Main loop
  cv::Mat imLeft, imRight, imProbFile;
  for(int ni        = startn; ni < stopn; ni++)
  {
    // Read left and right images from file
    imLeft  = cv::imread(vstrImageLeft[ni]);
    imRight = cv::imread(vstrImageRight[ni]);


    if(imLeft.empty() || imRight.empty())
    {
      LOG(ERROR) << "Failed to load image: " << vstrImageLeft[ni] << "/" << vstrImageRight[ni];
      break;
    }

    // load semantic image
    slamantic::SemanticImagePtr pSemanticImage;
    // if semantic is enabled
    if(SLAM.isSemanticEnabled())
    {
      pSemanticImage = LoadSemanticImage(semantic_dir, vstrNames[ni], imLeft, pSemantic);
    }

    double tframe = vTimestamps[ni];

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    cv::Mat Tcw = SLAM.TrackStereo(imLeft, imRight, tframe, vstrNames[ni], pSemanticImage);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


    if(!Tcw.empty())
    {

      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

      vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
      oOdometryTUM << std::fixed << std::setprecision(9) << vTimestamps[ni] << " " << setprecision(9)
                   << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " "
                   << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    LOG_EVERY_N(100, INFO) << "tracking time: " << ttrack << "s";
    vTimesTrack[ni] = ttrack;

    // Wait to load the next framec
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

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("vkittiCameraTrajectory.txt");
  SLAM.SaveTrajectoryTUM("poses.txt");

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

/**
 * Read custom cityscape dataset structure
 * @param strPathToSequence
 * @param vstrImageLeft
 * @param vstrImageRight
 * @param vstrNames
 * @param vTimestamps
 */
void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<string>& vstrNames,
                vector<double>& vTimestamps)
{

  string strPathTimeFile = strPathToSequence + "/frames.txt";
  LOG(INFO) << "Load: " << strPathTimeFile;
  ifstream fFrames(strPathTimeFile);
  CHECK(fFrames.good()) << " could not open " << strPathTimeFile;

  string strPrefixLeft  = strPathToSequence + "/imageResized/";
  string strPrefixRight = strPathToSequence + "/imageRightResized/";

  double t = 0;
  int    i = 0;
  while(!fFrames.eof())
  {
    string s;
    getline(fFrames, s);

    if(s.empty())
    {
      LOG(WARNING) << "empty frame number on line " << i;
      i++;
      continue;
    }

    vstrNames.emplace_back(s);
    vstrImageLeft.emplace_back(strPrefixLeft + s + "_leftImg8bit.png");
    vstrImageRight.emplace_back(strPrefixRight + s + "_rightImg8bit.png");

    // read timestamp
    std::string   timestampFile = strPathToSequence + "/timestamp/" + s + "_timestamp.txt";
    std::ifstream fTimestamp(timestampFile);
    CHECK(fTimestamp.good());
    fTimestamp >> t;
    vTimestamps.emplace_back(t / 1E9);
  }
}
