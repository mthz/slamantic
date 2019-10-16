//
// copyright Matthias Schoerghuber (AIT)
//
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>

using namespace std;


#include<System.h>
#include <Converter.h>



INITIALIZE_EASYLOGGINGPP

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageLeft,
                vector<string>& vstrImageRight,
                vector<double>& vTimestamps);

int main(int argc, char **argv)
{
  if(argc != 5)
  {
    cerr << endl << "Usage: ./stereo_vkitti path_to_vocabulary path_to_settings path_to_semantic_labels path_to_sequence" << endl;
    return 1;
  }

  std::string path_to_vocabulary(argv[1]);
  std::string path_to_settings(argv[2]);
  std::string path_to_semantic_labels(argv[3]);
  std::string path_to_sequence(argv[4]);

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
    semantic_dir = string(path_to_sequence) + "/frames/classSegmentation/Camera_0/";
  }

  cv::Mat imLabelId;
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


    slamantic::SemanticImagePtr pSemanticImage;
    if(SLAM.isSemanticEnabled()){
      stringstream ss;
      ss << "classgt_" << setfill('0') << setw(5) << ni;
      std::string semantic_file = semantic_dir + ss.str() + ".png";
      imLabelId = cv::imread(semantic_file, cv::IMREAD_UNCHANGED);
      pSemanticImage = std::make_shared<slamantic::SemanticImageRGB>(imLabelId, pSemantic);
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    cv::Mat Tcw = SLAM.TrackStereo(imLeft, imRight, tframe,"",pSemanticImage);


    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

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
  string   strPathTimeFile = strPathToSequence + "/extrinsic_camera0.txt";
  fTimes.open(strPathTimeFile.c_str());
  CHECK(fTimes.good());
  double t = 0;
  while(!fTimes.eof())
  {
    string s;
    getline(fTimes, s);
    if(!s.empty())
    {
      vTimestamps.push_back(t);
      t += 0.01;
    }
  }

  string strPrefixLeft  = strPathToSequence + "/frames/rgb/Camera_0/rgb_";
  string strPrefixRight = strPathToSequence + "/frames/rgb/Camera_1/rgb_";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for(int i = 0; i < nTimes; i++)
  {
    stringstream ss;
    ss << setfill('0') << setw(5) << i;
    vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".jpg";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".jpg";
  }
}
