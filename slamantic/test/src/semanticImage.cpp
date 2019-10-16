//
// copyright Matthias Schoerghuber (AIT)
//
#include <gtest/gtest.h>

#include <opencv2/highgui.hpp>

#include <slamantic/Semantic.hpp>
#include <slamantic/SemanticImage.hpp>


TEST(slamantic, semanticImage)
{

  std::string compile_time_test_dir(SLAMANTIC_COMPILE_TEST_DATA);

  cv::Mat     imgId = cv::imread(compile_time_test_dir + "/data/labelIds_frankfurt_000001_011245_leftImg8bit.png", CV_LOAD_IMAGE_UNCHANGED);
  CHECK(!imgId.empty());

  cv::Mat     imgRGB = cv::imread(compile_time_test_dir + "/data/labelColor_frankfurt_000001_011245_leftImg8bit.png", CV_LOAD_IMAGE_UNCHANGED);
  CHECK(!imgRGB.empty());

  cv::Mat     imgProb8U = cv::imread(compile_time_test_dir + "/data/labelProp_frankfurt_000001_011245_leftImg8bit.png", CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat imgProb;
  CHECK(!imgProb8U.empty());
  imgProb8U.convertTo(imgProb, CV_32FC1, 1. / 254);

  slamantic::SemanticPtr pSemantic = std::make_shared<slamantic::Semantic>();
  pSemantic->loadFromYaml(compile_time_test_dir + "/data/labels-cityscapes.yaml");

//  semanticImage()
  slamantic::SemanticImageId semId(imgId,pSemantic);
  EXPECT_EQ(semId.getType(), slamantic::SemanticImageType::ID);
  slamantic::SemanticImageRGB semRGB(imgRGB,pSemantic);
  EXPECT_EQ(semRGB.getType(), slamantic::SemanticImageType::COLOR);

  semId.setProbabilities(imgProb);

  EXPECT_EQ(semId.getLabelIdAt(cv::Point2i(406,182)), 13); //car
  EXPECT_NEAR(semId.getProbabilityAt(cv::Point2i(406,182)), 0.7, 0.1); //person

  EXPECT_EQ(semId.getLabelIdAt(cv::Point2i(406,181)), 2); //building
  EXPECT_EQ(semId.getLabelIdAt(cv::Point2i(850,200)), 11); //person
  EXPECT_GT(semId.getProbabilityAt(cv::Point2i(850,200)), 0.9); //person

  EXPECT_EQ(semRGB.getLabelIdAt(cv::Point2i(406,184)), 13); //car
  EXPECT_EQ(semRGB.getLabelIdAt(cv::Point2i(406,183)), 2); //building
  EXPECT_EQ(semId.getLabelIdAt(cv::Point2i(850,200)), 11); //person
}