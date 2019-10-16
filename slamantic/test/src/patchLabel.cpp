//
// copyright Matthias Schoerghuber (AIT)
//
#include <gtest/gtest.h>

#include <slamantic/PatchLabel.hpp>

TEST(slamantic,patchLabel){
  cv::Mat labelImg = cv::Mat::zeros(20,20,CV_8U);
  labelImg.at<std::uint8_t >(0,0) = 15;
  labelImg.at<std::uint8_t >(1,1) = 15;
  labelImg.at<std::uint8_t >(2,2) = 15;
  labelImg.at<std::uint8_t >(0,1) = 11;
  labelImg.at<std::uint8_t >(2,0) = 3;
  labelImg.at<std::uint8_t >(4,4) = 27;

  slamantic::SemanticImageId semanticImage(labelImg);
  slamantic::PatchLabel patch3(semanticImage,cv::Point2f(1,1),3);

  // test contains
  EXPECT_TRUE(patch3.contains(15));
  EXPECT_TRUE(patch3.contains(3));
  EXPECT_TRUE(patch3.contains(0));
  EXPECT_FALSE(patch3.contains(1));
  EXPECT_FALSE(patch3.contains(16));

  //  test label statistics
  EXPECT_EQ(patch3.getLabel(), 0); //< 0 is 4 times
  EXPECT_EQ(patch3.getLabelByRatio(0.75), 0); //< 15 is 3 times
  EXPECT_EQ(patch3.getLabelByRatio(0.74), slamantic::Label::invalid()); //< 15 is 3 times

  // larger patch
  slamantic::PatchLabel patch9(semanticImage,cv::Point2f(4,4),9);
  EXPECT_TRUE(patch9.contains(27));
  EXPECT_EQ(patch9.getLabel(), 0);

  // corner case
  slamantic::PatchLabel patch3c(semanticImage,cv::Point2f(0,0),3);
  EXPECT_EQ(patch3c.getLabel(), 15);
  EXPECT_TRUE(patch3c.contains(11));
  EXPECT_TRUE(patch3c.contains(0));
  EXPECT_FALSE(patch3c.contains(3));
}