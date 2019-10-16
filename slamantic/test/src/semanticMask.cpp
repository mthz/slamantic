//
// copyright Matthias Schoerghuber (AIT)
//
#include <gtest/gtest.h>

#include <opencv2/highgui.hpp>

#include <slamantic/Semantic.hpp>
#include <slamantic/SemanticImage.hpp>
#include <slamantic/SemanticMask.hpp>


TEST(slamantic, semanticMask)
{
  std::string compile_time_test_dir(SLAMANTIC_COMPILE_TEST_DATA);

  slamantic::SemanticPtr pSemantic = std::make_shared<slamantic::Semantic>();
  pSemantic->loadFromYaml(compile_time_test_dir + "/data/labels-cityscapes.yaml");

  // construct dummy image
  int     num_total = 20 * 20;
  int     num_label = 20 * 5;
  cv::Mat imgId     = cv::Mat::zeros(20, 20, CV_8U);
  // set row 1-5 to person
  imgId.rowRange(0, 5)  = 11;
  // set row 5-10 to car
  imgId.rowRange(5, 10) = 13;

  slamantic::SemanticImageId semId(imgId, pSemantic);


  // generate person mask

  slamantic::SemanticMask semMaskPerson(semId, [](slamantic::Label const& label)
  {
    return label.id == 11;
  });
  int num_dynamic_p = num_total - num_label;
  EXPECT_EQ(cv::countNonZero(semMaskPerson.getMask()), num_dynamic_p);

  semMaskPerson.erode(3);
  EXPECT_EQ(cv::countNonZero(semMaskPerson.getMask()),num_dynamic_p-20);


  // generate mask based on dynamics factor

  slamantic::SemanticMask semMaskDynamics(semId, [](slamantic::Label const& label)
  {
    return label.dynamicsFactor >= 0.5;
  });
  int num_dynamic_d = num_total - 2 * num_label;
  EXPECT_EQ(cv::countNonZero(semMaskDynamics.getMask()), num_dynamic_d);

  // dilate
  semMaskDynamics.dilate(3);
  EXPECT_EQ(cv::countNonZero(semMaskDynamics.getMask()),num_dynamic_d+20);

//  cv::imshow("semanti", semMaskDynamics.getMask());
//  cv::waitKey(0);
}