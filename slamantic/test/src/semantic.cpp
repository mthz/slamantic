//
// copyright Matthias Schoerghuber (AIT)
//
#include <gtest/gtest.h>

#include <slamantic/Semantic.hpp>


TEST(slamantic, semantic)
{
  std::string compile_time_test_dir(SLAMANTIC_COMPILE_TEST_DATA);

  slamantic::Semantic semantic;
  semantic.loadFromYaml(compile_time_test_dir + "/data/labels-cityscapes.yaml");

  // test getter of semantic structure with the middle label
  slamantic::Label testLabel    = semantic.getLabels()[semantic.getLabels().size() / 2];
  auto             testLabelGot = semantic.getLabel(testLabel.id);
  EXPECT_EQ(semantic.getLabel(testLabel.id).id, testLabelGot.id);
  EXPECT_EQ(semantic.getLabelByColor(testLabel.color).id, testLabelGot.id);
  EXPECT_EQ(semantic.getLabelIdByColor(testLabel.color), testLabelGot.id);
  EXPECT_EQ(semantic.getLabelIdByName(testLabel.name), testLabelGot.id);

  // test with the maximmum label
  testLabel    = semantic.getLabels().back();
  testLabelGot = semantic.getLabel(testLabel.id);
  EXPECT_EQ(semantic.getLabel(testLabel.id).id, testLabelGot.id);
  EXPECT_EQ(semantic.getLabelByColor(testLabel.color).id, testLabelGot.id);
  EXPECT_EQ(semantic.getLabelIdByColor(testLabel.color), testLabelGot.id);
  EXPECT_EQ(semantic.getLabelIdByName(testLabel.name), testLabelGot.id);


  //Test a few labels explicitly
  slamantic::Label const& labelPerson = semantic.getLabel(11);
  EXPECT_TRUE(labelPerson.name == "Person");
  EXPECT_EQ(labelPerson.id, 11);
  EXPECT_TRUE(labelPerson.isThing);
  EXPECT_TRUE(labelPerson.dynamicsFactor != 0);
  EXPECT_EQ(semantic.getLabelIdByName("Person"), 11);

  slamantic::Label const& labelSky = semantic.getLabel(10);
  EXPECT_TRUE(labelSky.name == "Sky");
  EXPECT_EQ(labelSky.id, 10);
  EXPECT_FALSE(labelSky.isThing);
  EXPECT_EQ(labelSky.dynamicsFactor, 0);
  EXPECT_EQ(semantic.getLabelIdByName("Sky"), 10);

  // test Options read out
  EXPECT_TRUE(semantic.getOptions().algorithm == "test");
  EXPECT_TRUE(semantic.getOptions().showLabelBar);
  EXPECT_EQ(semantic.getOptions().type, slamantic::SemanticImageType::COLOR);
}