//
// copyright Matthias Schoerghuber (AIT)
//
#include <gtest/gtest.h>

#include <slamantic/DynamicsFactor.hpp>

TEST(slamantic, dynamicsFactor)
{ // no label information (use with first observation)
  EXPECT_EQ(0.5, slamantic::computeDynamicsFactor(1, 0, 0, 20));
  EXPECT_GE(0.5, slamantic::computeDynamicsFactor(2, 0, 0, 20));
  EXPECT_GE(0.25, slamantic::computeDynamicsFactor(3, 0, 0, 20));
  EXPECT_EQ(0.0, slamantic::computeDynamicsFactor(4, 0, 0, 20));

// car highprop (use with 4th observation)
  EXPECT_EQ(0.75, slamantic::computeDynamicsFactor(1, 0.5, 1.0, 20));
  EXPECT_LE(0.5, slamantic::computeDynamicsFactor(2, 0.5, 1.0, 20));
  EXPECT_EQ(0.5, slamantic::computeDynamicsFactor(3, 0.5, 1.0, 20));
  EXPECT_EQ(0.5, slamantic::computeDynamicsFactor(4, 0.5, 1.0, 20));

// car lprop (use with 3th observation)
  EXPECT_LE(0.5, slamantic::computeDynamicsFactor(2, 0.5, 0.6, 20));
  EXPECT_LE(0.25, slamantic::computeDynamicsFactor(3, 0.5, 0.6, 20));

// static label hprop
  EXPECT_GT(0.25, slamantic::computeDynamicsFactor(2, -1.0, 1.0, 20));
  EXPECT_GT(0.25, slamantic::computeDynamicsFactor(3, -1.0, 0.7, 20));

// test sturation
  EXPECT_EQ(0.5 * 0.6, slamantic::computeDynamicsFactor(8, 0.5, 0.6, 20));
  EXPECT_EQ(0.5, slamantic::computeDynamicsFactor(8, 0.5, 1.0, 20));

  EXPECT_FALSE(slamantic::isDfDynamic(slamantic::DYNAMICS_FACTOR_STATIC_DYNAMIC));
  EXPECT_FALSE(slamantic::isDfDynamic(slamantic::DYNAMICS_FACTOR_STATIC_DYNAMIC - 0.1));
  EXPECT_TRUE(slamantic::isDfDynamic(slamantic::DYNAMICS_FACTOR_STATIC_DYNAMIC + 0.1));
  EXPECT_TRUE(slamantic::isDfDynamic(1.0));

  EXPECT_TRUE(slamantic::isDfStatic(slamantic::DYNAMICS_FACTOR_STATIC));
  EXPECT_FALSE(slamantic::isDfStatic(slamantic::DYNAMICS_FACTOR_STATIC + 0.1));
  EXPECT_TRUE(slamantic::isDfStatic(slamantic::DYNAMICS_FACTOR_STATIC - 0.1));
  EXPECT_TRUE(slamantic::isDfStatic(0.0));


  EXPECT_TRUE(slamantic::isDfPotentialDynamic(0.49));
  EXPECT_TRUE(slamantic::isDfPotentialDynamic(0.26));
  EXPECT_TRUE(slamantic::isDfPotentialDynamic(slamantic::DYNAMICS_FACTOR_STATIC_DYNAMIC));
  EXPECT_FALSE(slamantic::isDfPotentialDynamic(slamantic::DYNAMICS_FACTOR_STATIC));
  EXPECT_FALSE(slamantic::isDfPotentialDynamic(0.51));
}
