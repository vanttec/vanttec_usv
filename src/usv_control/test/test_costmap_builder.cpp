#include <gtest/gtest.h>
#include "usv_control/planning/costmap_builder.hpp"

class CostmapBuilderTest : public ::testing::Test {
protected:
  GridMap map_{-5.0, -5.0, 0.2, 50, 50};
  CostmapBuilder builder_{{0.5, 1.5, 2.0}};  // lethal, influence, decay
  Obstacle buoy_{0.1, 0.1, 0.15};
};

TEST_F(CostmapBuilderTest, BuoyCenterIsLethal) {
  builder_.build(map_, {buoy_});
  EXPECT_EQ(map_.getCost(25, 25), GridMap::LETHAL);
}

TEST_F(CostmapBuilderTest, InsideLethalRadiusIsLethal) {
  builder_.build(map_, {buoy_});
  EXPECT_EQ(map_.getCost(27, 25), GridMap::LETHAL);  // d al borde = 0.25
}

TEST_F(CostmapBuilderTest, GradientDecreasesWithDistance) {
  builder_.build(map_, {buoy_});
  uint8_t near = map_.getCost(29, 25);  // d = 0.65
  uint8_t far  = map_.getCost(31, 25);  // d = 1.05
  EXPECT_GT(near, GridMap::FREE);
  EXPECT_LT(near, GridMap::LETHAL);
  EXPECT_GT(near, far);
  EXPECT_GT(far, GridMap::FREE);
}

TEST_F(CostmapBuilderTest, BeyondInfluenceIsFree) {
  builder_.build(map_, {buoy_});
  EXPECT_EQ(map_.getCost(40, 25), GridMap::FREE);  // d = 2.85
}

TEST_F(CostmapBuilderTest, KeepsMaxCostNotLastStamped) {
  Obstacle second{1.3, 0.1, 0.15};
  builder_.build(map_, {buoy_, second});
  EXPECT_EQ(map_.getCost(27, 25), GridMap::LETHAL);  // la 2a boya no la "enfría"
}

TEST_F(CostmapBuilderTest, RebuildClearsOldObstacles) {
  builder_.build(map_, {buoy_});
  builder_.build(map_, {});
  EXPECT_EQ(map_.getCost(25, 25), GridMap::FREE);
}

TEST_F(CostmapBuilderTest, BuoyNearBorderDoesNotCrash) {
  builder_.build(map_, {Obstacle{-4.95, -4.95, 0.15}});
  EXPECT_EQ(map_.getCost(0, 0), GridMap::LETHAL);
}