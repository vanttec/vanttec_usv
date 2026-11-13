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
// ---------- Geocerca ----------
// Cuadrado de 6x6 m centrado en el origen; el mapa es de 10x10 m.
const std::vector<Point2D> kSquare{{-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0}};

TEST_F(CostmapBuilderTest, OutsideGeofenceIsLethal) {
  builder_.build(map_, {});
  builder_.stampGeofence(map_, kSquare);
  EXPECT_EQ(map_.getCost(2, 25), GridMap::LETHAL);   // x = -4.5, fuera
}

TEST_F(CostmapBuilderTest, GeofenceCenterIsFree) {
  builder_.build(map_, {});
  builder_.stampGeofence(map_, kSquare);
  EXPECT_EQ(map_.getCost(25, 25), GridMap::FREE);    // 3 m de la pared > influence
}

TEST_F(CostmapBuilderTest, GeofenceWallInflatesInside) {
  builder_.build(map_, {});
  builder_.stampGeofence(map_, kSquare);
  EXPECT_EQ(map_.getCost(10, 25), GridMap::LETHAL);  // x = -2.9, 0.1 m de la pared
  uint8_t near_wall = map_.getCost(14, 25);          // x = -2.1, 0.9 m de la pared
  EXPECT_GT(near_wall, GridMap::FREE);
  EXPECT_LT(near_wall, GridMap::LETHAL);
}

TEST_F(CostmapBuilderTest, GeofenceKeepsBuoys) {
  builder_.build(map_, {buoy_});
  builder_.stampGeofence(map_, kSquare);
  EXPECT_EQ(map_.getCost(25, 25), GridMap::LETHAL);  // la boya sigue ahí
}

TEST_F(CostmapBuilderTest, EmptyGeofenceChangesNothing) {
  builder_.build(map_, {});
  builder_.stampGeofence(map_, {});
  EXPECT_EQ(map_.getCost(0, 0), GridMap::FREE);
}

TEST_F(CostmapBuilderTest, StampDoesNotClear) {
  builder_.build(map_, {buoy_});
  builder_.stamp(map_, {Obstacle{3.0, 3.0, 0.15}});
  EXPECT_EQ(map_.getCost(25, 25), GridMap::LETHAL);  // la primera sigue
  EXPECT_EQ(map_.getCost(40, 40), GridMap::LETHAL);  // y se agregó la segunda
}
