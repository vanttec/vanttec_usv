#include <gtest/gtest.h>
#include "usv_control/planning/path_post_processor.hpp"
#include "usv_control/planning/a_star_planner.hpp"

class PostProcessorTest : public ::testing::Test {
protected:
  GridMap map_{0.0, 0.0, 1.0, 10, 10};
  PathPostProcessor pp_{100};
  std::vector<Cell> row0_ = {{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6}};
};

TEST_F(PostProcessorTest, ShortPathUnchanged) {
  std::vector<Cell> path = {{0,0},{0,1}};
  EXPECT_EQ(pp_.shortcut(map_, path), path);
}

TEST_F(PostProcessorTest, StraightLineCollapsesToEndpoints) {
  auto out = pp_.shortcut(map_, row0_);
  EXPECT_EQ(out, (std::vector<Cell>{{0,0},{0,6}}));
}

TEST_F(PostProcessorTest, AStarStaircaseCollapsesOnFreeMap) {
  auto path = AStarPlanner{1.0}.plan(map_, {0,0}, {3,6});
  ASSERT_TRUE(path.has_value());
  auto out = pp_.shortcut(map_, *path);
  EXPECT_EQ(out, (std::vector<Cell>{{0,0},{3,6}}));
}

TEST_F(PostProcessorTest, WallKeepsIntermediateVertices) {
  for (int i = 0; i < 10; ++i) if (i != 7) map_.setCost(i, 5, GridMap::LETHAL);
  auto path = AStarPlanner{1.0}.plan(map_, {0,0}, {0,9});
  ASSERT_TRUE(path.has_value());
  auto out = pp_.shortcut(map_, *path);
  EXPECT_EQ(out.front(), (Cell{0,0}));
  EXPECT_EQ(out.back(),  (Cell{0,9}));
  EXPECT_GE(out.size(), 3u);            // no puede atravesar la pared
  EXPECT_LT(out.size(), path->size());  // pero sí simplifica
}

TEST_F(PostProcessorTest, ThresholdBlocksShortcutThroughCostlyCell) {
  map_.setCost(0, 3, 200);  // arriba del umbral (100), no letal
  auto out = pp_.shortcut(map_, row0_);
  EXPECT_EQ(out, (std::vector<Cell>{{0,0},{0,2},{0,3},{0,6}}));
}

TEST_F(PostProcessorTest, HighThresholdAllowsSameShortcut) {
  map_.setCost(0, 3, 200);
  auto out = PathPostProcessor{250}.shortcut(map_, row0_);
  EXPECT_EQ(out, (std::vector<Cell>{{0,0},{0,6}}));
}

TEST_F(PostProcessorTest, LethalStartDoesNotBlock) {
  map_.setCost(0, 0, GridMap::LETHAL);
  auto out = pp_.shortcut(map_, row0_);
  EXPECT_EQ(out, (std::vector<Cell>{{0,0},{0,6}}));
}