#include <gtest/gtest.h>
#include <cstdlib>
#include "usv_control/planning/a_star_planner.hpp"

// Verifica las postcondiciones generales de un camino
static void expectValidPath(const GridMap& map, const std::vector<Cell>& path,
                            const Cell& start, const Cell& goal) {
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
  for (size_t k = 1; k < path.size(); ++k) {
    int di = std::abs(path[k].i - path[k - 1].i);
    int dj = std::abs(path[k].j - path[k - 1].j);
    EXPECT_TRUE(di <= 1 && dj <= 1 && (di + dj) > 0) << "saltó en k=" << k;
    EXPECT_NE(map.getCost(path[k].i, path[k].j), GridMap::LETHAL) << "letal en k=" << k;
  }
}

class AStarTest : public ::testing::Test {
protected:
  GridMap map_{0.0, 0.0, 1.0, 10, 10};
  AStarPlanner planner_{1.0};
};

TEST_F(AStarTest, StartEqualsGoal) {
  auto path = planner_.plan(map_, {3, 3}, {3, 3});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 1u);
}

TEST_F(AStarTest, StraightLineOnFreeMap) {
  auto path = planner_.plan(map_, {0, 0}, {0, 5});
  ASSERT_TRUE(path.has_value());
  expectValidPath(map_, *path, {0, 0}, {0, 5});
  EXPECT_EQ(path->size(), 6u);
}

TEST_F(AStarTest, PureDiagonalOnFreeMap) {
  auto path = planner_.plan(map_, {0, 0}, {4, 4});
  ASSERT_TRUE(path.has_value());
  expectValidPath(map_, *path, {0, 0}, {4, 4});
  EXPECT_EQ(path->size(), 5u);
}

TEST_F(AStarTest, GoalLethalReturnsEmpty) {
  map_.setCost(5, 5, GridMap::LETHAL);
  EXPECT_FALSE(planner_.plan(map_, {0, 0}, {5, 5}).has_value());
}

TEST_F(AStarTest, GoalOutsideReturnsEmpty) {
  EXPECT_FALSE(planner_.plan(map_, {0, 0}, {20, 0}).has_value());
}

TEST_F(AStarTest, FullWallReturnsEmpty) {
  for (int i = 0; i < 10; ++i) map_.setCost(i, 5, GridMap::LETHAL);
  EXPECT_FALSE(planner_.plan(map_, {0, 0}, {0, 9}).has_value());
}

TEST_F(AStarTest, WallWithGapGoesThroughGap) {
  for (int i = 0; i < 10; ++i) if (i != 7) map_.setCost(i, 5, GridMap::LETHAL);
  auto path = planner_.plan(map_, {0, 0}, {0, 9});
  ASSERT_TRUE(path.has_value());
  expectValidPath(map_, *path, {0, 0}, {0, 9});
  bool through_gap = false;
  for (const auto& c : *path) if (c == Cell{7, 5}) through_gap = true;
  EXPECT_TRUE(through_gap);
}

TEST_F(AStarTest, NoCornerCutting) {
  map_.setCost(1, 0, GridMap::LETHAL);
  map_.setCost(0, 1, GridMap::LETHAL);
  EXPECT_FALSE(planner_.plan(map_, {0, 0}, {1, 1}).has_value());
}

TEST_F(AStarTest, StartInLethalStillPlans) {
  map_.setCost(0, 0, GridMap::LETHAL);
  EXPECT_TRUE(planner_.plan(map_, {0, 0}, {0, 5}).has_value());
}

TEST(AStarCost, HighWeightAvoidsCostlyBand) {
  GridMap map{0.0, 0.0, 1.0, 10, 10};
  for (int j = 2; j <= 7; ++j) map.setCost(5, j, 200);  // caro pero no letal

  auto straight = AStarPlanner{0.0}.plan(map, {5, 0}, {5, 9});  // w = 0: ignora costo
  auto detour   = AStarPlanner{5.0}.plan(map, {5, 0}, {5, 9});  // w alto: lo evita

  ASSERT_TRUE(straight.has_value());
  ASSERT_TRUE(detour.has_value());
  bool straight_hits = false, detour_hits = false;
  for (const auto& c : *straight) if (c == Cell{5, 4}) straight_hits = true;
  for (const auto& c : *detour)   if (c == Cell{5, 4}) detour_hits = true;
  EXPECT_TRUE(straight_hits);
  EXPECT_FALSE(detour_hits);
}
// El bote está DENTRO de una mancha letal (cerca de una boya): sus 8 vecinas también son letales.
TEST_F(AStarTest, StartDeepInLethalEscapes) {
  for (int i = 0; i <= 2; ++i)
    for (int j = 0; j <= 2; ++j) map_.setCost(i, j, GridMap::LETHAL);
  auto path = planner_.plan(map_, {1, 1}, {8, 8});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->front(), (Cell{1, 1}));                    // empieza en el bote
  EXPECT_EQ(path->back(), (Cell{8, 8}));
  EXPECT_NE(map_.getCost((*path)[1].i, (*path)[1].j), GridMap::LETHAL);  // 2o punto ya es libre
}

TEST_F(AStarTest, StartLethalWithNoFreeCellFails) {
  map_.fill(GridMap::LETHAL);
  EXPECT_FALSE(planner_.plan(map_, {1, 1}, {8, 8}).has_value());
}
