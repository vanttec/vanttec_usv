#include <gtest/gtest.h>
#include "usv_control/planning/grid_map.hpp"

// Malla de 20 x 10 m a 0.2 m: x en [-10, 10), y en [-5, 5)
class GridMapTest : public ::testing::Test {
protected:
  GridMap map_{-10.0, -5.0, 0.2, 100, 50};
};

TEST_F(GridMapTest, ConstructorSetsSizeAndStartsFree) {
  EXPECT_EQ(map_.sizeX(), 100);
  EXPECT_EQ(map_.sizeY(), 50);
  EXPECT_DOUBLE_EQ(map_.resolution(), 0.2);
  EXPECT_EQ(map_.getCost(0, 0), GridMap::FREE);
  EXPECT_EQ(map_.getCost(99, 49), GridMap::FREE);
}

TEST_F(GridMapTest, WorldToCellInside) {
  int i, j;
  ASSERT_TRUE(map_.worldToCell(0.05, 0.05, i, j));
  EXPECT_EQ(i, 50);
  EXPECT_EQ(j, 25);
}

TEST_F(GridMapTest, WorldToCellNegativeCoordinates) {
  int i, j;
  ASSERT_TRUE(map_.worldToCell(-9.95, -4.95, i, j));
  EXPECT_EQ(i, 0);
  EXPECT_EQ(j, 0);
}

TEST_F(GridMapTest, WorldToCellOutside) {
  int i, j;
  EXPECT_FALSE(map_.worldToCell(-10.1, 0.0, i, j));
  EXPECT_FALSE(map_.worldToCell(10.1, 0.0, i, j));
  EXPECT_FALSE(map_.worldToCell(0.0, -5.1, i, j));
  EXPECT_FALSE(map_.worldToCell(0.0, 5.1, i, j));
}

TEST_F(GridMapTest, CellToWorldGivesCenter) {
  double x, y;
  map_.cellToWorld(0, 0, x, y);
  EXPECT_NEAR(x, -9.9, 1e-9);
  EXPECT_NEAR(y, -4.9, 1e-9);
}

TEST_F(GridMapTest, RoundTripCellWorldCell) {
  for (auto [ci, cj] : {std::pair{0, 0}, {99, 49}, {37, 12}}) {
    double x, y;
    int i, j;
    map_.cellToWorld(ci, cj, x, y);
    ASSERT_TRUE(map_.worldToCell(x, y, i, j));
    EXPECT_EQ(i, ci);
    EXPECT_EQ(j, cj);
  }
}

TEST_F(GridMapTest, RoundTripWorldCellWorldWithinHalfRes) {
  int i, j;
  double x, y;
  ASSERT_TRUE(map_.worldToCell(1.23, -2.34, i, j));
  map_.cellToWorld(i, j, x, y);
  EXPECT_LE(std::abs(x - 1.23), 0.1 + 1e-9);
  EXPECT_LE(std::abs(y + 2.34), 0.1 + 1e-9);
}

TEST_F(GridMapTest, IsInside) {
  EXPECT_TRUE(map_.isInside(0, 0));
  EXPECT_TRUE(map_.isInside(99, 49));
  EXPECT_FALSE(map_.isInside(-1, 0));
  EXPECT_FALSE(map_.isInside(100, 0));
  EXPECT_FALSE(map_.isInside(0, 50));
}

TEST_F(GridMapTest, SetGetCostDoesNotTranspose) {
  map_.setCost(3, 4, GridMap::LETHAL);
  EXPECT_EQ(map_.getCost(3, 4), GridMap::LETHAL);
  EXPECT_EQ(map_.getCost(4, 3), GridMap::FREE);  // detecta i/j intercambiados
}

TEST_F(GridMapTest, OutsideIsLethalAndSetIsIgnored) {
  EXPECT_EQ(map_.getCost(-1, 0), GridMap::LETHAL);
  EXPECT_EQ(map_.getCost(100, 0), GridMap::LETHAL);
  map_.setCost(-1, 0, GridMap::FREE);  // no debe tronar
}

TEST_F(GridMapTest, FillSetsEveryCell) {
  map_.setCost(3, 4, GridMap::LETHAL);
  map_.fill(GridMap::FREE);
  EXPECT_EQ(map_.getCost(3, 4), GridMap::FREE);
  EXPECT_EQ(map_.getCost(99, 49), GridMap::FREE);
}