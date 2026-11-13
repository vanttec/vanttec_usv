#pragma once
#include <optional>
#include <vector>
#include "usv_control/planning/grid_map.hpp"


class AStarPlanner {
public:
  explicit AStarPlanner(double cost_weight);
  std::optional<std::vector<Cell>> plan(const GridMap& map, const Cell& start, const Cell& goal) const;

private:
  double heuristic(const Cell& a, const Cell& b, double res) const;
  double stepCost(const GridMap& map, const Cell& to, bool diagonal) const;
  int toId(const Cell& c, int size_y) const;
  Cell fromId(int id, int size_y) const;
  std::vector<Cell> reconstruct(const std::vector<int>& parent, int goal_id, int size_y) const;
  double cost_weight_;
};