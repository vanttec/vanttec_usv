#pragma once
#include <cstdint>
#include <vector>
#include "usv_control/planning/grid_map.hpp"

class PathPostProcessor {
public:
  explicit PathPostProcessor(uint8_t cost_threshold);
  std::vector<Cell> shortcut(const GridMap& map, const std::vector<Cell>& path) const;

private:
  bool lineOfSight(const GridMap& map, const Cell& a, const Cell& b) const;
  uint8_t cost_threshold_;
};