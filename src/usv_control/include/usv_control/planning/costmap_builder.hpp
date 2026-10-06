#pragma once
#include <vector>
#include "usv_control/planning/grid_map.hpp"

struct Obstacle {
  double x, y;     // centro en NED [m]
  double radius;   // radio de la boya [m]
};

struct InflationParams {
  double lethal_radius;     // r_inscrito + margen + res/2 [m]
  double influence_radius;  // hasta dónde llega el gradiente [m]
  double decay;             // k: qué tan rápido se enfría el costo
};

class CostmapBuilder {
public:
  explicit CostmapBuilder(const InflationParams& params);
  void build(GridMap& map, const std::vector<Obstacle>& obstacles) const;

private:
  void stampObstacle(GridMap& map, const Obstacle& obstacle) const;
  uint8_t costAtDistance(double d) const;
  InflationParams params_;
};