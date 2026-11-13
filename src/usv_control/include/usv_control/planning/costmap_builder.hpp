#pragma once
#include <vector>
#include "usv_control/planning/grid_map.hpp"

struct Obstacle {
  double x, y;     // centro en NED [m]
  double radius;   // radio de la boya [m]
};

struct Point2D {
  double x, y;     // NED [m]
};

struct InflationParams {
  double lethal_radius;     // r_inscrito + margen + res/2 [m]
  double influence_radius;  // hasta dónde llega el gradiente [m]
  double decay;             // k: qué tan rápido se enfría el costo
};

class CostmapBuilder {
public:
  explicit CostmapBuilder(const InflationParams& params);
  void build(GridMap& map, const std::vector<Obstacle>& obstacles) const;   // limpia + estampa
  void stamp(GridMap& map, const std::vector<Obstacle>& obstacles) const;   // estampa SIN limpiar
  // Fuera del polígono = LETHAL; dentro, la pared infla igual que una boya.
  // Llamar DESPUÉS de build(), porque build() limpia el mapa.
  void stampGeofence(GridMap& map, const std::vector<Point2D>& polygon) const;

private:
  static bool insidePolygon(const std::vector<Point2D>& poly, double x, double y);
  static double distanceToEdges(const std::vector<Point2D>& poly, double x, double y);
  void stampObstacle(GridMap& map, const Obstacle& obstacle) const;
  uint8_t costAtDistance(double d) const;
  InflationParams params_;
};