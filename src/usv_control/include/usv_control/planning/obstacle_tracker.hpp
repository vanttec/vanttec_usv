#pragma once
#include <string>
#include <vector>
#include "usv_control/planning/costmap_builder.hpp"

// Una detección ya convertida a NED local.
struct Detection {
  double x, y;       // NED [m]
  double radius;     // [m]
  std::string uuid;  // vacío = percepción no da id
};

// Convierte un punto relativo al bote (x hacia la proa, y hacia estribor)
// a NED local, usando la pose del bote (boat_x = Norte, boat_y = Este, yaw desde el Norte).
Point2D bodyToNed(double boat_x, double boat_y, double yaw, double x_body, double y_body);

// Memoria de obstáculos en NED.
// Percepción solo ve lo que tiene enfrente; si una boya sale del campo de visión,
// NO debe desaparecer del mapa (el planner trazaría encima de ella).
//
// Asociación de cada detección con un obstáculo ya conocido:
//   1. mismo uuid (si viene), si no
//   2. el más cercano a menos de merge_distance, si no
//   3. obstáculo nuevo.
class ObstacleTracker {
public:
  // timeout <= 0: nunca se olvida un obstáculo (boyas fijas).
  ObstacleTracker(double merge_distance, double timeout);

  void update(const std::vector<Detection>& detections, double now);
  bool prune(double now);                      // true si borró alguno
  std::vector<Obstacle> obstacles() const;
  size_t size() const;

private:
  struct Track {
    Obstacle obstacle;
    std::string uuid;
    double last_seen;
  };
  int findMatch(const Detection& d) const;     // índice en tracks_, o -1

  double merge_distance_;
  double timeout_;
  std::vector<Track> tracks_;
};
