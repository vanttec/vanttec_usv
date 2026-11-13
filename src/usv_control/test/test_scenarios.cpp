// Escenarios completos: el mismo pipeline que corre onTimer()
//   build(boyas) → stampGeofence → plan → shortcut
// con los parámetros por defecto de config/a_star_planner.yaml.
// Cada escenario tiene su gemelo en config/sites/tests/ para correrlo en Gazebo.
// Si cambias parámetros y uno de estos falla, revisa si el cambio de comportamiento es el que querías.
#include <gtest/gtest.h>
#include <cmath>
#include <optional>
#include "usv_control/planning/costmap_builder.hpp"
#include "usv_control/planning/a_star_planner.hpp"
#include "usv_control/planning/path_post_processor.hpp"
#include "usv_control/planning/obstacle_tracker.hpp"

namespace {

// Valores del YAML general: boat_width 0.6, safety_margin 0.3, resolution 0.2
constexpr double kRes = 0.2;
constexpr double kLethal = 0.6 / 2.0 + 0.3 + kRes / 2.0;   // 0.7 m, igual que el nodo

struct Result {
  GridMap map{-50.0, -50.0, kRes, 500, 500};
  std::optional<std::vector<Cell>> raw;
  std::vector<Point2D> raw_m;        // camino crudo en metros
  std::vector<Point2D> simple_m;     // camino simplificado en metros
};

Result runScenario(const std::vector<Obstacle>& buoys, const std::vector<Point2D>& geofence,
                   Point2D start, Point2D goal) {
  Result r;
  CostmapBuilder builder({kLethal, 1.5, 2.0});
  builder.build(r.map, buoys);
  builder.stampGeofence(r.map, geofence);

  int si, sj, gi, gj;
  r.map.worldToCell(start.x, start.y, si, sj);
  r.map.worldToCell(goal.x, goal.y, gi, gj);
  r.raw = AStarPlanner(1.0).plan(r.map, {si, sj}, {gi, gj});
  if (!r.raw) return r;

  auto toMeters = [&](const std::vector<Cell>& cells) {
    std::vector<Point2D> out;
    for (const auto& c : cells) {
      double x, y;
      r.map.cellToWorld(c.i, c.j, x, y);
      out.push_back({x, y});
    }
    return out;
  };
  r.raw_m = toMeters(*r.raw);
  r.simple_m = toMeters(PathPostProcessor(100).shortcut(r.map, *r.raw));
  return r;
}

// y del camino crudo cuando cruza x (la primera celda con x >= x_cross)
double yAtX(const std::vector<Point2D>& path, double x_cross) {
  for (const auto& p : path) if (p.x >= x_cross) return p.y;
  return NAN;
}

// Distancia mínima del camino crudo al borde de cualquier boya
double minClearance(const std::vector<Point2D>& path, const std::vector<Obstacle>& buoys) {
  double best = 1e9;
  for (const auto& p : path)
    for (const auto& b : buoys)
      best = std::min(best, std::hypot(p.x - b.x, p.y - b.y) - b.radius);
  return best;
}

std::vector<Obstacle> gate(double x, double gap_centers, double r = 0.15) {
  return {{x, -gap_centers / 2.0, r}, {x, gap_centers / 2.0, r}};
}

}  // namespace

// S1 — sin boyas: línea recta, solo inicio y goal.
TEST(Scenarios, S1_FreeWaterIsStraight) {
  auto r = runScenario({}, {}, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_EQ(r.simple_m.size(), 2u);
}

// S2 — una boya en medio: la rodea sin entrar al radio letal.
TEST(Scenarios, S2_SingleBuoyIsAvoided) {
  std::vector<Obstacle> buoys{{10, 0, 0.3}};
  auto r = runScenario(buoys, {}, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_GT(minClearance(r.raw_m, buoys), kLethal);
  EXPECT_EQ(r.simple_m.size(), 3u);   // inicio, un vértice para rodear, goal
}

// S3 — puerta ancha (3 m entre centros): la cruza derecho.
TEST(Scenarios, S3_WideGateGoesStraightThrough) {
  auto buoys = gate(10, 3.0);
  auto r = runScenario(buoys, {}, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_LT(std::abs(yAtX(r.raw_m, 10)), 1.5);
  EXPECT_EQ(r.simple_m.size(), 2u);
}

// S4 — puerta de 2 m (1.7 m libres): la cruza, pero el shortcut deja muchos
// vértices pegados porque el centro de la puerta cuesta más que cost_threshold.
TEST(Scenarios, S4_NarrowGateCrossesWithManyVertices) {
  auto buoys = gate(10, 2.0);
  auto r = runScenario(buoys, {}, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_LT(std::abs(yAtX(r.raw_m, 10)), 1.0);
  EXPECT_GT(minClearance(r.raw_m, buoys), kLethal);
  EXPECT_GT(r.simple_m.size(), 5u);   // documenta el comportamiento actual
}

// S5 — canal estrecho sin puerta: recto por el centro.
TEST(Scenarios, S5_ChannelWithoutGateIsStraight) {
  std::vector<Point2D> channel{{-5, -2}, {30, -2}, {30, 2}, {-5, 2}};
  auto r = runScenario({}, channel, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_EQ(r.simple_m.size(), 2u);
}

// S5b — canal + puerta de 1.2 m: el inflado la cierra y no hay por dónde rodear → STOP.
TEST(Scenarios, S5b_ClosedGateInChannelStops) {
  std::vector<Point2D> channel{{-5, -2}, {30, -2}, {30, 2}, {-5, 2}};
  auto r = runScenario(gate(10, 1.2), channel, {0, 0}, {20, 0});
  EXPECT_FALSE(r.raw.has_value());
}

// S6 — slalom: tres boyas alternadas; pasa entre ellas sin tocar lo letal.
TEST(Scenarios, S6_SlalomWeaves) {
  std::vector<Obstacle> buoys{{8, 1, 0.15}, {16, -1, 0.15}, {24, 1, 0.15}};
  auto r = runScenario(buoys, {}, {0, 0}, {32, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_GT(minClearance(r.raw_m, buoys), kLethal);
  EXPECT_GE(r.simple_m.size(), 4u);
}

// S7 — pared de boyas con un hueco en y ≈ 5: tiene que ir por el hueco.
TEST(Scenarios, S7_WallWithGapUsesTheGap) {
  std::vector<Obstacle> wall;
  for (double y = -10.0; y <= 10.01; y += 0.8)
    if (y < 4.0 || y > 6.0) wall.push_back({10, y, 0.15});
  auto r = runScenario(wall, {}, {0, 0}, {20, 0});
  ASSERT_TRUE(r.raw.has_value());
  double y = yAtX(r.raw_m, 10);
  EXPECT_GT(y, 4.0);
  EXPECT_LT(y, 6.0);
}

// S8 — goal encima de una boya: STOP (el goal sí debe ser válido).
TEST(Scenarios, S8_GoalOnBuoyStops) {
  auto r = runScenario({{10, 0, 0.3}}, {}, {0, 0}, {10, 0});
  EXPECT_FALSE(r.raw.has_value());
}

// S9 — el bote arranca pegado a una boya (dentro de lo letal): escapa y llega.
TEST(Scenarios, S9_StartNextToBuoyEscapes) {
  auto r = runScenario({{0.5, 0, 0.15}}, {}, {0, 0}, {10, 0});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_EQ(r.map.getCost(r.raw->front().i, r.raw->front().j), GridMap::LETHAL);
  EXPECT_NE(r.map.getCost((*r.raw)[1].i, (*r.raw)[1].j), GridMap::LETHAL);
}

// S10 — goal fuera de la geocerca: STOP.
TEST(Scenarios, S10_GoalOutsideGeofenceStops) {
  std::vector<Point2D> square{{-15, -15}, {15, -15}, {15, 15}, {-15, 15}};
  auto r = runScenario({}, square, {0, 0}, {20, 0});
  EXPECT_FALSE(r.raw.has_value());
}

// S11 — callejón en U abierto hacia atrás: tiene que salir por detrás (x < 0) para llegar.
TEST(Scenarios, S11_DeadEndBacksOut) {
  std::vector<Obstacle> u;
  for (double y = -4.0; y <= 4.01; y += 0.8) u.push_back({8, y, 0.15});
  for (double x = 0.0; x < 8.0; x += 0.8) {
    u.push_back({x, 4, 0.15});
    u.push_back({x, -4, 0.15});
  }
  auto r = runScenario(u, {}, {2, 0}, {16, 0});
  ASSERT_TRUE(r.raw.has_value());
  double min_x = 1e9;
  for (const auto& p : r.raw_m) min_x = std::min(min_x, p.x);
  EXPECT_LT(min_x, 0.0);
}

// S12 — capas: geocerca estática + boya dinámica estampada encima no se borran entre sí.
TEST(Scenarios, S12_DynamicLayerKeepsGeofence) {
  CostmapBuilder builder({kLethal, 1.5, 2.0});
  GridMap static_map(-50.0, -50.0, kRes, 500, 500);
  builder.build(static_map, {});
  builder.stampGeofence(static_map, {{-5, -2}, {30, -2}, {30, 2}, {-5, 2}});

  GridMap map = static_map;                       // lo que hace rebuildDynamicLayer()
  builder.stamp(map, gate(10, 1.2));
  int si, sj, gi, gj;
  map.worldToCell(0, 0, si, sj);
  map.worldToCell(20, 0, gi, gj);
  EXPECT_FALSE(AStarPlanner(1.0).plan(map, {si, sj}, {gi, gj}).has_value());
  EXPECT_TRUE(AStarPlanner(1.0).plan(static_map, {si, sj}, {gi, gj}).has_value());
}

// S13 — percepción relativa: bote en (0,0) con proa al Este ve una boya 10 m a proa.
// Debe quedar en NED (0, 10) y el camino hacia (0, 20) la rodea.
TEST(Scenarios, S13_BodyFrameBuoyLandsInRightPlace) {
  Point2D p = bodyToNed(0, 0, M_PI / 2, 10.0, 0.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 10.0, 1e-9);

  ObstacleTracker tracker(1.0, 0.0);
  tracker.update({{p.x, p.y, 0.3, "b1"}}, 0.0);
  auto r = runScenario(tracker.obstacles(), {}, {0, 0}, {0, 20});
  ASSERT_TRUE(r.raw.has_value());
  EXPECT_GT(minClearance(r.raw_m, tracker.obstacles()), kLethal);
}
