#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "usv_interfaces/msg/usv_loc.hpp"
#include "usv_interfaces/msg/object_list.hpp"

#include "usv_control/planning/grid_map.hpp"
#include "usv_control/planning/costmap_builder.hpp"
#include "usv_control/planning/a_star_planner.hpp"
#include "usv_control/planning/path_post_processor.hpp"
#include "usv_control/planning/obstacle_tracker.hpp"

using std::placeholders::_1;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode()
  : Node("a_star_planner_node"),
    loc_timeout_(declare_parameter<double>("loc_timeout", 1.0)),
    frame_id_(declare_parameter<std::string>("frame_id", "ned")),
    require_valid_position_(declare_parameter<bool>("require_valid_position", true)),
    map_(makeGridMap()),
    static_map_(map_),
    costmap_builder_(makeInflationParams()),
    planner_(declare_parameter<double>("cost_weight", 1.0)),
    post_processor_(readCostThreshold()),
    tracker_(makeTracker())
  {
    readPerceptionParams();

    loc_sub_ = create_subscription<usv_interfaces::msg::UsvLoc>(
      "usv/loc", 10, std::bind(&PlannerNode::onLoc, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "usv/planning/goal", 10, std::bind(&PlannerNode::onGoal, this, _1));
    obj_sub_ = create_subscription<usv_interfaces::msg::ObjectList>(
      "usv/perception/objects", 10, std::bind(&PlannerNode::onObjects, this, _1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("usv/planning/path", 10);

    buildStaticCostmap();

    const double rate = declare_parameter<double>("publish_rate", 2.0);
    if (rate <= 0.0) {
      throw std::invalid_argument("publish_rate debe ser > 0, llegó " + std::to_string(rate));
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               std::bind(&PlannerNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "a_star_planner_node listo: mapa %d x %d celdas",
                map_.sizeX(), map_.sizeY());
  }

private:
  // ---------- Construcción a partir de parámetros ----------
  GridMap makeGridMap() {
    const double x_min = declare_parameter<double>("x_min", -50.0);
    const double x_max = declare_parameter<double>("x_max", 50.0);
    const double y_min = declare_parameter<double>("y_min", -50.0);
    const double y_max = declare_parameter<double>("y_max", 50.0);
    const double res   = declare_parameter<double>("resolution", 0.2);
    const int size_x = static_cast<int>(std::ceil((x_max - x_min) / res));
    const int size_y = static_cast<int>(std::ceil((y_max - y_min) / res));
    return GridMap(x_min, y_min, res, size_x, size_y);
  }

  uint8_t readCostThreshold() {
    const int t = declare_parameter<int>("cost_threshold", 100);
    if (t < 0 || t >= GridMap::LETHAL) {
      throw std::invalid_argument("cost_threshold debe estar en [0, 253], llegó " + std::to_string(t));
    }
    return static_cast<uint8_t>(t);
  }

  // Usa map_.resolution(): map_ DEBE estar declarado antes que costmap_builder_.
  InflationParams makeInflationParams() {
    const double width     = declare_parameter<double>("boat_width", 0.6);
    const double length    = declare_parameter<double>("boat_length", 1.0);
    const double margin    = declare_parameter<double>("safety_margin", 0.3);
    const double influence = declare_parameter<double>("influence_radius", 1.5);
    const double decay     = declare_parameter<double>("decay", 2.0);

    if (width <= 0.0 || length <= 0.0 || margin < 0.0) {
      throw std::invalid_argument("medidas del barco deben ser > 0 y margen >= 0");
    }

    const double r_inscribed = width / 2.0;
    const double lethal = r_inscribed + margin + map_.resolution() / 2.0;

    if (influence <= lethal) {
      throw std::invalid_argument("influence_radius (" + std::to_string(influence) +
        ") debe ser > lethal_radius calculado (" + std::to_string(lethal) + ")");
    }

    RCLCPP_INFO(get_logger(), "lethal_radius calculado: %.3f m", lethal);
    return InflationParams{lethal, influence, decay};
  }

  ObstacleTracker makeTracker() {
    const double merge = declare_parameter<double>("merge_distance", 1.0);
    const double timeout = declare_parameter<double>("obstacle_timeout", 0.0);
    return ObstacleTracker(merge, timeout);
  }

  void readPerceptionParams() {
    buoy_radius_ = declare_parameter<double>("buoy_radius", 0.15);
    sensor_x_ = declare_parameter<double>("sensor_x", 0.0);
    sensor_y_ = declare_parameter<double>("sensor_y", 0.0);
    if (buoy_radius_ <= 0.0) {
      throw std::invalid_argument("buoy_radius debe ser > 0");
    }
  }

  // buoys: [x1, y1, r1, x2, y2, r2, ...] en NED [m]
  std::vector<Obstacle> readBuoys() {
    const auto flat = declare_parameter<std::vector<double>>("buoys", std::vector<double>{});
    if (flat.size() % 3 != 0) {
      throw std::invalid_argument("buoys debe tener múltiplo de 3 valores (x, y, r), llegaron " +
                                  std::to_string(flat.size()));
    }
    std::vector<Obstacle> buoys;
    for (size_t k = 0; k < flat.size(); k += 3) {
      if (flat[k + 2] <= 0.0) {
        throw std::invalid_argument("buoys: radio de la boya " + std::to_string(k / 3) + " debe ser > 0");
      }
      buoys.push_back(Obstacle{flat[k], flat[k + 1], flat[k + 2]});
    }
    return buoys;
  }

  // geofence: [x1, y1, x2, y2, ...] vértices en orden (horario o antihorario). Vacío = sin geocerca.
  std::vector<Point2D> readGeofence() {
    const auto flat = declare_parameter<std::vector<double>>("geofence", std::vector<double>{});
    if (flat.empty()) return {};
    if (flat.size() % 2 != 0 || flat.size() < 6) {
      throw std::invalid_argument("geofence necesita pares (x, y) y al menos 3 vértices, llegaron " +
                                  std::to_string(flat.size()) + " valores");
    }
    std::vector<Point2D> poly;
    for (size_t k = 0; k < flat.size(); k += 2) {
      poly.push_back(Point2D{flat[k], flat[k + 1]});
    }
    return poly;
  }

  // Capa estática (YAML del sitio): se construye UNA vez y se guarda en static_map_.
  // Cada vez que cambian los obstáculos de percepción: map_ = static_map_ + capa dinámica.
  void buildStaticCostmap() {
    const auto buoys = readBuoys();
    const auto geofence = readGeofence();
    costmap_builder_.build(map_, buoys);
    costmap_builder_.stampGeofence(map_, geofence);
    static_map_ = map_;
    RCLCPP_INFO(get_logger(), "costmap estático: %zu boyas, geocerca de %zu vértices",
                buoys.size(), geofence.size());
  }

  void rebuildDynamicLayer() {
    map_ = static_map_;
    costmap_builder_.stamp(map_, tracker_.obstacles());
    map_dirty_ = false;
  }

  // ---------- Callbacks ----------
  void onLoc(const usv_interfaces::msg::UsvLoc::SharedPtr msg) {
    boat_x_ = msg->x;
    boat_y_ = msg->y;
    boat_yaw_ = msg->yaw;
    position_valid_ = msg->position_valid;
    last_loc_time_ = this->now();
    has_loc_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    has_goal_ = true;
  }

  // header.frame_id == frame_id_ ("ned") -> x, y ya están en NED.
  // Cualquier otro frame -> relativos al bote (x proa, y estribor), se transforman con la pose actual.
  void onObjects(const usv_interfaces::msg::ObjectList::SharedPtr msg) {
    const bool in_world = (msg->header.frame_id == frame_id_);
    if (!in_world && !has_loc_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "objetos relativos al bote pero sin pose: descartados");
      return;
    }

    std::vector<Detection> detections;
    detections.reserve(msg->obj_list.size());
    for (const auto& obj : msg->obj_list) {
      double x = obj.x;
      double y = obj.y;
      if (!in_world) {
        // El sensor no está en el origen del bote: primero se pasa al origen del bote.
        Point2D p = bodyToNed(boat_x_, boat_y_, boat_yaw_, obj.x + sensor_x_, obj.y + sensor_y_);
        x = p.x;
        y = p.y;
      }
      const double r = obj.radius > 0.0 ? obj.radius : buoy_radius_;
      detections.push_back(Detection{x, y, r, obj.uuid});
    }
    tracker_.update(detections, this->now().seconds());
    map_dirty_ = true;
  }

  void onTimer() {
    // 1. sin goal
    // 2. sin pose, o (now() - last_loc_time_).seconds() > loc_timeout_
    // 3. !position_valid_ (si require_valid_position_)
    // 4. bote o goal fuera del mapa (worldToCell)
    // 5. actualizar capa dinámica si cambió (percepción o timeout)
    // 6. planner_.plan(...) vacío
    // 7. post_processor_.shortcut(...)
    // 8. path_pub_->publish(toPathMsg(...))
    if (!has_goal_) {
      publishStop("sin goal");
      return;
    }
    if (!has_loc_ || (this->now() - last_loc_time_).seconds() > loc_timeout_) {
      publishStop("sin pose o timeout");
      return;
    }
    if (require_valid_position_ && !position_valid_) {
      publishStop("posición no válida");
      return;
    }
    int boat_i, boat_j, goal_i, goal_j;
    if (!map_.worldToCell(boat_x_, boat_y_, boat_i, boat_j) ||
        !map_.worldToCell(goal_x_, goal_y_, goal_i, goal_j)) {
      publishStop("bote o goal fuera del mapa");
      return;
    }
    if (tracker_.prune(this->now().seconds())) map_dirty_ = true;
    if (map_dirty_) rebuildDynamicLayer();

    auto cells = planner_.plan(map_, Cell{boat_i, boat_j}, Cell{goal_i, goal_j});
    if (!cells) {
      publishStop("no se encontró camino");
      return;
    }
    auto simplified = post_processor_.shortcut(map_, *cells);
    path_pub_->publish(toPathMsg(simplified, boat_x_, boat_y_, goal_x_, goal_y_));
  }

  // ---------- Salida ----------
  void publishStop(const std::string& reason) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    path_pub_->publish(msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "STOP: %s", reason.c_str());
  }

  // Primer punto = bote exacto, intermedios = centros de celda, último = goal exacto.
  nav_msgs::msg::Path toPathMsg(const std::vector<Cell>& cells, double boat_x, double boat_y,
                                double goal_x, double goal_y) {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = frame_id_;

    auto addPose = [&](double x, double y) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      path.poses.push_back(p);
    };

    addPose(boat_x, boat_y);
    for (size_t k = 1; k + 1 < cells.size(); ++k) {
      double x, y;
      map_.cellToWorld(cells[k].i, cells[k].j, x, y);
      addPose(x, y);
    }
    addPose(goal_x, goal_y);

    return path;
  }

  // ---------- Miembros ----------
  // NO reordenar: C++ los inicializa en ESTE orden (no el de la lista del constructor).
  // map_ va antes que static_map_ y costmap_builder_ porque ambos lo leen.

  // Parámetros
  double loc_timeout_;
  std::string frame_id_;
  bool require_valid_position_;
  double buoy_radius_ = 0.15;
  double sensor_x_ = 0.0, sensor_y_ = 0.0;   // posición del sensor en el bote [m] (proa, estribor)

  // Planeación
  GridMap map_;          // el que usa A*: estático + dinámico
  GridMap static_map_;   // solo geocerca + boyas del YAML
  CostmapBuilder costmap_builder_;
  AStarPlanner planner_;
  PathPostProcessor post_processor_;
  ObstacleTracker tracker_;
  bool map_dirty_ = false;

  // Estado
  double boat_x_ = 0.0, boat_y_ = 0.0, boat_yaw_ = 0.0;
  bool position_valid_ = false;
  bool has_loc_ = false;
  rclcpp::Time last_loc_time_;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  bool has_goal_ = false;

  // ROS
  rclcpp::Subscription<usv_interfaces::msg::UsvLoc>::SharedPtr loc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr obj_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
