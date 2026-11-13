#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "usv_interfaces/msg/usv_loc.hpp"

#include "usv_control/planning/grid_map.hpp"
#include "usv_control/planning/costmap_builder.hpp"
#include "usv_control/planning/a_star_planner.hpp"
#include "usv_control/planning/path_post_processor.hpp"

using std::placeholders::_1;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode()
  : Node("a_star_planner_node"),
    loc_timeout_(declare_parameter<double>("loc_timeout", 1.0)),
    frame_id_(declare_parameter<std::string>("frame_id", "ned")),
    map_(makeGridMap()),
    costmap_builder_(makeInflationParams()),
    planner_(declare_parameter<double>("cost_weight", 1.0)),
    post_processor_(static_cast<uint8_t>(declare_parameter<int>("cost_threshold", 100)))
  {
    loc_sub_ = create_subscription<usv_interfaces::msg::UsvLoc>(
      "usv/loc", 10, std::bind(&PlannerNode::onLoc, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "usv/planning/goal", 10, std::bind(&PlannerNode::onGoal, this, _1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("usv/planning/path", 10);

    const double rate = declare_parameter<double>("publish_rate", 2.0);
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

  InflationParams makeInflationParams() {
    return InflationParams{
      declare_parameter<double>("lethal_radius", 0.6),
      declare_parameter<double>("influence_radius", 1.5),
      declare_parameter<double>("decay", 2.0)};
  }

  // ---------- Callbacks ----------
  void onLoc(const usv_interfaces::msg::UsvLoc::SharedPtr msg) {
    // TODO: guardar x, y, position_valid; last_loc_time_ = now(); has_loc_ = true
    boat_x_ = msg->x;
    boat_y_ = msg->y;
    position_valid_ = msg->position_valid;
    last_loc_time_ = this->now();
    has_loc_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // TODO: guardar msg->pose.position.x / .y; has_goal_ = true
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    has_goal_ = true;
  }

  void onTimer() {
    // 1. sin goal
    // 2. sin pose, o (now() - last_loc_time_).seconds() > loc_timeout_
    // 3. !position_valid_
    // 4. bote o goal fuera del mapa (worldToCell)
    // 5. costmap_builder_.build(map_, {});
    // 6. planner_.plan(...) vacío
    // 7. post_processor_.shortcut(...)
    // 8. path_pub_->publish(toPathMsg(...))
    if(!has_goal_){
        publishStop("sin goal");
        return;
    }
    if(!has_loc_ || (this->now() - last_loc_time_).seconds() > loc_timeout_){
        publishStop("sin pose o timout");
        return;
    }
    // if(!position_valid_){
    //     publishStop("position no válida");
    //     return;
    // }
    int boat_i, boat_j, goal_i, goal_j;
    if(!map_.worldToCell(boat_x_,boat_y_,boat_i,boat_j) || !map_.worldToCell(goal_x_,goal_y_,goal_i,goal_j)){
        publishStop("bote o goal fuera del mapa");
        return;
    }
    costmap_builder_.build(map_,{});
    auto cells = planner_.plan(map_,Cell{boat_i,boat_j},Cell{goal_i,goal_j});
    if(!cells){
        publishStop("no se encontró camino");
        return;
    }
    auto simplified = post_processor_.shortcut(map_, *cells);
    path_pub_->publish(toPathMsg(simplified, boat_x_, boat_y_, goal_x_, goal_y_));
  }

  // ---------- Salida ----------
  void publishStop(const std::string& reason) {
    // TODO: Path con header lleno y cero poses; publicar
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "STOP: %s", reason.c_str());
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    path_pub_->publish(msg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "STOP: %s", reason.c_str());
  }

  nav_msgs::msg::Path toPathMsg(const std::vector<Cell>& cells,double boat_x, double boat_y,double goal_x, double goal_y) {
    // TODO: header (stamp = now(), frame_id_); una PoseStamped por celda (cellToWorld),
    // orientation.w = 1; primer punto = bote exacto, último = goal exacto
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
  // Parámetros
  double loc_timeout_;
  std::string frame_id_;

  // Planeación
  GridMap map_;
  CostmapBuilder costmap_builder_;
  AStarPlanner planner_;
  PathPostProcessor post_processor_;

  // Estado
  double boat_x_ = 0.0, boat_y_ = 0.0;
  bool position_valid_ = false;
  bool has_loc_ = false;
  rclcpp::Time last_loc_time_;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  bool has_goal_ = false;

  // ROS
  rclcpp::Subscription<usv_interfaces::msg::UsvLoc>::SharedPtr loc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}