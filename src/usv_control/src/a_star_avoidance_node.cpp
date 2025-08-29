#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "usv_interfaces/msg/waypoint_list.hpp"

#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/path.hpp"
#include "AStar.hpp"

using namespace std::chrono_literals;

struct Wp {
  double x, y;
};

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
    {
        generator.setWorldSize({int(map_size), int(map_size)});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/state/pose", 10,
            [this](const geometry_msgs::msg::Pose2D &msg) { pose = msg; });

        obstacle_sub_ = this->create_subscription<nav_msgs::msg::GridCells>(
            "/usv/obstacle_margins_map", 10,
            [this](const nav_msgs::msg::GridCells &msg)
            {
                generator.clearCollisions();
                for(int i = 0 ; i < msg.cells.size() ; i++){
                    generator.addCollision({int(msg.cells[i].x*multiplier+map_size/2), int(msg.cells[i].y*multiplier+map_size/2)});
                }
            });

        goals_sub_ = this->create_subscription<usv_interfaces::msg::WaypointList>(
            "/usv/goals", 10,
            [this](const usv_interfaces::msg::WaypointList &msg) {
                Wp wp1{pose.x, pose.y};
                Wp wp2;
                a_star_path.poses.clear();
                for(int i = 0 ; i < msg.waypoint_list.size() ; i++){
                    if(i>0){
                        wp1 = Wp{msg.waypoint_list[i-1].x, msg.waypoint_list[i-1].y};
                    }
                    wp2 = Wp{msg.waypoint_list[i].x, msg.waypoint_list[i].y};

                    auto path = generator.findPath({wp2.x*multiplier+map_size/2, 
                                                    wp2.y*multiplier+map_size/2},
                                                    {wp1.x*multiplier+map_size/2, 
                                                    wp1.y*multiplier+map_size/2});

                    for(auto& coordinate : path) {
                        // std::cout << coordinate.x << " " << coordinate.y << "\n";
                        pose_stamped_tmp_.pose.position.x = (coordinate.x-map_size/2)/multiplier;
                        pose_stamped_tmp_.pose.position.y = (coordinate.y-map_size/2)/multiplier;
                        a_star_path.poses.push_back(pose_stamped_tmp_);                    
                    }
                }
            });

        a_star_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/path_to_follow", 10);

        a_star_path.header.frame_id = "world";
        a_star_path.header.stamp = ObstacleAvoidanceNode::now();
        pose_stamped_tmp_.header.frame_id = "world";


        updateTimer =
            this->create_wall_timer(100ms, std::bind(&ObstacleAvoidanceNode::update, this));
    }


private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr a_star_path_pub_;

    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<usv_interfaces::msg::WaypointList>::SharedPtr goals_sub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
    geometry_msgs::msg::Pose2D pose;
    nav_msgs::msg::Path a_star_path;

    AStar::Generator generator;

    double map_size{300.};
    double multiplier = 2.;

    void update() {
        a_star_path_pub_->publish(a_star_path);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
