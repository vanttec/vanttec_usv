#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class LidarMissionFinderNode : public rclcpp::Node
{
public:
LidarMissionFinderNode() : Node("lidar_mission_finder_node")
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/usv/state/odom", 1,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            auto & q = msg->pose.pose.orientation;
            pose.x() = msg->pose.pose.position.x;
            pose.y() = msg->pose.pose.position.y;
            pose.z() = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                1.0 - 2.0*(q.y*q.y + q.z*q.z));

            rot_m << std::cos(pose(2)), - std::sin(pose(2)), 0, 
                    std::sin(pose(2)), std::cos(pose(2)), 0, 
                    0, 0, 1;
        });
    
    lidar_clusters_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/clustered_marker", 10,
        [this](const visualization_msgs::msg::MarkerArray &msg) {
            if(msg.markers.size() == 0)
                return;

            int idx = 0;
            double closest_dist = std::numeric_limits<double>::max();
            for(int i = 0 ; i < msg.markers.size() ; i++){
                if(msg.markers[i].pose.position.x > 0.1){
                    double dist = std::fabs(
                        msg.markers[i].pose.position.x*msg.markers[i].pose.position.x +
                        msg.markers[i].pose.position.y*msg.markers[i].pose.position.y
                    );
                    if(dist < closest_dist){
                        closest_dist = dist;
                        idx = i;
                    }
                }
            }

            Eigen::Vector3d local_goal;
            local_goal.x() = msg.markers[idx].pose.position.x;
            local_goal.y() = msg.markers[idx].pose.position.y;
            local_goal.z() = std::atan2(local_goal.y(), local_goal.x());

            goal = rot_m*forward(local_goal, -2.5) + pose;

            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(0, 0, goal.z());
            tf2_quat.normalize();

            next_mission_msg.pose.position.x = goal.x();
            next_mission_msg.pose.position.y = goal.y();
            next_mission_msg.pose.orientation = tf2::toMsg(tf2_quat);
            next_mission_msg.header.stamp = this->get_clock()->now();
            next_mission_pub_->publish(next_mission_msg);
        });
    
    next_mission_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_mission_guess", 10);

    next_mission_msg.header.frame_id = "world";
}

Eigen::Vector3d forward(const Eigen::Vector3d &goal, double distance){
  Eigen::Vector3d p;
  p << std::cos(goal(2)), std::sin(goal(2)), 0.0;
  return goal + distance * p;
}

private:
rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_clusters_sub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_mission_pub_;

geometry_msgs::msg::PoseStamped next_mission_msg;

Eigen::Vector3d goal;
Eigen::Matrix3d rot_m;
Eigen::Vector3d pose;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarMissionFinderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}