#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class LOSNode : public rclcpp::Node {
 public:
  LOSNode() : Node("los_node") {
    using namespace std::placeholders;

    path_sub = this->create_subscription<nav_msgs::msg::Path>(
        "/usv/path_to_follow", 10,
        [this](const nav_msgs::msg::Path &msg) { 
            if(path.poses.size() == 0)
                path = msg;
            else if(msg.poses[msg.poses.size() - 1].pose != path.poses[path.poses.size() - 1].pose) {
                path = msg;
                wp_i = 0;
            }
            });

    pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/usv/state/pose", 10,
        [this](const geometry_msgs::msg::Pose2D &msg) { this->pose = msg; });

    vel_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_velocity", 10);

    heading_vel_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_angular_velocity", 10);

    current_path_ref_pub = this->create_publisher<nav_msgs::msg::Path>(
        "/usv/current_path_ref", 10);

    pose_path_pub = this->create_publisher<nav_msgs::msg::Path>(
        "/usv/pose_path", 10);

    pose_stamped_tmp_.header.frame_id = "world";
    current_ref.header.frame_id = "world";
    current_ref.header.stamp = LOSNode::now();
    current_ref.poses.push_back(pose_stamped_tmp_);
    current_ref.poses.push_back(pose_stamped_tmp_);

    pose_accum.header.frame_id = "world";
    pose_accum.header.stamp = LOSNode::now();

    updateTimer =
        this->create_wall_timer(100ms, std::bind(&LOSNode::timer_callback, this));
  }

 private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub, heading_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_path_ref_pub, pose_path_pub;

    nav_msgs::msg::Path path, current_ref, pose_accum;
    geometry_msgs::msg::Pose2D pose;
    std_msgs::msg::Float64 vel_msg, heading_vel_msg;

    geometry_msgs::msg::PoseStamped pose_stamped_tmp_;

    double psi_d{0.0};

    const double lookahead_dist{2.0};

    int wp_i{0};

    rclcpp::TimerBase::SharedPtr updateTimer;

    void timer_callback() {
        // Look for the waypoint indexes
        double dist, angle_diff;
        
        if(wp_i < path.poses.size() - 1){
            // Find farthest point from current posision, limited by lookahead.
            std::size_t max_idx_ = wp_i;
            double max_distance_found_ = 0;
            for(int i = wp_i; i < path.poses.size(); i++){
                
                dist = distance(this->pose, path.poses[i].pose.position);
                angle_diff = get_angle_diff(this->pose, path.poses[i].pose.position);

                if(dist < lookahead_dist && dist > max_distance_found_ && 
                    ((i - max_idx_) < 10) && std::fabs(angle_diff) < 1.3){
                    max_idx_ = i;
                    max_distance_found_ = dist;
                }
            }

            wp_i = max_idx_;
        }

        if(path.poses.size() != 0){
            // Update the desired psi
            psi_d = std::atan2((path.poses[wp_i].pose.position.y - this->pose.y), 
                            (path.poses[wp_i].pose.position.x - this->pose.x));

            // Set and publish ros msgs
            vel_msg.data = std::clamp(distance(this->pose, path.poses[wp_i].pose.position) - 1 , 0.0, 0.5);
            vel_pub->publish(vel_msg);

            heading_vel_msg.data = -get_angle_diff(this->pose.theta, psi_d);
            
            if((wp_i + 1) == path.poses.size()) {
                // vel_msg.data = 0.0;
                heading_vel_msg.data = 0.0;
            }

            // RCLCPP_INFO(get_logger(), "wp_i: %d, size: %d, vel: %f", wp_i, path.poses.size(), heading_vel_msg.data);
            
            // Setting x: %f y: %f as new zero", zero_x, zero_y);
            heading_vel_pub->publish(heading_vel_msg);

            current_ref.poses[0].pose.position.x = this->pose.x;
            current_ref.poses[0].pose.position.y = this->pose.y;
            current_ref.poses[1] = path.poses[wp_i];
            current_path_ref_pub->publish(current_ref);
        }

        pose_stamped_tmp_.pose.position.x = pose.x;
        pose_stamped_tmp_.pose.position.y = pose.y;
        pose_accum.poses.push_back(pose_stamped_tmp_);
        pose_path_pub->publish(pose_accum);
    }

    double distance(geometry_msgs::msg::Pose2D pos, geometry_msgs::msg::Point wp){
        return sqrt(pow(pos.x - wp.x, 2) + pow(pos.y - wp.y, 2));
    }

    // needed direction for pose to point towards p
    float needed_angle(geometry_msgs::msg::Pose2D pos, geometry_msgs::msg::Point wp){
        return std::atan2(wp.y - pos.y, wp.x - pos.x);
    }

    float get_angle_diff(geometry_msgs::msg::Pose2D pos, geometry_msgs::msg::Point wp){
        double angle_diff = std::fmod((pos.theta - needed_angle(pos, wp) + M_PI), 2*M_PI) - M_PI;
        return angle_diff < -M_PI ? angle_diff + 2 * M_PI : angle_diff;        
    }

    float get_angle_diff(double psi_1, double psi_2){
        double angle_diff = std::fmod((psi_1 - psi_2 + M_PI), 2*M_PI) - M_PI;
        return angle_diff < -M_PI ? angle_diff + 2 * M_PI : angle_diff;        
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LOSNode>());
  rclcpp::shutdown();
  return 0;
}
