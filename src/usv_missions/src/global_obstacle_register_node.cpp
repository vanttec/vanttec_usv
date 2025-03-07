#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

struct MarkerProps {
  int type;
  double x, y, z, z_trans;
};

class GlobalObstacleRegisterNode : public rclcpp::Node {
public:
    GlobalObstacleRegisterNode() : Node("global_obstacle_register_node") {
        // Parameters
        this->declare_parameter("min_obstacle_separation", 0.5);  // Minimum distance between obstacles
        this->declare_parameter("max_tracking_distance", 10.0);   // Maximum distance to track obstacles
        this->declare_parameter("tracking_lifetime", 30.0);       // Seconds to keep tracking an obstacle without updates
        
        min_obstacle_separation_ = this->get_parameter("min_obstacle_separation").as_double();
        max_tracking_distance_ = this->get_parameter("max_tracking_distance").as_double();
        tracking_lifetime_ = this->get_parameter("tracking_lifetime").as_double();
        
        // Publishers
        object_list_global_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/obj_list_global", 10);
        object_list_local_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/obj_list", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);
        on_watch_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/on_watch", 10);

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/state/pose", 10, 
            std::bind(&GlobalObstacleRegisterNode::pose_callback, this, _1));
        
        inferences_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
            "/inferences", 10, 
            std::bind(&GlobalObstacleRegisterNode::inferences_callback, this, _1));
        
        // Timer for periodic publishing and cleanup
        timer_ = this->create_wall_timer(
            50ms, std::bind(&GlobalObstacleRegisterNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Dynamic Obstacle Tracker Node initialized");
    }

private:
    // ROS2 Components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr object_list_local_pub_, object_list_global_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_, on_watch_marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr inferences_sub_;
    
    // Configuration parameters
    double min_obstacle_separation_;
    double max_tracking_distance_;
    double tracking_lifetime_;
    
    // Vehicle state
    geometry_msgs::msg::Pose2D pose_;
    
    // Global storage
    usv_interfaces::msg::ObjectList global_obstacles_;
    usv_interfaces::msg::ObjectList local_obstacles_;
    visualization_msgs::msg::MarkerArray global_markers_;
    visualization_msgs::msg::MarkerArray watch_markers_;
    
    // Obstacle tracking - stores last update time for each obstacle
    std::vector<rclcpp::Time> obstacle_last_seen_;
    int next_obstacle_id_ = 0;
    
    // Color definitions
    int color_list[6][4]{
        {1,0,0,1},  // Red
        {0,1,0,1},  // Green
        {0,0,1,1},  // Blue
        {1,1,0,1},  // Yellow
        {0,0,0,1},  // Black
        {0,0,0,0}   // Transparent
    };
    
    // Marker type definitions
    std::map<std::string, MarkerProps> marker_type = {
        {"round", MarkerProps{2, 0.5, 0.5, 0.5, 0}},
        {"boat", MarkerProps{2, 1.0, 1.0, 1.0, 0}},
        {"marker", MarkerProps{3, 0.5, 0.5, 1, 0.25}},
        {"picture", MarkerProps{1, 0.5, 0.5, 0.5, 0.25}},
    };
    
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        pose_ = *msg;
    }
    
    void inferences_callback(const usv_interfaces::msg::ObjectList::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu inferences", msg->obj_list.size());
        
        for (const auto& inference : msg->obj_list) {
            process_inference(inference);
        }
    }
    
    void process_inference(const usv_interfaces::msg::Object& inference) {
        // Convert local coordinates to global
        auto global_pos = local_to_global(inference.x, inference.y);
        
        // Check if this inference is close to any existing obstacle
        bool is_new_obstacle = true;
        std::size_t closest_idx = 0;
        double closest_distance = std::numeric_limits<double>::max();
        
        for (std::size_t i = 0; i < global_obstacles_.obj_list.size(); ++i) {
            const auto& obs = global_obstacles_.obj_list[i];
            double distance = std::hypot(
                global_pos.first - obs.x,
                global_pos.second - obs.y
            );
            
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_idx = i;
            }
            
            if (distance < min_obstacle_separation_) {
                is_new_obstacle = false;
                break;
            }
        }
        
        if (is_new_obstacle) {
            // Add as new obstacle
            usv_interfaces::msg::Object new_obj;
            new_obj.x = global_pos.first;
            new_obj.y = global_pos.second;
            new_obj.v_x = 0.0;  // Initially, we don't know velocity
            new_obj.v_y = 0.0;
            new_obj.color = inference.color;
            new_obj.type = inference.type.empty() ? "marker" : inference.type;
            
            global_obstacles_.obj_list.push_back(new_obj);
            obstacle_last_seen_.push_back(this->now());
            
            // Create marker for visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.id = next_obstacle_id_++;
            marker.type = marker_type[new_obj.type].type;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = new_obj.x;
            marker.pose.position.y = new_obj.y;
            marker.pose.position.z = marker_type[new_obj.type].z_trans;
            
            marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                .x(marker_type[new_obj.type].x)
                .y(marker_type[new_obj.type].y)
                .z(marker_type[new_obj.type].z);
            
            int color_idx = new_obj.color >= 0 && new_obj.color < 6 ? new_obj.color : 0;
            marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>()
                .r(color_list[color_idx][0])
                .g(color_list[color_idx][1])
                .b(color_list[color_idx][2])
                .a(color_list[color_idx][3]);
            
            global_markers_.markers.push_back(marker);
            
            RCLCPP_INFO(this->get_logger(), "Added new obstacle at (%f, %f)", new_obj.x, new_obj.y);
        } else {
            // Update existing obstacle
            auto& obs = global_obstacles_.obj_list[closest_idx];
            auto& marker = global_markers_.markers[closest_idx];
            
            // Update velocity estimate
            double dt = (this->now() - obstacle_last_seen_[closest_idx]).seconds();
            if (dt > 0.0) {
                obs.v_x = (global_pos.first - obs.x) / dt;
                obs.v_y = (global_pos.second - obs.y) / dt;
            }
            
            // Update position
            obs.x = global_pos.first;
            obs.y = global_pos.second;
            
            // Update marker
            marker.pose.position.x = obs.x;
            marker.pose.position.y = obs.y;
            
            // Update the time
            obstacle_last_seen_[closest_idx] = this->now();
            
            RCLCPP_INFO(this->get_logger(), "Updated obstacle at (%f, %f), velocity: (%f, %f)", 
                obs.x, obs.y, obs.v_x, obs.v_y);
        }
    }
    
    void timer_callback() {
        // Clean up expired obstacles
        // cleanup_old_obstacles();
        
        // Update local obstacles list from global list
        update_local_obstacles();
        
        // Publish everything
        object_list_global_pub_->publish(global_obstacles_);
        object_list_local_pub_->publish(local_obstacles_);
        marker_pub_->publish(global_markers_);
        on_watch_marker_pub_->publish(watch_markers_);
    }
    
    void cleanup_old_obstacles() {
        auto current_time = this->now();
        std::vector<std::size_t> to_remove;
        
        // Find obstacles that haven't been seen for a while
        for (std::size_t i = 0; i < obstacle_last_seen_.size(); ++i) {
            double time_since_last_seen = (current_time - obstacle_last_seen_[i]).seconds();
            if (time_since_last_seen > tracking_lifetime_) {
                to_remove.push_back(i);
            }
        }
        
        // Remove them in reverse order to maintain valid indices
        for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
            std::size_t idx = *it;
            
            // Remove from all containers
            global_obstacles_.obj_list.erase(global_obstacles_.obj_list.begin() + idx);
            obstacle_last_seen_.erase(obstacle_last_seen_.begin() + idx);
            
            // Mark the marker for deletion
            if (idx < global_markers_.markers.size()) {
                global_markers_.markers[idx].action = visualization_msgs::msg::Marker::DELETE;
            }
        }
        
        // Clean up deleted markers periodically
        bool cleaned_markers = false;
        for (auto it = global_markers_.markers.begin(); it != global_markers_.markers.end();) {
            if (it->action == visualization_msgs::msg::Marker::DELETE) {
                it = global_markers_.markers.erase(it);
                cleaned_markers = true;
            } else {
                ++it;
            }
        }
        
        if (cleaned_markers) {
            // Update marker IDs to ensure they stay unique
            for (std::size_t i = 0; i < global_markers_.markers.size(); ++i) {
                global_markers_.markers[i].id = i;
            }
        }
    }
    
    void update_local_obstacles() {
        // Clear current local obstacles
        local_obstacles_.obj_list.clear();
        watch_markers_.markers.clear();
        
        for (std::size_t i = 0; i < global_obstacles_.obj_list.size(); ++i) {
            const auto& global_obj = global_obstacles_.obj_list[i];
            
            // Convert to vehicle-relative coordinates
            auto local_pos = global_to_local(global_obj.x, global_obj.y);
            double local_x = local_pos.first;
            double local_y = local_pos.second;
            
            // Calculate distance to obstacle
            double distance = std::hypot(local_x, local_y);
            
            // Check if in tracking range and field of view
            bool in_range = distance < max_tracking_distance_;
            bool in_fov = local_x > 0 && std::fabs(std::atan2(local_y, local_x)) < 1.0; // ~60 degree FOV
            
            // Create watch marker
            visualization_msgs::msg::Marker watch_marker;
            watch_marker.header.frame_id = "world";
            watch_marker.header.stamp = this->now();
            watch_marker.id = i;
            watch_marker.type = marker_type[global_obj.type].type;
            
            watch_marker.pose.position.x = global_obj.x;
            watch_marker.pose.position.y = global_obj.y;
            watch_marker.pose.position.z = marker_type[global_obj.type].z_trans;
            
            watch_marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                .x(marker_type[global_obj.type].x * 2.0)
                .y(marker_type[global_obj.type].y * 2.0)
                .z(marker_type[global_obj.type].z * 2.0);
            
            if (in_range && in_fov) {
                // Add to local list if in range and field of view
                usv_interfaces::msg::Object local_obj;
                local_obj.x = local_x;
                local_obj.y = local_y;
                local_obj.v_x = global_obj.v_x; // Keep same velocity for now
                local_obj.v_y = global_obj.v_y;
                local_obj.color = global_obj.color;
                local_obj.type = global_obj.type;
                
                local_obstacles_.obj_list.push_back(local_obj);
                
                // Highlight in watch markers
                int color_idx = global_obj.color >= 0 && global_obj.color < 6 ? global_obj.color : 0;
                watch_marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>()
                    .r(color_list[color_idx][0])
                    .g(color_list[color_idx][1])
                    .b(color_list[color_idx][2])
                    .a(0.5);
                watch_marker.action = visualization_msgs::msg::Marker::ADD;
            } else {
                // Not in view, mark for deletion in watch markers
                watch_marker.action = visualization_msgs::msg::Marker::DELETE;
            }
            
            watch_markers_.markers.push_back(watch_marker);
        }
    }
    
    // Coordinate transformation utilities
    std::pair<double, double> local_to_global(double local_x, double local_y) {
        double global_x = pose_.x + local_x * std::cos(pose_.theta) - local_y * std::sin(pose_.theta);
        double global_y = pose_.y + local_x * std::sin(pose_.theta) + local_y * std::cos(pose_.theta);
        return {global_x, global_y};
    }
    
    std::pair<double, double> global_to_local(double global_x, double global_y) {
        double dx = global_x - pose_.x;
        double dy = global_y - pose_.y;
        double local_x = dx * std::cos(-pose_.theta) - dy * std::sin(-pose_.theta);
        double local_y = dx * std::sin(-pose_.theta) + dy * std::cos(-pose_.theta);
        return {local_x, local_y};
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalObstacleRegisterNode>());
    rclcpp::shutdown();
    return 0;
}