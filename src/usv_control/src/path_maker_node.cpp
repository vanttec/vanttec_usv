#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <stack>
#include <cmath>
#include <string>
#include <optional>
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <limits>
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "usv_interfaces/msg/waypoint_list.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

struct Wp
{
    double x, y, theta;
};

class PathMakerNode : public rclcpp::Node
{
public:
    PathMakerNode() : Node("path_maker_node")
    {
        for (int i = 0; i < 100; i++){
            for (int j = 0; j < wp_vec.size(); j++){
                x = ...
                y = ...
                z = ...
                register_wp(x, y, theta);
            }
        }

        path_to_follow_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/path_to_follow", 10);

        current_path_ref_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/current_path_ref", 10);

        goals_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/goals_markers", 10);

        tmp_marker.header.frame_id = "world";
        tmp_marker.id = 0;
        tmp_marker.type = visualization_msgs::msg::Marker::ARROW;
        tmp_marker.action = 0;
        tmp_marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(1.).y(0.1).z(0.1);
        tmp_marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(1).a(1);
        tmp_marker.pose.position.z = 1.;

        pose_stamped_tmp_.header.frame_id = "world";
        pose_stamped_tmp_.header.stamp = PathMakerNode::now();
        current_path_ref.header = pose_stamped_tmp_.header;
        path_to_follow.header   = pose_stamped_tmp_.header;
        current_path_ref.poses.push_back(pose_stamped_tmp_);
        current_path_ref.poses.push_back(pose_stamped_tmp_);

        timer_ = this->create_wall_timer(10ms, std::bind(&PathMakerNode::update, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    rclcpp::Subscription<usv_interfaces::msg::WaypointList>::SharedPtr goals_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goals_markers_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_to_follow_pub_, current_path_ref_pub_;

    geometry_msgs::msg::PoseStamped pose_stamped_tmp_;

    visualization_msgs::msg::MarkerArray goals_markers;
    visualization_msgs::msg::Marker tmp_marker;

    nav_msgs::msg::Path current_path_ref, path_to_follow;

    std::vector<Wp> wp_vec;
    int wp_i{0};

    double pose[3]{0., 0., 0.};
    int closest_wp = 0;

    Wp next_wp{pose[0], pose[1]}, base_wp{pose[0], pose[1]};
    bool suitable_wp{false};

    // Function to compute the Euclidean distance between two points (x1, y1) and (x2, y2)
    double computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // Function to find the indices of the nearest waypoint behind and ahead within lookahead distance
    std::pair<int, int> findLookaheadWaypoints(double lookahead_distance) {

    double min_distance = std::numeric_limits<double>::max();

    // Iterate over waypoints to find the closest wp
    int j = closest_wp;
    bool in_lookregion = true;
    double previous_dist = std::numeric_limits<double>::max();
    while(j < path_to_follow.poses.size() && in_lookregion){
        double distance_to_wp = computeDistance(pose[0], pose[1], path_to_follow.poses[j].pose.position.x, path_to_follow.poses[j].pose.position.y);
        if (distance_to_wp < min_distance) {
            closest_wp = j;
            min_distance = distance_to_wp;
        }
        if(distance_to_wp > 1.5*lookahead_distance) {
            in_lookregion = false;
        }
        j++;
    }

    // Iterate over waypoints to find the furthest ahead
    int closest_wp_ahead = closest_wp;
    double max_distance_ahead = 0.;
    int i = closest_wp;
    bool in_lookahead = true;
    while(i < path_to_follow.poses.size() && in_lookahead){
        double distance_to_wp = computeDistance(
            path_to_follow.poses[closest_wp].pose.position.x, path_to_follow.poses[closest_wp].pose.position.y, 
            path_to_follow.poses[i].pose.position.x, path_to_follow.poses[i].pose.position.y);
        if (distance_to_wp <= lookahead_distance && distance_to_wp > max_distance_ahead) {
        closest_wp_ahead = i;
        max_distance_ahead = distance_to_wp;
        } else if(distance_to_wp > lookahead_distance){
            in_lookahead = false;
        }
        i++;
    }

    // Iterate over waypoints to find the furthest behind
    int closest_wp_behind = closest_wp;
    // double max_distance_behind = 0.;
    // i = closest_wp;
    // bool in_lookbehind = true;
    // while(i > 0 && in_lookbehind){
    //     double distance_to_wp = computeDistance(
    //         path_to_follow.poses[closest_wp].pose.position.x, path_to_follow.poses[closest_wp].pose.position.y, 
    //         path_to_follow.poses[i].pose.position.x, path_to_follow.poses[i].pose.position.y);
    //     if (distance_to_wp <= lookahead_distance && distance_to_wp > max_distance_behind) {
    //     closest_wp_behind = i;
    //     max_distance_behind = distance_to_wp;
    //     } else if(distance_to_wp > lookahead_distance){
    //         in_lookbehind = false;
    //     }
    //     i--;
    // }

    if(closest_wp_behind < 0){
        closest_wp_behind = 1;
    }
    if(closest_wp_ahead <= closest_wp_behind){
        if(closest_wp_ahead < path_to_follow.poses.size() - 1){
            closest_wp_ahead = closest_wp_behind+1;
        } else {
            closest_wp_behind = closest_wp_ahead - 1;
        }
    }

    return {closest_wp_behind, closest_wp_ahead};
    }

    void register_wp(double reg_x, double reg_y, double reg_theta){
        wp_vec.push_back(Wp{reg_x, reg_y, reg_theta});
        tmp_marker.pose.position.x = reg_x;
        tmp_marker.pose.position.y = reg_y;
        tf2::Quaternion q;
        q.setRPY(0, 0, reg_theta);
        tmp_marker.pose.orientation.x = q.x();
        tmp_marker.pose.orientation.y = q.y();
        tmp_marker.pose.orientation.z = q.z();
        tmp_marker.pose.orientation.w = q.w();
        tmp_marker.id = wp_vec.size();
        goals_markers.markers.push_back(tmp_marker);
    }

    
    // Helper function to compute a control point based on theta and a control distance
    Wp computeControlPoint(const Wp& wp, double control_distance) {
    Wp control_point;
    control_point.x = wp.x + control_distance * cos(wp.theta);
    control_point.y = wp.y + control_distance * sin(wp.theta);
    control_point.theta = wp.theta;
    return control_point;
    }

// Bezier interpolation function with quaternion SLERP for angles
void interpolateWaypoints(const Wp &start_wp, const Wp &end_wp, int num_interpolations, double control_distance) {
    // Compute the control points for the Bezier curve
    Wp control_point_start = computeControlPoint(start_wp, control_distance);
    Wp control_point_end = computeControlPoint(end_wp, -control_distance);  // Control point for end wp is in the reverse direction
    
    for (int i = 0; i <= num_interpolations; ++i) {
        double t = static_cast<double>(i) / num_interpolations;
        
        // Quadratic Bezier curve calculation for x and y
        double x_interp = (1 - t) * (1 - t) * (1 - t) * start_wp.x +
                        3 * (1 - t) * (1 - t) * t * control_point_start.x +
                        3 * (1 - t) * t * t * control_point_end.x +
                        t * t * t * end_wp.x;
        
        double y_interp = (1 - t) * (1 - t) * (1 - t) * start_wp.y +
                        3 * (1 - t) * (1 - t) * t * control_point_start.y +
                        3 * (1 - t) * t * t * control_point_end.y +
                        t * t * t * end_wp.y;    
    
        // Interpolating theta using quaternion SLERP to handle wraparound
        tf2::Quaternion start_q, end_q, interp_q;
        start_q.setRPY(0, 0, start_wp.theta);
        end_q.setRPY(0, 0, end_wp.theta);

        // Perform spherical linear interpolation (SLERP) between quaternions
        interp_q = start_q.slerp(end_q, t);

        // Normalize quaternion to avoid issues
        interp_q.normalize();

        // Create pose for the interpolated point
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = x_interp;
        pose_stamped.pose.position.y = y_interp;

        // Set the interpolated quaternion orientation
        pose_stamped.pose.orientation.x = interp_q.x();
        pose_stamped.pose.orientation.y = interp_q.y();
        pose_stamped.pose.orientation.z = interp_q.z();
        pose_stamped.pose.orientation.w = interp_q.w();

        // Add the interpolated pose to the path
        path_to_follow.poses.push_back(pose_stamped);
    }
}

    void update()
    {
        if(path_to_follow.poses.size() > 0){
            // Determine lookahead distance
            // double lookahead_distance = 1.5;  // meters (path-tracking)
            // double lookahead_distance = 2.;  // meters (path-tracking + avoidance)
            double lookahead_distance = 1.8;  // meters (path-tracking + dynamic avoidance)

            // Find waypoints behind and ahead of the boat
            auto [wp_behind_i, wp_ahead_i] = findLookaheadWaypoints(lookahead_distance);

            // If valid waypoints were found, create a guidance line for control
            if (wp_behind_i != -1 && wp_ahead_i != -1) {
                current_path_ref.poses[0].pose.position.x = path_to_follow.poses[wp_behind_i].pose.position.x;
                current_path_ref.poses[0].pose.position.y = path_to_follow.poses[wp_behind_i].pose.position.y;
                // current_path_ref.poses[0].pose.position.x = pose[0];
                // current_path_ref.poses[0].pose.position.y = pose[1];
                current_path_ref.poses[0].pose.position.z = 0.;
                current_path_ref.poses[1].pose.position.x = path_to_follow.poses[wp_ahead_i].pose.position.x;
                current_path_ref.poses[1].pose.position.y = path_to_follow.poses[wp_ahead_i].pose.position.y;
                current_path_ref.poses[1].pose.position.z = 1.;
            }
            // RCLCPP_INFO(this->get_logger(), "Indexes: %d, %d", wp_behind_i, wp_ahead_i);
        }

        // Clear the path and add the base waypoint
        path_to_follow.poses.clear();

        // Iterate through the waypoints and interpolate between each pair
        if (wp_vec.size() >= 2) {
            for (size_t i = 0; i < wp_vec.size() - 1; ++i) {
            const Wp &start_wp = wp_vec[i];
            const Wp &end_wp = wp_vec[i + 1];
            
            // Interpolate between the current pair of waypoints
            int num_interpolations = 100; // Number of intermediate points
            double control_distance = 0.9;  // Distance for Bezier control points
            interpolateWaypoints(start_wp, end_wp, num_interpolations, control_distance);
            }
        }

        path_to_follow_pub_->publish(path_to_follow);
        current_path_ref_pub_->publish(current_path_ref);
        goals_markers_pub_->publish(goals_markers);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathMakerNode>());
    rclcpp::shutdown();
    return 0;
}