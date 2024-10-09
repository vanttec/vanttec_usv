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

class WaypointHandlerNode : public rclcpp::Node
{
public:
    WaypointHandlerNode() : Node("waypoint_handler_node")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/state/pose", 10,
            [this](const geometry_msgs::msg::Pose2D &msg)
            {
                pose[0] = msg.x;
                pose[1] = msg.y;
                pose[2] = msg.theta;
            });

        goals_sub_ = this->create_subscription<usv_interfaces::msg::WaypointList>(
            "/usv/goals", 10,
            [this](const usv_interfaces::msg::WaypointList &msg)
            {
                if(wp_vec.size() == 0){
                    register_wp(pose[0], pose[1], pose[2]);
                }
                // next_wp = Wp{msg.waypoint_list[0].x, msg.waypoint_list[0].y};
                if (msg.waypoint_list.size() == 0)
                {
                    return;
                }
                for (int i = 0; i < msg.waypoint_list.size(); i++)
                {
                    bool wp_repeats{false};
                    for (int j = 0; j < wp_vec.size(); j++)
                    {
                        if (sqrt(pow(wp_vec[j].x - msg.waypoint_list[i].x, 2) + pow(wp_vec[j].y - msg.waypoint_list[i].y, 2)) < 0.2)
                        {
                            wp_repeats = true;
                        }
                    }
                    if (!wp_repeats)
                    {
                        register_wp(msg.waypoint_list[i].x, msg.waypoint_list[i].y, msg.waypoint_list[i].theta);
                    }
                }
            });

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
        current_path_ref.header.frame_id = "world";
        current_path_ref.header.stamp = WaypointHandlerNode::now();
        path_to_follow.header = current_path_ref.header;
        current_path_ref.poses.push_back(pose_stamped_tmp_);
        current_path_ref.poses.push_back(pose_stamped_tmp_);

        timer_ = this->create_wall_timer(10ms, std::bind(&WaypointHandlerNode::update, this));
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

    Wp next_wp{pose[0], pose[1]}, base_wp{pose[0], pose[1]};
    bool suitable_wp{false};

    // Function to compute the Euclidean distance between two points (x1, y1) and (x2, y2)
    double computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // Function to find the indices of the nearest waypoint behind and ahead within lookahead distance
    std::pair<int, int> findLookaheadWaypoints(double lookahead_distance) {
    int closest_wp_behind = -1;
    int closest_wp_ahead = -1;
    int closest_wp = -1;
    
    double max_distance_behind = 0.;
    double max_distance_ahead = 0.;
    double min_distance = std::numeric_limits<double>::max();;

    // Iterate over waypoints to find the closest distance

    for(int j = 0; j < path_to_follow.poses.size() ; j++) {
        double distance_to_wp = computeDistance(pose[0], pose[1], path_to_follow.poses[j].pose.position.x, path_to_follow.poses[j].pose.position.y);
        // If waypoint is behind the boat (negative theta difference)
        if (distance_to_wp < min_distance) {
        closest_wp = j;
        min_distance = distance_to_wp;
        }
    }
    // Iterate over waypoints to find the closest behind and ahead
    for(int i = 0; i < path_to_follow.poses.size(); i++) {
    // if(i != closest_wp){
        double distance_to_wp = computeDistance(
            path_to_follow.poses[closest_wp].pose.position.x, path_to_follow.poses[closest_wp].pose.position.y, 
            path_to_follow.poses[i].pose.position.x, path_to_follow.poses[i].pose.position.y);

        // Determine if waypoint is behind the boat
        double heading_to_wp = std::atan2(
            path_to_follow.poses[i].pose.position.y - path_to_follow.poses[closest_wp].pose.position.y,
            path_to_follow.poses[i].pose.position.x - path_to_follow.poses[closest_wp].pose.position.x);
        tf2::Quaternion quat;
        tf2::fromMsg(path_to_follow.poses[closest_wp].pose.orientation, quat);
        double roll, pitch, psi_;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, psi_);
        psi_ = std::fmod((psi_) + M_PI, 2*M_PI) - M_PI; // [-pi, pi]"

        double theta_diff = heading_to_wp - psi_;
        if (theta_diff > M_PI) theta_diff -= 2 * M_PI;
        if (theta_diff < -M_PI) theta_diff += 2 * M_PI;

        // If waypoint is behind the boat (negative theta difference)
        if (fabs(theta_diff) >= M_PI_2 && distance_to_wp > max_distance_behind && distance_to_wp <= lookahead_distance) {
        closest_wp_behind = i;
        max_distance_behind = distance_to_wp;
        }

        // If waypoint is ahead of the boat and within lookahead distance
        if (fabs(theta_diff) < M_PI_2 && distance_to_wp <= lookahead_distance && distance_to_wp > max_distance_ahead) {
        closest_wp_ahead = i;
        max_distance_ahead = distance_to_wp;
        }
    // }
    }

    if(closest_wp_behind < 0){
        closest_wp_behind = 1;
    }
    if(closest_wp_ahead <= closest_wp_behind && closest_wp_ahead < path_to_follow.poses.size()){
        closest_wp_ahead = closest_wp_behind+1;
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

    // Bezier interpolation function
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
    
        // Interpolating theta, ensuring shortest angular difference
        double theta_diff = end_wp.theta - start_wp.theta;
        if (theta_diff > M_PI) theta_diff -= 2 * M_PI;
        if (theta_diff < -M_PI) theta_diff += 2 * M_PI;
        double theta_interp = start_wp.theta + t * theta_diff;

        // Create pose for the interpolated point
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = x_interp;
        pose_stamped.pose.position.y = y_interp;

        // Set the orientation from interpolated theta using a quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_interp);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        // Add the interpolated pose to the path
        path_to_follow.poses.push_back(pose_stamped);
    }
    }

    void update()
    {
        if(path_to_follow.poses.size() > 0){
            // Determine lookahead distance
            // double lookahead_distance = 1.5;  // meters (path-tracking)
            double lookahead_distance = 2.;  // meters (path-tracking + avoidance)

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
            RCLCPP_INFO(this->get_logger(), "Indexes: %d, %d", wp_behind_i, wp_ahead_i);
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
            double control_distance = 1.5;  // Distance for Bezier control points
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
    rclcpp::spin(std::make_shared<WaypointHandlerNode>());
    rclcpp::shutdown();
    return 0;
}