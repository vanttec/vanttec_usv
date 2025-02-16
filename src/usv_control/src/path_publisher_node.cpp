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
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
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

using namespace std::chrono_literals;

class PathPublisher_node : public rclcpp::Node
{
public:
    PathPublisher_node() : Node("path_publisher_node")
    {
        // Declare and get parameters
        this->declare_parameter("wave.pattern", "sine");
        this->declare_parameter("wave.amplitude", 5.0);
        this->declare_parameter("wave.frequency", 0.3);
        this->declare_parameter("wave.length", 100.0);
        this->declare_parameter("wave.delta", 0.1);

        wave_pattern_ = this->get_parameter("wave.pattern").as_string();
        amplitude_ = this->get_parameter("wave.amplitude").as_double();
        frequency_ = this->get_parameter("wave.frequency").as_double();
        path_length_ = this->get_parameter("wave.length").as_double();
        delta_ = this->get_parameter("wave.delta").as_double();

        // Rest of your subscriptions
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/state/pose", 10,
            [this](const geometry_msgs::msg::Pose2D &msg)
            {
                pose[0] = msg.x;
                pose[1] = msg.y;
                pose[2] = msg.theta;
            });

        mission_id_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/usv/mission/id", 10,
            [this](const std_msgs::msg::Int8 &msg)
            {
                switch (msg.data)
                {
                case 3:
                    lookahead_distance = 1.;
                    break;
                case 4:
                    lookahead_distance = 2.;
                    break;
                default:
                    lookahead_distance = 2.;
                    break;
                }
            });

        path_to_follow_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/path_to_follow", 10);

        current_path_ref_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/current_path_ref", 10);

        pose_stamped_tmp_.header.frame_id = "world";
        pose_stamped_tmp_.header.stamp = PathPublisher_node::now();
        current_path_ref.header = pose_stamped_tmp_.header;
        path_to_follow.header = pose_stamped_tmp_.header;
        current_path_ref.poses.push_back(pose_stamped_tmp_);
        current_path_ref.poses.push_back(pose_stamped_tmp_);

        timer_ = this->create_wall_timer(20ms, std::bind(&PathPublisher_node::update, this));
    }

private:
    // Config parameters
    std::string wave_pattern_;
    double amplitude_;
    double frequency_;
    double path_length_;
    double delta_;

    // Existing member variables
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_id_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_to_follow_pub_, current_path_ref_pub_;
    geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
    nav_msgs::msg::Path current_path_ref, path_to_follow;
    double pose[3]{0., 0., 0.};
    int closest_wp = 0;
    double lookahead_distance = 2.;

    // Helper functions remain the same
    double computeDistance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    std::pair<double, double> getWavePoint(double x)
    {
        double y = 0.0;
        double theta = 0.0;

        if (wave_pattern_ == "sine")
        {
            y = amplitude_ * sin(frequency_ * x);
            double dy = amplitude_ * frequency_ * cos(frequency_ * x);
            theta = atan2(dy, 1.0);
        }
        else if (wave_pattern_ == "straight")
        {
            y = 0.0;
            theta = 0.0;
        }
        else if (wave_pattern_ == "zigzag")
        {
            // Create a zigzag pattern
            double period = 2 * M_PI / frequency_;
            double phase = fmod(x, period) / period;
            if (phase < 0.5)
            {
                y = 2 * amplitude_ * phase;
            }
            else
            {
                y = 2 * amplitude_ * (1 - phase);
            }
            theta = atan2(amplitude_ * 2 / period * (phase < 0.5 ? 1 : -1), 1.0);
        }

        return {y, theta};
    }

    std::pair<int, int> findLookaheadWaypoints(double lookahead_distance)
    {
        // Your existing findLookaheadWaypoints implementation
        double min_distance = std::numeric_limits<double>::max();

        int j = closest_wp;
        bool in_lookregion = true;
        while (j < path_to_follow.poses.size() && in_lookregion)
        {
            double distance_to_wp = computeDistance(pose[0], pose[1], 
                path_to_follow.poses[j].pose.position.x, 
                path_to_follow.poses[j].pose.position.y);
            if (distance_to_wp < min_distance)
            {
                closest_wp = j;
                min_distance = distance_to_wp;
            }
            if (distance_to_wp > 1.5 * lookahead_distance)
            {
                in_lookregion = false;
            }
            j++;
        }

        int closest_wp_ahead = closest_wp;
        double max_distance_ahead = 0.;
        int i = closest_wp;
        bool in_lookahead = true;
        while (i < path_to_follow.poses.size() && in_lookahead)
        {
            double distance_to_wp = computeDistance(
                path_to_follow.poses[closest_wp].pose.position.x,
                path_to_follow.poses[closest_wp].pose.position.y,
                path_to_follow.poses[i].pose.position.x,
                path_to_follow.poses[i].pose.position.y);
            if (distance_to_wp <= lookahead_distance && distance_to_wp > max_distance_ahead)
            {
                closest_wp_ahead = i;
                max_distance_ahead = distance_to_wp;
            }
            else if (distance_to_wp > lookahead_distance)
            {
                in_lookahead = false;
            }
            i++;
        }

        int closest_wp_behind = closest_wp;
        if (closest_wp_behind < 0)
        {
            closest_wp_behind = 1;
        }
        if (closest_wp_ahead <= closest_wp_behind)
        {
            if (closest_wp_ahead < path_to_follow.poses.size() - 1)
            {
                closest_wp_ahead = closest_wp_behind + 1;
            }
            else
            {
                closest_wp_behind = closest_wp_ahead - 1;
            }
        }

        return {closest_wp_behind, closest_wp_ahead};
    }

    void update()
    {
        path_to_follow.poses.clear();

        int num_waypoints = static_cast<int>(path_length_ / delta_);

        for (int i = 0; i < num_waypoints; ++i)
        {
            double x = i * delta_;
            auto [y, theta] = getWavePoint(x);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_to_follow.poses.push_back(pose_stamped);
        }

        if (path_to_follow.poses.size() > 0)
        {
            auto [wp_behind_i, wp_ahead_i] = findLookaheadWaypoints(lookahead_distance);

            if (wp_behind_i != -1 && wp_ahead_i != -1)
            {
                current_path_ref.poses[0].pose.position.x = path_to_follow.poses[wp_behind_i].pose.position.x;
                current_path_ref.poses[0].pose.position.y = path_to_follow.poses[wp_behind_i].pose.position.y;
                current_path_ref.poses[0].pose.position.z = 0.;

                current_path_ref.poses[1].pose.position.x = path_to_follow.poses[wp_ahead_i].pose.position.x;
                current_path_ref.poses[1].pose.position.y = path_to_follow.poses[wp_ahead_i].pose.position.y;
                current_path_ref.poses[1].pose.position.z = 0.;
            }
            current_path_ref_pub_->publish(current_path_ref);
        }

        path_to_follow_pub_->publish(path_to_follow);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher_node>());
    rclcpp::shutdown();
    return 0;
}