#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

class LosSteeringNode : public rclcpp::Node
{
public:
    LosSteeringNode() : Node("los_steering_node")
    {
        // ROS 2 Parameters
        this->declare_parameter<double>("lookahead_distance", 2.0);
        this->declare_parameter<double>("cruise_velocity", 0.6); // Updated default to 0.6

        // Subscriptions
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "usv/planning/path", 10,
            std::bind(&LosSteeringNode::path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "usv/state/pose", 10,
            std::bind(&LosSteeringNode::pose_callback, this, std::placeholders::_1));

        // Publishers
        heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("/setpoint/heading", 10);
        vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/setpoint/velocity", 10);
        
        RCLCPP_INFO(this->get_logger(), "LOS Steering Node initialized with Station Keeping.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_points_.clear();
        for (const auto& pose_stamped : msg->poses)
        {
            path_points_.emplace_back(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y
            );
        }
        
        if (!path_points_.empty()) {
            // Save the absolute final waypoint for station keeping
            final_waypoint_ = path_points_.back();
            station_keeping_active_ = false; 
            RCLCPP_INFO(this->get_logger(), "New path received (%zu points). Replacing current path and exiting station keeping.", path_points_.size());
        }
    }

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        Eigen::Vector2d p_boat(msg->x, msg->y);
        double cruise_vel = this->get_parameter("cruise_velocity").as_double();

        // No path has ever been received
        if (path_points_.empty() && !station_keeping_active_)
        {
            publish_setpoints(0.0, 0.0);
            return;
        }

        // If we previously finished the path, maintain station keeping
        if (station_keeping_active_)
        {
            handle_station_keeping(p_boat, cruise_vel);
            return;
        }

        // 1. Eliminate passed waypoints based on orthogonal projection
        while (path_points_.size() >= 2)
        {
            Eigen::Vector2d w0 = path_points_[0];
            Eigen::Vector2d w1 = path_points_[1];
            Eigen::Vector2d v_seg = w1 - w0;
            
            double seg_len_sq = v_seg.squaredNorm();
            if (seg_len_sq == 0.0)
            {
                path_points_.erase(path_points_.begin());
                continue;
            }

            Eigen::Vector2d v_boat = p_boat - w0;
            double t = v_boat.dot(v_seg) / seg_len_sq;

            if (t >= 1.0)
            {
                // The boat's projection is beyond w1
                path_points_.erase(path_points_.begin());
            }
            else
            {
                break; 
            }
        }

        // If path completion is detected
        if (path_points_.size() < 2)
        {
            RCLCPP_INFO(this->get_logger(), "Path completed. Entering station keeping mode.");
            station_keeping_active_ = true;
            handle_station_keeping(p_boat, cruise_vel);
            return;
        }

        // 2. Find the closest point on the active segment (Cross-track point)
        Eigen::Vector2d w0 = path_points_[0];
        Eigen::Vector2d w1 = path_points_[1];
        Eigen::Vector2d v_seg = w1 - w0;
        double seg_len = v_seg.norm();
        Eigen::Vector2d dir = v_seg / seg_len;

        Eigen::Vector2d v_boat = p_boat - w0;
        double t = v_boat.dot(v_seg) / (seg_len * seg_len);
        
        t = std::clamp(t, 0.0, 1.0);
        Eigen::Vector2d p_closest = w0 + t * v_seg;

        // 3. Calculate Lookahead Point along the remaining path
        double lookahead_dist = this->get_parameter("lookahead_distance").as_double();
        double remaining_lookahead = lookahead_dist;
        Eigen::Vector2d p_lookahead = p_closest;
        
        double dist_to_w1 = (w1 - p_closest).norm();

        if (dist_to_w1 >= remaining_lookahead)
        {
            p_lookahead = p_closest + remaining_lookahead * dir;
        }
        else
        {
            remaining_lookahead -= dist_to_w1;
            p_lookahead = w1; 
            
            for (size_t i = 1; i < path_points_.size() - 1; ++i)
            {
                Eigen::Vector2d wp_curr = path_points_[i];
                Eigen::Vector2d wp_next = path_points_[i+1];
                Eigen::Vector2d next_seg = wp_next - wp_curr;
                
                double next_seg_len = next_seg.norm();
                
                if (next_seg_len >= remaining_lookahead)
                {
                    p_lookahead = wp_curr + (next_seg / next_seg_len) * remaining_lookahead;
                    remaining_lookahead = 0.0;
                    break;
                }
                else
                {
                    remaining_lookahead -= next_seg_len;
                    p_lookahead = wp_next;
                }
            }
        }

        // 4. Calculate desired heading and publish for active path following
        double desired_heading = std::atan2(p_lookahead.y() - p_boat.y(), p_lookahead.x() - p_boat.x());
        publish_setpoints(desired_heading, cruise_vel);
    }

    void handle_station_keeping(const Eigen::Vector2d& p_boat, double cruise_vel)
    {
        double distance_to_target = (final_waypoint_ - p_boat).norm();
        double desired_heading = std::atan2(final_waypoint_.y() - p_boat.y(), final_waypoint_.x() - p_boat.x());

        // Drift tolerance of 2 meters
        if (distance_to_target > 2.0)
        {
            // Point towards the target and move forward to correct the drift
            publish_setpoints(desired_heading, cruise_vel);
        }
        else
        {
            // Within 1 meter radius, set velocity to 0 but keep heading oriented toward the coordinate
            publish_setpoints(desired_heading, 0.0);
        }
    }

    void publish_setpoints(double heading, double velocity)
    {
        std_msgs::msg::Float64 heading_msg;
        heading_msg.data = heading;
        heading_pub_->publish(heading_msg);

        std_msgs::msg::Float64 vel_msg;
        vel_msg.data = velocity;
        vel_pub_->publish(vel_msg);
    }

    // Node state
    std::vector<Eigen::Vector2d> path_points_;
    Eigen::Vector2d final_waypoint_;
    bool station_keeping_active_ = false;
    
    // Pubs/Subs
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LosSteeringNode>());
    rclcpp::shutdown();
    return 0;
}