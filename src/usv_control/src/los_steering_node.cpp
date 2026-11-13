#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class LosSteeringNode : public rclcpp::Node
{
public:
    LosSteeringNode() : Node("los_steering_node")
    {
        this->declare_parameter<double>("lookahead_distance", 2.0);
        this->declare_parameter<double>("cruise_velocity", 0.6);

        // Subscriptions
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "usv/planning/path", 10,
            std::bind(&LosSteeringNode::path_callback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "usv/state/pose", 10,
            std::bind(&LosSteeringNode::pose_callback, this, std::placeholders::_1));

        // Control Publishers
        heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("/setpoint/heading", 10);
        vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/setpoint/velocity", 10);
        
        // Visualization Publishers
        crosstrack_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visualization/crosstrack", 10);
        los_vector_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visualization/los_vector", 10);

        RCLCPP_INFO(this->get_logger(), "LOS Steering Node initialized with RViz Visualizations.");
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
            final_waypoint_ = path_points_.back();
            station_keeping_active_ = false; 
            RCLCPP_INFO(this->get_logger(), "New path received (%zu points).", path_points_.size());
        }
    }

    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        Eigen::Vector2d p_boat(msg->x, msg->y);
        double cruise_vel = this->get_parameter("cruise_velocity").as_double();

        if (path_points_.empty() && !station_keeping_active_)
        {
            publish_setpoints(0.0, 0.0);
            return;
        }

        if (station_keeping_active_)
        {
            handle_station_keeping(p_boat, cruise_vel);
            return;
        }

        if (path_points_.size() < 2)
        {
            enter_station_keeping(p_boat, cruise_vel);
            return;
        }

        // Iterate through the points list to find the point with the smallest distance (norm)
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_points_.size(); ++i)
        {
            double dist = (path_points_[i] - p_boat).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Find the pair of points (the active segment)
        size_t active_idx = 0;
        if (closest_idx == 0)
        {
            active_idx = 0;
        }
        else if (closest_idx == path_points_.size() - 1)
        {
            active_idx = closest_idx - 1;
        }
        else
        {
            Eigen::Vector2d p_prev = path_points_[closest_idx - 1];
            Eigen::Vector2d p_curr = path_points_[closest_idx];
            Eigen::Vector2d p_next = path_points_[closest_idx + 1];

            auto dist_to_seg = [&](const Eigen::Vector2d& w0, const Eigen::Vector2d& w1) {
                Eigen::Vector2d v_seg = w1 - w0;
                double seg_len_sq = v_seg.squaredNorm();
                if (seg_len_sq == 0.0) return (p_boat - w0).norm();
                
                double t = std::clamp((p_boat - w0).dot(v_seg) / seg_len_sq, 0.0, 1.0);
                Eigen::Vector2d p_proj = w0 + t * v_seg;
                return (p_boat - p_proj).norm();
            };

            double dist_prev = dist_to_seg(p_prev, p_curr);
            double dist_next = dist_to_seg(p_curr, p_next);

            if (dist_prev < dist_next) {
                active_idx = closest_idx - 1;
            } else {
                active_idx = closest_idx;
            }
        }

        // Draw a line between the two points and find the closest point on the line (cross-track point)
        Eigen::Vector2d w0 = path_points_[active_idx];
        Eigen::Vector2d w1 = path_points_[active_idx + 1];
        Eigen::Vector2d v_seg = w1 - w0;
        double seg_len = v_seg.norm();
        
        if (seg_len == 0.0) 
        {
            path_points_.erase(path_points_.begin() + active_idx);
            return; 
        }

        Eigen::Vector2d dir = v_seg / seg_len;
        double t_unclamped = (p_boat - w0).dot(v_seg) / (seg_len * seg_len);
        
        if (active_idx == path_points_.size() - 2 && t_unclamped >= 1.0)
        {
            path_points_.clear();
            enter_station_keeping(p_boat, cruise_vel);
            return;
        }

        double t_clamped = std::clamp(t_unclamped, 0.0, 1.0);
        Eigen::Vector2d p_closest = w0 + t_clamped * v_seg;

        // Eliminate all previous points from the list
        if (active_idx > 0)
        {
            path_points_.erase(path_points_.begin(), path_points_.begin() + active_idx);
        }

        // Calculate Lookahead Point along the remaining path
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

        // Calculate desired heading, publish controls, and visualize
        double desired_heading = std::atan2(p_lookahead.y() - p_boat.y(), p_lookahead.x() - p_boat.x());
        publish_setpoints(desired_heading, cruise_vel);

        // Publish visualization paths
        crosstrack_pub_->publish(create_viz_line(p_boat, p_closest));
        los_vector_pub_->publish(create_viz_line(p_boat, p_lookahead));
    }

    void enter_station_keeping(const Eigen::Vector2d& p_boat, double cruise_vel)
    {
        RCLCPP_INFO(this->get_logger(), "Path completed. Entering station keeping mode.");
        station_keeping_active_ = true;
        handle_station_keeping(p_boat, cruise_vel);
    }

    void handle_station_keeping(const Eigen::Vector2d& p_boat, double cruise_vel)
    {
        double distance_to_target = (final_waypoint_ - p_boat).norm();
        double desired_heading = std::atan2(final_waypoint_.y() - p_boat.y(), final_waypoint_.x() - p_boat.x());

        if (distance_to_target > 1.0)
        {
            publish_setpoints(desired_heading, cruise_vel);
        }
        else
        {
            publish_setpoints(desired_heading, 0.0);
        }

        // During station keeping, just point both visualizations to the target coordinate
        auto viz_line = create_viz_line(p_boat, final_waypoint_);
        crosstrack_pub_->publish(viz_line);
        los_vector_pub_->publish(viz_line);
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

    // Helper to create a 2-point path for RViz line visualization
    nav_msgs::msg::Path create_viz_line(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "ned"; 
        path_msg.header.stamp = this->get_clock()->now();

        geometry_msgs::msg::PoseStamped pose1;
        pose1.pose.position.x = p1.x();
        pose1.pose.position.y = p1.y();
        pose1.pose.position.z = 0.0;
        pose1.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped pose2;
        pose2.pose.position.x = p2.x();
        pose2.pose.position.y = p2.y();
        pose2.pose.position.z = 0.0;
        pose2.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose1);
        path_msg.poses.push_back(pose2);

        return path_msg;
    }

    std::vector<Eigen::Vector2d> path_points_;
    Eigen::Vector2d final_waypoint_;
    bool station_keeping_active_ = false;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr crosstrack_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr los_vector_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LosSteeringNode>());
    rclcpp::shutdown();
    return 0;
}