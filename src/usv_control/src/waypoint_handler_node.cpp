#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "usv_interfaces/msg/waypoint.hpp"

using namespace std::chrono_literals;

class WpHandlerNode : public rclcpp::Node
{
public:
    WpHandlerNode() : Node("waypoint_handler_node")
    {
        using namespace std::placeholders;

        poseSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "usv/state/pose", 10,
            [this](const geometry_msgs::msg::Pose2D &msg)
            {
                this->pose.x = msg.x;
                this->pose.y = msg.y;
                // this->pose.theta = -msg.theta; // For SBG
                this->pose.theta = msg.theta; // For Simulations
            });

        waypointSub = this->create_subscription<usv_interfaces::msg::Waypoint>(
            "usv/waypoint", 10,
            [this](const usv_interfaces::msg::Waypoint &msg)
            {
                if(msg.x != this->wp.x || msg.y != this->wp.y){
                    this->arrived.data = false;
                    this->wp.x = msg.x;
                    this->wp.y = msg.y;
                }
            });

        desired_pivot_Sub = this->create_subscription<std_msgs::msg::Bool>(
            "usv/waypoint/pivot", 10,
            [this](const std_msgs::msg::Bool &msg)
            {
                this->pivot.data = msg.data;
            });

        headingDesPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_heading", 10);

        pivotPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/pivot_enable", 10);

        arrivedPub = this->create_publisher<std_msgs::msg::Bool>(
            "/usv/waypoint/arrived", 10);

        headingErrorPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/error_heading", 10);

        desiredPosePub = this->create_publisher<usv_interfaces::msg::Waypoint>(
            "/usv/wp/desired_pose", 10);

        velocityDesPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_velocity", 10);

        wpErrorPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/error_distance", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&WpHandlerNode::update, this));
    }


private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;
    rclcpp::Subscription<usv_interfaces::msg::Waypoint>::SharedPtr waypointSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr desired_pivot_Sub;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr headingDesPub, pivotPub, velocityDesPub, headingErrorPub, wpErrorPub;
    rclcpp::Publisher<usv_interfaces::msg::Waypoint>::SharedPtr desiredPosePub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrivedPub;

    geometry_msgs::msg::Pose2D pose;
    usv_interfaces::msg::Waypoint wp;
    std_msgs::msg::Float64 heading_d, velocity_d, heading_e, distance_e, pivot_set;

    std_msgs::msg::Bool arrived, pivot;

    rclcpp::TimerBase::SharedPtr updateTimer;

    void update() {
        float x_diff, y_diff, head, dist;
        float PI = M_PI + 1e-3;

        velocity_d.data = 0;
        pivot_set.data = 0;
        if(pivot.data){
            arrived.data = false;
            heading_d.data = 6*M_PI;
            pivot_set.data = 1.0;
        } else if(!arrived.data){
            x_diff = wp.x - pose.x;
            y_diff = wp.y - pose.y;
            dist = std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2));
            distance_e.data = dist;

            if(dist > 0.7){
                head = std::atan2(y_diff, x_diff);
                velocity_d.data = std::clamp((double)(dist / 2), 0.125, 1.0);
                
                // Get the right absolute direction to go to in (-pi, pi) range
                if(x_diff < 0){
                    head = std::atan2(std::abs(y_diff), std::abs(x_diff));
                    if(y_diff < 0)
                        head = - PI + head;
                    else if(y_diff > 0)
                        head = PI - head;
                    else
                        head = PI;
                }

                // Get to the direction in the least amount of radians
                if(std::fabs(head - pose.theta) > M_PI){
                    if(head < pose.theta)
                        head+=2*PI;
                    else 
                        head-=2*PI;
                }

                RCLCPP_INFO(rclcpp::get_logger("wp_node"), "h: %f, t: %f", head, pose.theta);
                heading_d.data = head;
                heading_e.data = head - pose.theta;
                headingErrorPub->publish(heading_e);
            } else
                arrived.data = true;
            velocityDesPub->publish(velocity_d);
        }


        headingDesPub->publish(heading_d);
        wpErrorPub->publish(distance_e);
        desiredPosePub->publish(wp);
        arrivedPub->publish(arrived);
        pivotPub->publish(pivot_set);
  }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WpHandlerNode>());
    rclcpp::shutdown();
    return 0;
}