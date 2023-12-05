#include <cmath>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
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
                this->pose.theta = msg.theta;
            });

        waypointSub = this->create_subscription<usv_interfaces::msg::Waypoint>(
            "usv/waypoint", 10,
            [this](const usv_interfaces::msg::Waypoint &msg)
            {
                if(msg.x != this->wp.x || msg.y != this->wp.y){
                    arrived = false;
                    this->wp.x = msg.x;
                    this->wp.y = msg.y;
                }
            });

        headingDesPub = this->create_publisher<std_msgs::msg::Float64>(
            "/setpoint/heading", 10);

        velocityDesPub = this->create_publisher<std_msgs::msg::Float64>(
            "/setpoint/velocity", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&WpHandlerNode::update, this));
    }


private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;
    rclcpp::Subscription<usv_interfaces::msg::Waypoint>::SharedPtr waypointSub;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr headingDesPub, velocityDesPub;

    geometry_msgs::msg::Pose2D pose;
    usv_interfaces::msg::Waypoint wp;
    std_msgs::msg::Float64 heading_d, velocity_d;

    bool arrived;

    rclcpp::TimerBase::SharedPtr updateTimer;

    void update() {
        float x_diff, y_diff, head, dist;
        if(!arrived){
            x_diff = wp.x - pose.x;
            y_diff = wp.y - pose.y;
            velocity_d.data = 0;
            dist = std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2));
            if(dist > 0.25){
                head = std::atan2(y_diff, x_diff);
                if(head > M_PI)
                    head -= 2*M_PI;
                this->heading_d.data = head;
                velocity_d.data = dist;
                headingDesPub->publish(heading_d);
            } else {
                arrived = true;
            }
            if(dist > 2)
                velocity_d.data = 1.5;
            velocityDesPub->publish(velocity_d);
        }
  }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WpHandlerNode>());
    rclcpp::shutdown();
    return 0;
}