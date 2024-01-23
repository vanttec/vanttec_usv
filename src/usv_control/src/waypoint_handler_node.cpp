#include <cmath>
#include <algorithm>

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
                this->pose.theta = -msg.theta;
            });

        waypointSub = this->create_subscription<usv_interfaces::msg::Waypoint>(
            "usv/waypoint", 10,
            [this](const usv_interfaces::msg::Waypoint &msg)
            {
                if(msg.x != this->wp.x || msg.y != this->wp.y){
                    this->arrived = false;
                    this->wp.x = msg.x;
                    this->wp.y = msg.y;
                }
            });

        headingDesPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_heading", 10);

        headingErrorPub = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/error_heading", 10);

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
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr headingDesPub, velocityDesPub, headingErrorPub, wpErrorPub;

    geometry_msgs::msg::Pose2D pose;
    usv_interfaces::msg::Waypoint wp;
    std_msgs::msg::Float64 heading_d, velocity_d, heading_e, distance_e;

    bool arrived;

    rclcpp::TimerBase::SharedPtr updateTimer;

    void update() {
        float x_diff, y_diff, head, dist;
        float PI = M_PI + 1e-3;

        if(!arrived){
            x_diff = wp.x - pose.x;
            y_diff = wp.y - pose.y;
            velocity_d.data = 0;
            dist = std::sqrt(std::pow(x_diff,2) + std::pow(y_diff,2));
            distance_e.data = dist;
            if(dist > 0.25){
                head = std::atan2(y_diff, x_diff);
                velocity_d.data = std::clamp((double)(dist / 2), 0.125, 1.0);
                
                // Get the right direction to go to
                // if(x_diff < 0){
                //     head = std::atan2(std::abs(y_diff), std::abs(x_diff));
                //     if(y_diff < 0)
                //         head = - PI + head;
                //     else if(y_diff > 0)
                //         head = PI - head;
                //     else
                //         head = PI;
                // }

                // if(head > PI)
                //     head-=2*PI;
                // else if(head < -PI)
                //     head+=2*PI;

                // What's happening, is that when the boat is trying to turn 180 degs, whenever it's in the 3rd quadrant, the guidance tells it to go to the fourth, but that
                // requires it to go in the opposite direction, translate it to go further huh

                // if((head > PI/2 && pose.theta < -PI/2) || (pose.theta > PI/2 && head < -PI/2)){
                //     if(head > pose.theta)
                //         head -= 2 * PI;
                //     else
                //         head += 2 * PI;
                // }


                this->heading_d.data = head;
                this->heading_e.data = head - pose.theta;
                RCLCPP_INFO(get_logger(), "h: %f, t: %f, e: %f", head, pose.theta, head - pose.theta);
                headingDesPub->publish(heading_d);
                headingErrorPub->publish(heading_e);
            } else
                arrived = true;
            velocityDesPub->publish(velocity_d);
            wpErrorPub->publish(distance_e);
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