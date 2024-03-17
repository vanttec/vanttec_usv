#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class IndividualThrusterNode : public rclcpp::Node {
public:
    IndividualThrusterNode();

private:
    double map_thruster(double x) const;
    void update_thrust();

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motorPub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftMotorSub, rightMotorSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shootSub;
    double left_thrust{0}, right_thrust{0};

    bool stop_on_timeout{true};
    double shoot{0};
    rclcpp::Time last_recev;

    rclcpp::TimerBase::SharedPtr update_timer;
};