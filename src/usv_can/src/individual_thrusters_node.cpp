#include "individual_thrusters_node.h"

IndividualThrusterNode::IndividualThrusterNode() : Node("IndividualThrusterNode") {
    stop_on_timeout = this->declare_parameter<bool>("stop_on_timeout", true);
    RCLCPP_INFO(this->get_logger(), "stop_on_timeout: %d", stop_on_timeout);

    leftMotorSub = this->create_subscription<std_msgs::msg::Float64>(
        "in/left_motor", 10,
        [this](const std_msgs::msg::Float64 &msg){
            this->left_thrust = map_thruster(msg.data);
            this->last_recev = this->now();
            // this->update_thrust();
        }
    );

    shootSub = this->create_subscription<std_msgs::msg::Bool>(
        "/shoot", 10,
        [this](const std_msgs::msg::Bool &msg){
            if(msg.data)
                this->shoot = 1.0;
            else
                this->shoot = 0.0;
        }
    );

    rightMotorSub = this->create_subscription<std_msgs::msg::Float64>(
        "in/right_motor", 10,
        [this](const std_msgs::msg::Float64 &msg){
            this->right_thrust = map_thruster(msg.data);
            this->last_recev = this->now();
            // this->update_thrust();
        }
    );

    motorPub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "motors", 10
    );

    last_recev = this->now();
    using namespace std::chrono_literals;
    update_timer = this->create_wall_timer(25ms, std::bind(&IndividualThrusterNode::update_thrust, this));
}

double IndividualThrusterNode::map_thruster(double x) const {
    if(x > 36.5){
        x = 1;
    } else if(x < -30){
        x = -1;
    } else if(x > 0){
        x /= 36.5;
    } else {
        x /= 30;
    }

    return x;
}

void IndividualThrusterNode::update_thrust(){
    rclcpp::Duration diff = this->now() - last_recev;
    int elapsed_ms = diff.nanoseconds() / 1000000;
    auto &clock = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 10000, "%d", elapsed_ms);
    if(elapsed_ms > 250){
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 10000, "Setting motors to 0, no input recieved in %d!", elapsed_ms);
        left_thrust = 0;
        right_thrust = 0;
        shoot = 0;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = std::vector<float>{
        static_cast<float>(left_thrust), 
        static_cast<float>(left_thrust),

        static_cast<float>(shoot), 
        0,0,0,

        static_cast<float>(-right_thrust), 
        static_cast<float>(-right_thrust),
        };

    motorPub->publish(msg);
}
