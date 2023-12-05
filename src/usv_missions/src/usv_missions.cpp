#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class USVMissions : public rclcpp::Node {
    public:
        USVMissions(): Node("usv_missions") {
            current_mission_sub_ = this->create_subscription<std_msgs::msg::Int8>(
                "/usv_master/current_mission", 10, std::bind(&USVMissions::curr_mission_callback, this, _1)
            );
            
            vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32>("/usv_missions/velodine_setpoint", 10);

            waypoint_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/usv_missions/waypoint", 10);

            mission_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv_missions/mission_status", 10);
        
            timer_ = this->create_wall_timer(
            500ms, std::bind(&USVMissions::timer_callback, this));

        }

    private:

        int current_miss = -1;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr current_mission_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_setpoint_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_;

        void timer_callback() {
            auto status_message = std_msgs::msg::Int8();
            // message.data = "Hello, world! " + std::to_string(count_++);
            status_message.data = 2;
            // RCLCPP_INFO(this->get_logger(), "Publishing mission: '%d'", message.data);
            mission_status_pub_->publish(status_message);
        }

        void curr_mission_callback(const std_msgs::msg::Int8 & msg) {
            current_miss = msg.data;

            if (current_miss < 0 || current_miss > 8) return;

            switch (current_miss) {
                case 1:
                    RCLCPP_INFO(this->get_logger(), "Navigating, doing mission: '%d'", current_miss);
                break;
                
                case 2:
                    RCLCPP_INFO(this->get_logger(), "Searching for target o alo asi, mission: '%d'", current_miss);
                break;

                case 3:
                    RCLCPP_INFO(this->get_logger(), "Going through passage? mission: '%d'", current_miss);
                break;

                default:
                    RCLCPP_INFO(this->get_logger(), "DEFAULT Current mission: '%d'", current_miss);
                break;
            }

            
        }
        
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVMissions>());
  rclcpp::shutdown();
  return 0;
}