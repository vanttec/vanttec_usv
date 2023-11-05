#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
int mis = 0;

class USVMaster : public rclcpp::Node {
  public:
    USVMaster(): Node("usv_master"), mission(0) {
      mission_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv_master/current_mission", 10);

      xbee_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/usv_comms/xbee_command", 10, std::bind(&USVMaster::xbee_callback, this, _1)
      );

      mission_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/usv_misisons/mission_status", 10, std::bind(&USVMaster::mission_callback, this, _1)
      );

      timer_ = this->create_wall_timer(
      500ms, std::bind(&USVMaster::timer_callback, this));
    }

  private:


  int mission;

    void timer_callback() {
      auto message = std_msgs::msg::Int8();
      message.data = 0;
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      mission_pub_->publish(message);
    }

    void xbee_callback(const std_msgs::msg::String & msg)  {
        std::string a = msg.data.c_str();
      
      if (msg.data[0] == 'm') {
        mission = a[2] - '0';
        RCLCPP_INFO(this->get_logger(), "Mission: '%d'", mission);

      } else {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      }

    }

    void mission_callback(const std_msgs::msg::String & msg)  {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr xbee_sub_;
};

int main(int argc, char * argv[])
{
  std::cout << "aaa";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVMaster>());
  rclcpp::shutdown();
  return 0;
}