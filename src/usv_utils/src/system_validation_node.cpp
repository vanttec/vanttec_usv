#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>
#include <map>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "sbg_driver/msg/sbg_gps_hdt.hpp"

#include "usv_interfaces/msg/system_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SystemStatusNode : public rclcpp::Node {
    public:
        SystemStatusNode(): Node("system_validation_node") {

            system_status_pub_ = this->create_publisher<usv_interfaces::msg::SystemStatus>("/usv/status", 10);

            sbg_ekf_nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
                "/sbg/ekf_nav", 10,
                [this](const sbg_driver::msg::SbgEkfNav &msg){
                    out.ekf_status = msg.status.solution_mode;
                });

            sbg_gps_pos_sub_ = this->create_subscription<sbg_driver::msg::SbgGpsPos>(
                "/sbg/gps_pos", 10,
                [this](const sbg_driver::msg::SbgGpsPos &msg){
                    out.gps_pos_status = msg.status.type;
                });

            sbg_gps_hdt_sub_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>(
                "/sbg/gps_hdt", 10,
                [this](const sbg_driver::msg::SbgGpsHdt &msg){
                    out.gps_hdt_status = msg.status;
                });
            
            can_op_mode_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
                "/usv/op_mode", 10,
                [this](const std_msgs::msg::UInt16 &msg){
                    out.op_mode = msg.data;
                    last_recev = this->now();
                });
            
            last_recev = this->now();

            timer_ = this->create_wall_timer(
            100ms, std::bind(&SystemStatusNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<usv_interfaces::msg::SystemStatus>::SharedPtr system_status_pub_;
        
        rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr sbg_ekf_nav_sub_;
        rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr sbg_gps_pos_sub_;
        rclcpp::Subscription<sbg_driver::msg::SbgGpsHdt>::SharedPtr sbg_gps_hdt_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr can_op_mode_sub_;

        usv_interfaces::msg::SystemStatus out;

        rclcpp::Time last_recev;
              
        void timer_callback() {

            rclcpp::Duration diff = this->now() - last_recev;
            int elapsed_ms = diff.nanoseconds() / 1000000;
            if(elapsed_ms > 500){
                out.can_stm_status = false;
            } else {
                out.can_stm_status = true;
            }

            system_status_pub_->publish(out);
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemStatusNode>());
  rclcpp::shutdown();
  return 0;
}

