#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MagOrientation : public rclcpp::Node {
public:
  MagOrientation() : Node("mag_orientation_node") {
    subscription = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
      "/sbg/ekf_quat", 10, std::bind(&MagOrientation::sbg_callback, this, std::placeholders::_1));

    publisher = this->create_publisher<std_msgs::msg::Float64>("/heading", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo de Orientación iniciado.");
  }

private:
  void sbg_callback(const sbg_driver::msg::SbgEkfQuat::SharedPtr msg) {

    //bool ready = (msg->status.solution_mode >= 2) &&
                 //(msg->status.heading_valid) &&
                 //(msg->status.mag_ref_used);

    //if (!ready) {
     // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
       // "DATOS NO CONFIABLES - Modo: %d | Heading: %s | Mag: %s",
        //msg->status.solution_mode,
        //msg->status.heading_valid ? "OK" : "INVALID",
        //msg->status.mag_ref_used ? "USED" : "NOT USED");
      //return;
    //}

    tf2::Quaternion q(
      msg->quaternion.x,
      msg->quaternion.y,
      msg->quaternion.z,
      msg->quaternion.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (yaw < 0) yaw += 2.0 * M_PI;

    auto message = std_msgs::msg::Float64();
    message.data = yaw;
    publisher->publish(message);

    RCLCPP_INFO(this->get_logger(), "Heading: %.3f rad (%.2f * PI)", yaw, yaw / M_PI);
  }

  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagOrientation>());
  rclcpp::shutdown();
  return 0;
}
