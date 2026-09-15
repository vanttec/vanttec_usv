#include <cstdio>

#include "CANRxNode.h"
#include "CANTxNode.h"
#include "individual_thrusters_node.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // CAN interface name is a ROS2 parameter, default "can0"
  auto paramNode = std::make_shared<rclcpp::Node>("can_param_node");
  paramNode->declare_parameter<std::string>("can_interface", "can0");
  std::string canInterface = paramNode->get_parameter("can_interface").as_string();
  RCLCPP_INFO(paramNode->get_logger(), "Opening CAN interface: %s", canInterface.c_str());

  auto handler = std::make_shared<vanttec::CANHandler>(canInterface);
  auto txNode = std::make_shared<CANTxNode>(handler);
  auto rxNode = std::make_shared<CANRxNode>(handler);
  
  // Restored: Handles math scaling and 250ms safety timeouts for the motors
  auto thrusterNode = std::make_shared<IndividualThrusterNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(txNode);
  executor.add_node(rxNode);
  executor.add_node(thrusterNode);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
