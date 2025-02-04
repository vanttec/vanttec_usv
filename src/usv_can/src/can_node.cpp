#include <cstdio>

#include "CANRxNode.h"
#include "CANTxNode.h"
#include "individual_thrusters_node.h"
#include "Vanttec_CANLib/Utils/CANDeserialization.h"
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // TODO move can to parameter
  auto handler = std::make_shared<vanttec::CANHandler>("can_vtec");
  auto txNode = std::make_shared<CANTxNode>(handler);
  auto rxNode = std::make_shared<CANRxNode>(handler);
  auto thrusterNode = std::make_shared<IndividualThrusterNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(txNode);
  executor.add_node(rxNode);
  executor.add_node(thrusterNode);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
