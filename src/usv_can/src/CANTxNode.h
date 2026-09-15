//
// Created by Abiel on 3/22/23.
// Updated for HAL CAN protocol
//

#ifndef USV_ROS2_CANTXNODE_H
#define USV_ROS2_CANTXNODE_H

#include "Vanttec_CANLib/CANMessage.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <vector>

class CANTxNode : public rclcpp::Node {
 public:
  CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler);

  ~CANTxNode();

 private:
  std::shared_ptr<vanttec::CANHandler> handler{nullptr};

  // Periodic liveness ping to the CAN bus (arb ID 0x1E)
  rclcpp::TimerBase::SharedPtr pingTimer;

  // Dedicated write thread — drains the CANHandler write queue
  std::thread canWriteThread;

  // Motor transmission state
  std::vector<float> lastMotorArray;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motorSub;

 protected:
  void send_ping_msg();
  void motorCb(const std_msgs::msg::Float32MultiArray &msg);
};

#endif  // USV_ROS2_CANTXNODE_H
