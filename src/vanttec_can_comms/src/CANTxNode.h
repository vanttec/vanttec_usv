//
// Created by Abiel on 3/22/23.
//

#ifndef USV_ROS2_CANTXNODE_H
#define USV_ROS2_CANTXNODE_H

#include "Vanttec_CANLib/Utils/CANDeserialization.h"
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"

class CANTxNode : public rclcpp::Node {
 public:
  CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler);

 protected:
  void send_ping_msg();
  
 private:
  std::shared_ptr<vanttec::CANHandler> handler{nullptr};
  rclcpp::TimerBase::SharedPtr pingTimer;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motorSub;
  std::vector<float> lastMotorArray{0, 8};  // [0] * 8

 protected:
  void motorCb(const std_msgs::msg::Float32MultiArray &msg);
  int i = 0;
  std::thread canWriteThread;
};

#endif  // USV_ROS2_CANTXNODE_H
