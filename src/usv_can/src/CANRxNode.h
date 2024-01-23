//
// Created by Abiel on 3/22/23.
//

#ifndef USV_ROS2_CANRXNODE_H
#define USV_ROS2_CANRXNODE_H

#include "Vanttec_CANLib/Utils/CANDeserialization.h"
#include "Vanttec_CANLib/Utils/CANSerialization.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"

class CANRxNode : public rclcpp::Node {
 public:
  CANRxNode(const std::shared_ptr<vanttec::CANHandler> &handler);

  void update();

 private:
  std::shared_ptr<vanttec::CANHandler> handler{nullptr};
  rclcpp::TimerBase::SharedPtr updateTimer;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pingPublisher, modePublisher;

  std::stringstream debugLogBuffer;

 protected:
  void handleModeMsg(can_frame frame);

  void handleDebugMsg(can_frame frame);

  void handlePingMsg(can_frame frame);
};

#endif  // USV_ROS2_CANRXNODE_H
