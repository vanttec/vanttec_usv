//
// Created by Abiel on 3/22/23.
// Updated: motor commands removed (motors are driven directly, not via CAN)
//

#ifndef USV_ROS2_CANTXNODE_H
#define USV_ROS2_CANTXNODE_H

#include "Vanttec_CANLib/CANMessage.h"
#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"

#include <thread>

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

 protected:
  void send_ping_msg();
};

#endif  // USV_ROS2_CANTXNODE_H
