//
// Created by Abiel on 3/22/23.
// Updated: motor commands removed (motors are driven directly, not via CAN)
//

#include "CANTxNode.h"
#include <thread>

using namespace std::chrono_literals;

CANTxNode::CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANTxNode") {
  RCLCPP_INFO(this->get_logger(), "Starting CAN TX Node");
  this->handler = handler;

  // Dedicated write thread — drains the write queue at 1 kHz
  canWriteThread = std::thread([this] {
    while (rclcpp::ok()) {
      this->handler->update_write();
      std::this_thread::sleep_for(1ms);
    }
  });

  // Send a liveness ping onto the CAN bus at 1 Hz (arb ID 0x1E)
  pingTimer =
      this->create_wall_timer(1000ms, std::bind(&CANTxNode::send_ping_msg, this));
}

CANTxNode::~CANTxNode() {
  if (canWriteThread.joinable()) canWriteThread.join();
}

void CANTxNode::send_ping_msg() {
  vanttec::CANMessage msg;
  msg.arb_id = 0x1E;      // Jetson liveness ping arbitration ID
  msg.data[0] = 0x01;     // simple alive flag
  msg.len = 1;
  handler->write(msg);
}