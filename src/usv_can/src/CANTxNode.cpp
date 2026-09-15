//
// Created by Abiel on 3/22/23.
// Updated for HAL CAN protocol
//

#include "CANTxNode.h"
#include <thread>
#include <cstring>

using namespace std::chrono_literals;

CANTxNode::CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANTxNode") {
  RCLCPP_INFO(this->get_logger(), "Starting CAN TX Node");
  this->handler = handler;

  // Initialize motor array with 8 zeroes (fixes previous {0, 8} bug)
  lastMotorArray = std::vector<float>(8, 0.0f);

  // Subscribe to the motors topic produced by IndividualThrusterNode
  motorSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "motors", 10, std::bind(&CANTxNode::motorCb, this, std::placeholders::_1));

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

void CANTxNode::motorCb(const std_msgs::msg::Float32MultiArray &msg) {
  for (size_t i = 0; i < msg.data.size() && i < 8; i++) {
    // Only send the CAN frame if the motor value has changed
    if (msg.data[i] != lastMotorArray[i]) {
      lastMotorArray[i] = msg.data[i];
      
      vanttec::CANMessage canMsg;
      canMsg.arb_id = 0x110 + i; // Motor 0 = 0x110, Motor 1 = 0x111, etc.
      std::memcpy(canMsg.data, &msg.data[i], sizeof(float)); // raw 4-byte float LE
      canMsg.len = sizeof(float);
      
      handler->write(canMsg);
    }
  }
}