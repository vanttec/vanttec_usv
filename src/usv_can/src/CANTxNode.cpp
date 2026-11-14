//
// Created by Abiel on 3/22/23.
// Uses the STM32 shared CAN payload protocol.
//

#include "CANTxNode.h"
#include <cstring>
#include <thread>

using namespace std::chrono_literals;

namespace {
constexpr uint32_t kJetsonHeartbeatId = 0x01E;
constexpr uint32_t kMotorGroupBaseId = 0x120;
}

CANTxNode::CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANTxNode") {
  RCLCPP_INFO(this->get_logger(), "Starting CAN TX Node");
  this->handler = handler;

  lastMotorArray = std::vector<float>(8, 0.0f);

  motorSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "motors", 10, std::bind(&CANTxNode::motorCb, this, std::placeholders::_1));

  canWriteThread = std::thread([this] {
    while (rclcpp::ok()) {
      this->handler->update_write();
      std::this_thread::sleep_for(1ms);
    }
  });

  // Updated back to 100ms (10Hz) to perfectly match the original code
  pingTimer =
      this->create_wall_timer(100ms, std::bind(&CANTxNode::send_ping_msg, this));
}

CANTxNode::~CANTxNode() {
  if (canWriteThread.joinable()) canWriteThread.join();
}

void CANTxNode::send_ping_msg() {
  vanttec::CANMessage msg;
  msg.arb_id = kJetsonHeartbeatId;
  msg.data[0] = 0x01;
  msg.len = 1;
  handler->write(msg);
}

void CANTxNode::motorCb(const std_msgs::msg::Float32MultiArray &msg) {
  if (msg.data.size() != 8) {
    RCLCPP_ERROR(this->get_logger(), "Invalid motor array size");
    return;
  }

  for (size_t group = 0; group < 4; group++) {
    vanttec::CANMessage canMsg;
    canMsg.arb_id = kMotorGroupBaseId + group;
    std::memcpy(canMsg.data, &msg.data[group * 2], sizeof(float) * 2);
    canMsg.len = sizeof(float) * 2;

    if (msg.data[group * 2] != lastMotorArray[group * 2] ||
        msg.data[group * 2 + 1] != lastMotorArray[group * 2 + 1]) {
      handler->write(canMsg);
    }
  }

  lastMotorArray = msg.data;
}