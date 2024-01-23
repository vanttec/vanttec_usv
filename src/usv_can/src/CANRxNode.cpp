//
// Created by Abiel on 3/22/23.
//

#include "CANRxNode.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

CANRxNode::CANRxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANRxNode") {
  this->handler = handler;

  updateTimer =
      this->create_wall_timer(10ms, std::bind(&CANRxNode::update, this));
  pingPublisher =
      this->create_publisher<std_msgs::msg::UInt16>("out/stm32_ping", 1);
  modePublisher =
      this->create_publisher<std_msgs::msg::UInt16>("out/mode", 10);

  handler->register_parser(0x14,
                           std::bind(&CANRxNode::handleDebugMsg, this, _1));

  handler->register_parser(0x21,
                          std::bind(&CANRxNode::handleModeMsg, this, _1));

  handler->register_parser(0x1F,
                          std::bind(&CANRxNode::handlePingMsg, this, _1));
}

void CANRxNode::update() { handler->update_read(); }

void CANRxNode::handleDebugMsg(can_frame frame) {
  if (frame.can_dlc - 1 <= 0) return;
  if (debugLogBuffer.str().size() > 1000) {
    RCLCPP_ERROR(this->get_logger(),
                 "Debug log has not been flushed!, Clearing Buffer...");
    debugLogBuffer.str(std::string());  // Clear buffer
  }

  for (int i = 1; i < frame.can_dlc; i++) {
    char newChar = frame.data[i];
    if (newChar == '\n') {
      // Flush current buffer
      RCLCPP_INFO(this->get_logger(), "DEBUG LOG: %s",
                  debugLogBuffer.str().c_str());
      debugLogBuffer.str(std::string());  // Clear buffer
    } else {
      debugLogBuffer << newChar;
    }
  }
}

void CANRxNode::handleModeMsg(can_frame frame){
  std_msgs::msg::UInt16 msg;
  msg.data = can_parse_short(frame.data, frame.can_dlc);
  modePublisher->publish(msg);
}

void CANRxNode::handlePingMsg(can_frame frame) {
  std_msgs::msg::UInt16 msg;
  msg.data = can_parse_short(frame.data, frame.can_dlc);
  pingPublisher->publish(msg);
}