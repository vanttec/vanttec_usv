//
// Created by Abiel on 3/22/23.
// Updated for HAL CAN protocol (DashboardLumos STM32 firmware)
//
// CAN ID Map (arbitration ID → message):
//   0x010 — STM32 telemetry gateway heartbeat (4-byte LE uint32 uptime ms, 1 Hz)
//   0x150 — Actuator Control PCB (byte[0]=pump_active, byte[1]=actuator_active)
//   0x200 — Battery Control PCB (float voltage + float current, LE, 8 bytes)
//

#include "CANRxNode.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

CANRxNode::CANRxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANRxNode") {
  this->handler = handler;

  updateTimer =
      this->create_wall_timer(10ms, std::bind(&CANRxNode::update, this));

  // Publishers
  pingPublisher =
      this->create_publisher<std_msgs::msg::UInt32>("out/stm32_ping", 1);
  batteryPublisher =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("out/battery", 1);
  actuatorPublisher =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>("out/actuators", 1);

  // Register parsers by HAL CAN arbitration ID
  handler->register_parser(0x010,
                            std::bind(&CANRxNode::handleHeartbeatMsg, this, _1));
  handler->register_parser(0x200,
                            std::bind(&CANRxNode::handleBatteryMsg, this, _1));
  handler->register_parser(0x150,
                            std::bind(&CANRxNode::handleActuatorMsg, this, _1));
}

void CANRxNode::update() { handler->update_read(); }

// ID 0x010 — 4-byte little-endian uint32 HAL_GetTick() uptime in ms
void CANRxNode::handleHeartbeatMsg(can_frame frame) {
  if (frame.can_dlc < 4) return;

  uint32_t uptime = static_cast<uint32_t>(frame.data[0])
                  | (static_cast<uint32_t>(frame.data[1]) << 8)
                  | (static_cast<uint32_t>(frame.data[2]) << 16)
                  | (static_cast<uint32_t>(frame.data[3]) << 24);

  std_msgs::msg::UInt32 msg;
  msg.data = uptime;
  pingPublisher->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "STM32 heartbeat: %u ms", uptime);
}

// ID 0x200 — 8 bytes: float voltage (bytes 0-3) + float current (bytes 4-7), little-endian
void CANRxNode::handleBatteryMsg(can_frame frame) {
  if (frame.can_dlc < 8) return;

  float voltage = 0.0f;
  float current = 0.0f;
  std::memcpy(&voltage, &frame.data[0], sizeof(float));
  std::memcpy(&current, &frame.data[4], sizeof(float));

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {voltage, current};
  batteryPublisher->publish(msg);
}

// ID 0x150 — byte[0]=pump_active, byte[1]=actuator_active
void CANRxNode::handleActuatorMsg(can_frame frame) {
  if (frame.can_dlc < 2) return;

  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {frame.data[0], frame.data[1]};
  actuatorPublisher->publish(msg);
}