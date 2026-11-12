//
// Created by Abiel on 3/22/23.
// Updated for HAL CAN protocol (DashboardLumos STM32 firmware)
//
// CAN ID Map (arbitration ID -> message):
//   0x010 — STM32 uptime (uint32 LE), mode, auxiliary selector, armed flag
//   0x108 — STM32 local battery monitor voltage (float32 LE)
//   0x150 — Actuator Control PCB (byte[0]=actuator_active)
//   0x200 — Battery Control PCB (float voltage, LE, 4 bytes)
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
      this->create_publisher<std_msgs::msg::UInt32>("out/stm32_heartbeat", 1);
  modePublisher = this->create_publisher<std_msgs::msg::UInt8>("out/stm32_mode", 1);
  auxiliaryPublisher = this->create_publisher<std_msgs::msg::UInt8>("out/stm32_auxiliary", 1);
  armedPublisher = this->create_publisher<std_msgs::msg::UInt8>("out/stm32_armed", 1);
  batteryPublisher =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("out/battery", 1);
  stm32BatteryPublisher =
      this->create_publisher<std_msgs::msg::Float32>("out/stm32_battery_voltage", 1);
  actuatorPublisher =
      this->create_publisher<std_msgs::msg::UInt8MultiArray>("out/actuators", 1);

  // Register parsers by HAL CAN arbitration ID
  handler->register_parser(0x010,
                            std::bind(&CANRxNode::handleHeartbeatMsg, this, _1));
  handler->register_parser(0x108,
                            std::bind(&CANRxNode::handleStm32BatteryMsg, this, _1));
  handler->register_parser(0x200,
                            std::bind(&CANRxNode::handleBatteryMsg, this, _1));
  handler->register_parser(0x150,
                            std::bind(&CANRxNode::handleActuatorMsg, this, _1));
}

void CANRxNode::update() { handler->update_read(); }

// ID 0x010 — 4-byte little-endian uint32 HAL_GetTick() uptime in ms
void CANRxNode::handleHeartbeatMsg(can_frame frame) {
  if (frame.can_dlc < 7) return;

  uint32_t uptime = static_cast<uint32_t>(frame.data[0])
                  | (static_cast<uint32_t>(frame.data[1]) << 8)
                  | (static_cast<uint32_t>(frame.data[2]) << 16)
                  | (static_cast<uint32_t>(frame.data[3]) << 24);

  std_msgs::msg::UInt32 msg;
  msg.data = uptime;
  pingPublisher->publish(msg);

  std_msgs::msg::UInt8 mode;
  mode.data = frame.data[4];
  modePublisher->publish(mode);

  std_msgs::msg::UInt8 auxiliary;
  auxiliary.data = frame.data[5];
  auxiliaryPublisher->publish(auxiliary);

  std_msgs::msg::UInt8 armed;
  armed.data = frame.data[6];
  armedPublisher->publish(armed);
  RCLCPP_DEBUG(this->get_logger(), "STM32 heartbeat: %u ms", uptime);
}

// Decode IEEE-754 float32 from little-endian wire bytes on any host endian.
static float decode_float32_le(const __u8 *bytes) {
  const uint32_t bits = static_cast<uint32_t>(bytes[0])
                      | (static_cast<uint32_t>(bytes[1]) << 8)
                      | (static_cast<uint32_t>(bytes[2]) << 16)
                      | (static_cast<uint32_t>(bytes[3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// ID 0x200 — 4 bytes: float battery voltage, little-endian
void CANRxNode::handleBatteryMsg(can_frame frame) {
  if (frame.can_dlc < 4) return;

  float voltage = decode_float32_le(frame.data);

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {voltage};
  batteryPublisher->publish(msg);
}

void CANRxNode::handleStm32BatteryMsg(can_frame frame) {
  if (frame.can_dlc < 4) return;
  std_msgs::msg::Float32 msg;
  msg.data = decode_float32_le(frame.data);
  stm32BatteryPublisher->publish(msg);
}

// ID 0x150 — byte[0]=actuator_active
void CANRxNode::handleActuatorMsg(can_frame frame) {
  if (frame.can_dlc < 1) return;

  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {frame.data[0]};
  actuatorPublisher->publish(msg);
}
