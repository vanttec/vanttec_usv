//
// Created by Abiel on 3/22/23.
// Updated for HAL CAN protocol (DashboardLumos STM32 firmware)
//

#ifndef USV_ROS2_CANRXNODE_H
#define USV_ROS2_CANRXNODE_H

#include "Vanttec_CANLib_Linux/CANHandler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <linux/can.h>
#include <cstring>

class CANRxNode : public rclcpp::Node {
 public:
  CANRxNode(const std::shared_ptr<vanttec::CANHandler> &handler);

  void update();

 private:
  std::shared_ptr<vanttec::CANHandler> handler{nullptr};
  rclcpp::TimerBase::SharedPtr updateTimer;

  // STM32 uptime counter: uint32 ms from arb ID 0x010.
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pingPublisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr modePublisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr auxiliaryPublisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr armedPublisher;

  // Direct Battery PCB voltage as float32 array (arb ID 0x200).
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr batteryPublisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stm32BatteryPublisher;

  // Actuator flags: [pump_active, actuator_active] as uint8 array (arb ID 0x150)
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr actuatorPublisher;

 protected:
  // ID 0x010 — uptime uint32 LE, followed by mode, aux selector, and armed bytes.
  void handleHeartbeatMsg(can_frame frame);

  // ID 0x200 — direct Battery PCB voltage, float32 LE.
  void handleBatteryMsg(can_frame frame);
  // ID 0x108 — STM32 local battery monitor: float32 voltage, little-endian.
  void handleStm32BatteryMsg(can_frame frame);

  // ID 0x150 — Actuator Control PCB: byte[0]=pump_active, byte[1]=actuator_active
  void handleActuatorMsg(can_frame frame);
};

#endif  // USV_ROS2_CANRXNODE_H
