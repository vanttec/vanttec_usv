//
// Created by Abiel on 3/22/23.
//

#include "CANTxNode.h"
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

CANTxNode::CANTxNode(const std::shared_ptr<vanttec::CANHandler> &handler)
    : Node("CANTxNode") {
  RCLCPP_INFO(this->get_logger(), "Starting CAN TX Node");
  this->handler = handler;

  motorSub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "motors", 10, std::bind(&CANTxNode::motorCb, this, _1));

  canWriteThread = std::thread([this]{
    using namespace std::chrono_literals;
    while(rclcpp::ok()){
      this->handler->update_write();
      std::this_thread::sleep_for(1ms);
    }
  });

  pingTimer = 
      this->create_wall_timer(100ms, std::bind(&CANTxNode::send_ping_msg, this));
}


void CANTxNode::send_ping_msg() {
  vanttec::CANMessage msg;
  vanttec::packByte(msg, 0x1E, 0.01);
  handler->write(msg);
}

void CANTxNode::motorCb(const std_msgs::msg::Float32MultiArray &msg) {
  if (msg.data.size() != 8) {
    RCLCPP_ERROR(this->get_logger(), "Invalid motor array size");
    return;
  }

  for (size_t i = 0; i < msg.data.size(); i++) {
    if (msg.data[i] == lastMotorArray[i]) continue;
    vanttec::CANMessage canMsg;
    vanttec::packFloat(canMsg, 0x15 + i, msg.data[i]);
    handler->write(canMsg);
  }

  lastMotorArray = msg.data;
}