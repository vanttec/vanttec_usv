#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace std::chrono_literals;

class KillSwitchSimNode : public rclcpp::Node {
public:
  KillSwitchSimNode() : Node("killswitch_sim_node"){
    using namespace std::placeholders;

    leftSub = this->create_subscription<std_msgs::msg::Float64>(
      "/usv/left_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->left.data = msg.data * 1.0; });

    rightSub = this->create_subscription<std_msgs::msg::Float64>(
      "/usv/right_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->right.data = msg.data * 1.0; });

    arrivedSub = this->create_subscription<std_msgs::msg::Bool>(
      "/usv/waypoint/arrived", 10,
        [this](const std_msgs::msg::Bool &msg) { this->arrived.data = msg.data; });

    leftPub = this->create_publisher<std_msgs::msg::Float64>("/model/vtec_s3/joint/left_engine_propeller_joint/cmd_thrust", 10);
    rightPub = this->create_publisher<std_msgs::msg::Float64>("/model/vtec_s3/joint/right_engine_propeller_joint/cmd_thrust", 10);

    service = this->create_service<std_srvs::srv::Empty>("auto", std::bind(&KillSwitchSimNode::autonomous, this, _1, _2));

    updateTimer = this->create_wall_timer(10ms, std::bind(&KillSwitchSimNode::update, this));
  }

protected:
  void autonomous(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
              std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    if(this->autonomous_on)
      this->autonomous_on = false;
    else 
      this->autonomous_on = true;
    
    RCLCPP_INFO(get_logger(), "Setting autonomous as: %s", autonomous_on ? "true" : "false");
  }

private:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leftPub, rightPub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftSub, rightSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrivedSub;

  std_msgs::msg::Float64 left, right, zero;
  std_msgs::msg::Bool arrived;

  bool autonomous_on{false};

  rclcpp::TimerBase::SharedPtr updateTimer;

  void update() {
    // if(!this->arrived.data ^ this->autonomous_on){
      leftPub->publish(this->right);
      rightPub->publish(this->left);
    // } else{
    //   leftPub->publish(zero);
    //   rightPub->publish(zero);
    // }
  }
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KillSwitchSimNode>());
  rclcpp::shutdown();
  return 0;
}