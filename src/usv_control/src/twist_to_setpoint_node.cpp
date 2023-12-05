#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class TwistToSetpoint : public rclcpp::Node {
 public:
  TwistToSetpoint() : Node("twist_to_setpoint_node") {
    using namespace std::placeholders;

    velocitySetpointPub =
        this->create_publisher<std_msgs::msg::Float64>("velocity", 10);

    headingSetpointPub =
        this->create_publisher<std_msgs::msg::Float64>("heading", 10);

    velMsg.data = 0;
    headingMsg.data = 0;

    velocitySub = this->create_subscription<geometry_msgs::msg::Twist>(
        "velocity_twist", 10, [this](const geometry_msgs::msg::Twist &msg) {
          // linear.x as velocity Use angular.z as heading
          velMsg.data += msg.linear.x;
          headingMsg.data += msg.angular.z;

          this->velocitySetpointPub->publish(velMsg);
          this->headingSetpointPub->publish(headingMsg);
        });
  }

 private:
  std_msgs::msg::Float64 velMsg, headingMsg;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocitySetpointPub,
      headingSetpointPub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocitySub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToSetpoint>());
  rclcpp::shutdown();
  return 0;
}