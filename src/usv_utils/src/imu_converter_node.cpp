#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <functional>
#include <chrono>
using namespace std::chrono_literals;


class ImuConverterNode : public rclcpp::Node {
public:
  ImuConverterNode() : Node("imu_converter_node"){
    using namespace std::placeholders;

    convertedOdomPub = 
      this->create_publisher<nav_msgs::msg::Odometry>("/out/converted_odometry", 10);

    odomSub = this->create_subscription<nav_msgs::msg::Odometry>("in/odometry", 10,
      std::bind(&ImuConverterNode::odom_cb, this, _1));

    posePub = this->create_publisher<geometry_msgs::msg::Pose2D>("out/pose", 10);

    velocityPub = this->create_publisher<geometry_msgs::msg::Vector3>("out/velocity", 10);

    service = this->create_service<std_srvs::srv::Empty>("zero_imu", std::bind(&ImuConverterNode::zero_imu, this, _1, _2));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&ImuConverterNode::timer_callback, this));
  }

protected:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
    // TODO process imu, and publish new rotated and zero imu
    nav_msgs::msg::Odometry newOdom = *msg;
    tf2::Quaternion q(newOdom.pose.pose.orientation.x, newOdom.pose.pose.orientation.y,
                      newOdom.pose.pose.orientation.z, newOdom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double psi = yaw;

    newOdom.header.stamp = msg->header.stamp;
    newOdom.header.frame_id = msg->header.frame_id;

    newOdom.pose.pose.position.x = msg->pose.pose.position.y - zero_x;
    newOdom.pose.pose.position.y = msg->pose.pose.position.x - zero_y;

    double u_orig = msg->twist.twist.linear.x;
    double v_orig = -msg->twist.twist.linear.y;

/*    newOdom.twist.twist.linear.x =
        std::cos(psi) * u_orig - std::sin(psi) * v_orig;
    newOdom.twist.twist.linear.y =
        -(std::sin(psi) * u_orig + std::cos(psi) * v_orig);
    newOdom.twist.twist.angular.z = msg->twist.twist.angular.z;*/
    newOdom.twist.twist.linear.x =
        u_orig;
    newOdom.twist.twist.linear.y =
        v_orig;
    newOdom.twist.twist.angular.z = msg->twist.twist.angular.z;


    convertedOdomPub->publish(newOdom);
    last_odom = newOdom;
    if(!hasZeroInit){
      zero_imu(nullptr, nullptr);
      hasZeroInit = true;
    }

    geometry_msgs::msg::Pose2D pose;
    pose.x = newOdom.pose.pose.position.x;
    pose.y = newOdom.pose.pose.position.y;
    pose.theta = psi;
    posePub->publish(pose);

    geometry_msgs::msg::Vector3 velocity;
    velocity.x = newOdom.twist.twist.linear.x;
    velocity.y = newOdom.twist.twist.linear.y;
    velocity.z = newOdom.twist.twist.angular.z;
    velocityPub->publish(velocity);
  }

  void tf_broadcast(const geometry_msgs::msg::Pose2D &msg){
    // TODO send tf broadcast to new usv stamp
  }

  void zero_imu(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
              std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    zero_x = last_odom.pose.pose.position.x + zero_x;
    zero_y = last_odom.pose.pose.position.y + zero_y;
    RCLCPP_INFO(get_logger(), "Setting x: %f y: %f as new zero", zero_x, zero_y);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr convertedOdomPub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr posePub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityPub;

  nav_msgs::msg::Odometry last_odom;

  bool hasZeroInit{false};
  double zero_x{0}, zero_y{0};

  void timer_callback() {
    RCLCPP_INFO(get_logger(), "OKAAAYYYY LET'S GO!");
  }        
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuConverterNode>());
  rclcpp::shutdown();
  return 0;
}
