#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "model/dynamic_model.h"

using namespace std::chrono_literals;

class DynamicModelSim : public rclcpp::Node {
 public:
  DynamicModelSim() : Node("Dynamic_Model_Sim") {
    using namespace std::placeholders;

    posePub =
        this->create_publisher<geometry_msgs::msg::Pose2D>("output/pose", 10);

    localVelPub =
        this->create_publisher<geometry_msgs::msg::Vector3>("output/vel", 10);
    odomPub =
        this->create_publisher<nav_msgs::msg::Odometry>("output/odom", 10);

    leftThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
        "input/left_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->Tport = msg.data; });

    rightThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
        "input/right_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->Tstbd = msg.data; });

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    updateTimer = this->create_wall_timer(
        10ms, std::bind(&DynamicModelSim::update, this));
  }

 protected:
  void update() {
    auto out = model.update(Tport, Tstbd);

    /**
     * Output stage
     */
    double x = out.pose_x;  // position in x
    double y = out.pose_y;  // position in y
    double etheta = out.pose_psi;

    tf2::Quaternion q;
    q.setRPY(0, 0, etheta);

    geometry_msgs::msg::Pose2D pose;
    nav_msgs::msg::Odometry odom;

    if(etheta > 0)
      etheta -= std::floor(etheta / (M_PI * 2)) * M_PI * 2;
    else
      etheta -= std::ceil(etheta / (M_PI * 2)) * M_PI * 2;
    
    if(etheta > M_PI)
      etheta -= 2 * M_PI;
    else if(etheta < -M_PI)
      etheta += 2 * M_PI;

    pose.x = x;
    pose.y = y;
    pose.theta = etheta;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = q[0];
    odom.pose.pose.orientation.y = q[1];
    odom.pose.pose.orientation.z = q[2];
    odom.pose.pose.orientation.w = q[3];

    posePub->publish(pose);

    double u, v, r;

    geometry_msgs::msg::Vector3 velMsg;

    u = out.u;  // surge velocity
    v = out.v;  // sway velocity
    r = out.r;  // yaw rate
    velMsg.x = u;
    velMsg.y = v;
    velMsg.z = r;
    odom.twist.twist.linear.x = u;
    odom.twist.twist.linear.y = v;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = r;

    odomPub->publish(odom);
    localVelPub->publish(velMsg);

    tf_broadcast(pose);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr posePub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr localVelPub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  rclcpp::TimerBase::SharedPtr updateTimer;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftThrusterSub,
      rightThrusterSub;
  double Tport{0}, Tstbd{0};

  // DynamicModel model{0,0,0};
  DynamicModel model{12,15,-2.3};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  void tf_broadcast(const geometry_msgs::msg::Pose2D &msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "usv";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg.x;
    t.transform.translation.y = msg.y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster->sendTransform(t);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicModelSim>());
  rclcpp::shutdown();
  return 0;
}