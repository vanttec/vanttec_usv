#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class KinematicModelSim : public rclcpp::Node {
 public:
  KinematicModelSim() : Node("Kinematic_Model_Sim") {
    using namespace std::placeholders;

    posePub =
        this->create_publisher<geometry_msgs::msg::Pose2D>("output/pose", 10);

    localVelPub =
        this->create_publisher<geometry_msgs::msg::Vector3>("output/vel", 10);
    odomPub =
        this->create_publisher<nav_msgs::msg::Odometry>("output/odom", 10);

    velocitySetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "input/velocity", 10, [this](const std_msgs::msg::Float64 &msg) {
          this->velocitySetpoint = msg.data;
        });

    anglularVelSetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "input/angular_velocity", 10,
        [this](const std_msgs::msg::Float64 &msg) {
          this->angularVelSetpoint = msg.data;
        });

    this->declare_parameters(
        "", std::map<std::string, double>({{"min_angular_velocity", -M_PI},
                                           {"max_angular_velocity", M_PI},
                                           {"min_velocity", 0.0},
                                           {"max_velocity", 1.0}}));

    minAngularVel = this->get_parameter("min_angular_velocity").as_double();
    maxAngularVel = this->get_parameter("max_angular_velocity").as_double();
    minVelocity = this->get_parameter("min_velocity").as_double();
    maxVelocity = this->get_parameter("max_velocity").as_double();

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    localVel.x = 0;
    localVel.y = 0;
    localVel.z = 0;

    updateTimer = this->create_wall_timer(
        10ms, std::bind(&KinematicModelSim::update, this));
  }

 protected:
  void update() {
    // Update velocity based on setpoints
    double T = 1.0 / 2.0;

    localVel.z += T * (angularVelSetpoint - localVel.z);
    localVel.z = std::clamp(localVel.z, minAngularVel, maxAngularVel);

    localVel.x += (T * (velocitySetpoint - localVel.x));
    localVel.x = std::clamp(localVel.x, minVelocity, maxVelocity);
    localVel.y = 0;

    geometry_msgs::msg::Vector3 globalVel;

    Eigen::Matrix3f J;
    Eigen::Vector3f localVelM;
    localVelM << localVel.x, localVel.y, localVel.z;
    J << std::cos(pose.theta), -std::sin(pose.theta), 0, std::sin(pose.theta),
        std::cos(pose.theta), 0, 0, 0, 1;

    // Integrate velocity
    pose.x += globalVel.x * dt;
    pose.y += globalVel.y * dt;
    pose.theta += globalVel.z * dt;
    posePub->publish(pose);

    // Update odom
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odomPub->publish(odom);

    tf_broadcast(pose);
  }

  void tf_broadcast(const geometry_msgs::msg::Pose2D &msg) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "usv";

    t.transform.translation.x = msg.x;
    t.transform.translation.y = msg.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster->sendTransform(t);
  }

  double constrainAngle(double angle) {
    angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr posePub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr localVelPub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;

  static constexpr double dt = 0.01;
  geometry_msgs::msg::Pose2D pose;
  // TODO better name? selfVel = boat frame vel
  geometry_msgs::msg::Vector3 localVel;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocitySetpointSub,
      anglularVelSetpointSub;
  double velocitySetpoint{0}, angularVelSetpoint{0};
  double minAngularVel{-M_PI}, maxAngularVel{M_PI};
  double minVelocity{0}, maxVelocity{1};

  rclcpp::TimerBase::SharedPtr updateTimer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModelSim>());
  rclcpp::shutdown();
  return 0;
}