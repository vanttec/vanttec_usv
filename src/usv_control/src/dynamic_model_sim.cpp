#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <random>
#include <cstdlib>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "model/dynamic_model.h"

using namespace std::chrono_literals;

class DynamicModelSim : public rclcpp::Node {
 public:
  DynamicModelSim() : Node("Dynamic_Model_Sim") {
    using namespace std::placeholders;

    distribution = std::uniform_real_distribution<double>(-1.0, 1.0);

    // Declare and acquire `boatname` parameter
    boatname_ = this->declare_parameter<std::string>("boatname", "usv");

    posePub =
        this->create_publisher<geometry_msgs::msg::Pose2D>("usv/state/pose", 10);
    localVelPub =
        this->create_publisher<geometry_msgs::msg::Vector3>("usv/state/velocity", 10);
    odomPub =
        this->create_publisher<nav_msgs::msg::Odometry>("output/odom", 10);

    disturbancesPub = 
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/usv/disturbances", 10);

    leftThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
        "usv/left_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { 
          disturbance_msg.data[0] = 0.*distribution(generator);
          this->Tport = msg.data + disturbance_msg.data[0]; 
        });

    rightThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
        "usv/right_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { 
          disturbance_msg.data[1] = 0.*distribution(generator);
          this->Tstbd = msg.data + disturbance_msg.data[1]; 
          });

    pose_path_pub = this->create_publisher<nav_msgs::msg::Path>(
        "usv/pose_path", 10);

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_stamped_tmp_.header.frame_id = "world";
    pose_path.header.frame_id = "world";
    pose_path.header.stamp = DynamicModelSim::now();

    updateTimer = this->create_wall_timer(
        10ms, std::bind(&DynamicModelSim::update, this));

    generator.seed(std::random_device{}());

    disturbance_msg.data.push_back(0.);
    disturbance_msg.data.push_back(0.);
  }

 protected:

  double normalize_angle(double ang){
    double out = std::fmod(ang + M_PI, M_PI*2);
    if(out < 0)
      out+=M_PI*2;
    return out - M_PI;
  }

  void update() {

    auto out = model.update(Tport, Tstbd);

    /**
     * Output stage
     */
    double x = out.pose_x;  // position in x
    double y = out.pose_y;  // position in y
    double etheta = out.pose_psi;


    geometry_msgs::msg::Pose2D pose;
    nav_msgs::msg::Odometry odom;

    pose.x = x;
    pose.y = y;
    pose.theta = normalize_angle(etheta);

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);

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

    pose_stamped_tmp_.pose.position.x = pose.x;
    pose_stamped_tmp_.pose.position.y = pose.y;
    pose_path.poses.push_back(pose_stamped_tmp_);
    if(pose_path.poses.size() > 5000){
      pose_path.poses.erase(pose_path.poses.begin(), pose_path.poses.begin()+1);
    }

    odomPub->publish(odom);
    localVelPub->publish(velMsg);
    pose_path_pub->publish(pose_path);
    disturbancesPub->publish(disturbance_msg);

    tf_broadcast(pose);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr posePub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr localVelPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr disturbancesPub;
  
  rclcpp::TimerBase::SharedPtr updateTimer;

  geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
    std_msgs::msg::Float64MultiArray disturbance_msg;
    nav_msgs::msg::Path pose_path;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftThrusterSub,
      rightThrusterSub;
  double Tport{0}, Tstbd{0};

  DynamicModel model{0,0,0};
  // DynamicModel model{0,0,M_PI/4};
  // DynamicModel model{11,4,-3.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::string boatname_;

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution;

  void tf_broadcast(const geometry_msgs::msg::Pose2D &msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = boatname_.c_str();;

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