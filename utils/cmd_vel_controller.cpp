//
// Created by Abiel on 11/1/22.
//

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher &pub){
  float leftSide = msg->linear.x + msg->angular.z;
  float rightSide = msg->linear.x - msg->angular.z;
  std_msgs::Float32MultiArray msgOut;
  msgOut.data = std::vector<float>{leftSide, rightSide, 0,0,0,0,0,0};

  pub.publish(msgOut);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "vanttec_cmd_vel_controller");
  ros::NodeHandle n;

  ROS_INFO("controller started!");
  ros::Publisher motor_vel_pub = n.advertise<std_msgs::Float32MultiArray>("motors", 10);
  auto cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(&cmd_velCallback, _1, boost::ref(motor_vel_pub)));
  ros::spin();

  return 0;
}