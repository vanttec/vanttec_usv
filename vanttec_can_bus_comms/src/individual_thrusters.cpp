#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <sstream>
#include <vector>

float left_thrust = 0;
float right_thrust = 0;

void leftThrustCallback(const std_msgs::Float32::ConstPtr& left, ros::Publisher &pub)
{
  std_msgs::Float32MultiArray msg;
  left_thrust = left->data;

  if(left_thrust > 36.5){
    left_thrust = 1;
  } else if(left_thrust < -30){
    left_thrust = -1;
  } else if(left_thrust > 0){
    left_thrust = left_thrust / 36.5;
  } else{
    left_thrust = left_thrust / 30;
  }

  msg.data = std::vector<float>{left_thrust, right_thrust, 0,0,0,0,0,0};
  pub.publish(msg);
}

void rightThrustCallback(const std_msgs::Float32::ConstPtr& right, ros::Publisher &pub)
{
  std_msgs::Float32MultiArray msg;
  right_thrust = right->data;

  if(right_thrust > 36.5)
    right_thrust = 1;
  else if(right_thrust < -30)
    right_thrust = -1;
  else if(right_thrust > 0)
    right_thrust/=36.5;
  else
    right_thrust/=30;

  msg.data = std::vector<float>{left_thrust, right_thrust, 0,0,0,0,0,0};
  pub.publish(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "individual_thrusters");
  ros::NodeHandle n;
  ros::Publisher motors_pub = n.advertise<std_msgs::Float32MultiArray>("/motors", 1000);
  auto left_sub = n.subscribe<std_msgs::Float32>("/motors/left_thrust", 1000, boost::bind(&leftThrustCallback, _1, boost::ref(motors_pub)));
  auto right_sub = n.subscribe<std_msgs::Float32>("/motors/right_thrust", 1000, boost::bind(&rightThrustCallback, _1, boost::ref(motors_pub)));
  ros::spin();
  return 0;
}