#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "nav_msgs/Odometry.h"

void odomCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "boat";
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = -msg->y;
  /*transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = 0.0;*/

  tf2::Quaternion q;
  q.setRPY(3.1415926535, 0, -msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  /*transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;*/

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "boat_tf2_broadcaster");
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/vectornav/ins_2d/NED_pose", 1000, &odomCallback);
  //ros::Subscriber sub = node.subscribe("/usv_control/dynamic_model_simulate/odom", 1000, &odomCallback);

  ros::spin();
  return 0;
}
