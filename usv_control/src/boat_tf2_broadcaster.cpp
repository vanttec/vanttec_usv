#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"

class TfAndPath{
public:
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  TfAndPath(){
  path_pub = node.advertise<nav_msgs::Path>("/usv_control/broadcaster/usv_path", 1000);
  pose_sub = node.subscribe("/vectornav/ins_2d/NED_pose", 1000, &TfAndPath::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "boat";
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = -msg->y;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(3.1415926535, 0, -msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = msg->x;
    pose.pose.position.y = -msg->y;
    pose.pose.position.z = 0;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.push_back(pose);
    
    path_pub.publish(path);
  }

private:
  ros::NodeHandle node;
  ros::Publisher path_pub;
  ros::Subscriber pose_sub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "boat_tf2_broadcaster");
  TfAndPath tfAndPath;

  while(true){
    ros::Rate(100).sleep();
    ros::spinOnce();
  }

  return 0;
}
