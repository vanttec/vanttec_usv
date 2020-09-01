/** ----------------------------------------------------------------------------
 * @file: velocity_obstacle.cpp
 * @date: September 1, 2020
 * @author: Ivana Collado
 * @email: a00569475@itesm.mx
 * 
 * @brief: Subscribes to robot and obstacles states and publishes desired 
 * velocity and angle to avoid obatcles using the velocity obstacle model. 
 * ---------------------------------------------------------------------------*/
/** ----------------------------------------------------------------------------
 * @todo: 
 * Waypoint pose2D array msg
 * Perception msgs names uppercase
 * 
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

#include "usv_perception/obstacles_list.h"

// NAMESPACES ------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------
struct Obstacle{
  // Obstacle position in x
  double x;
  // Obstacle position in y
  double y;
  // Obstacle radius
  double r;
  // Right tangent angle
  double tan_r;
  // Left tangent angle
  double tan_l;
};

// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * Input Parameters
  * */
double robot_radius_ = 0.5; //m
/**
  * Size of buffer queue 
  * */
int queue_size_ = 10;
/**
  * Subscribers
  * */
ros::Subscriber vel_sub_;
ros::Subscriber pose_sub_;
ros::Subscriber waypoints_sub_;
ros::Subscriber obstacles_sub_;
/**
  * Publishers
  * */
ros::Publisher desired_vel_pub_;
ros::Publisher desired_heading_pub_;
/**
  * Speed variables
  * */
double speed_x_ = 0.0;
double speed_y_ = 0.0;
double speed_yaw_ = 0.0;
/**
  * Position variables
  * */
double pos_x_ = 0.0;
double pos_y_ = 0.0;
double pos_theta_ = 0.0;
/**
  * Waypoint list vector
  * */
std::vector<double> waypoint_list_;
/**
  * Obstacle list vector
  * */
std::vector<Obstacle> obstacle_list_;

// LAUNCH PARAMETERS ---------------------------------------------------------
/**
  * Topics
  * */
const std::string topic_vel_sub_ = "/vectornav/ins_2d/local_vel";
const std::string topic_pose_sub_ = "/vectornav/ins_2d/NED_pose";
const std::string topic_waypoints_sub_ = "/mission/waypoints";
const std::string topic_obstacles_sub_ = "/usv_perception/lidar_detector/obstacles";
const std::string topic_desired_vel_pub_ = "/guidance/desired_speed";
const std::string topic_desired_heading_pub_ = "/guidance/desired_heading";
/** 
  * Parameters
  * */
const std::string param_queue_size_ = "queue_size";

// FUNCTION DECLARATION --------------------------------------------------------
/**
  * Initialize Model class
  * @return void.
  * */
void initialize(ros::NodeHandle &vo_node);
/**
  * Callback to obtain current velocity. 
  * @param speed[in]: Received velocity vector.
  * @return void.
  * */
void on_speed_msg(const geometry_msgs::Vector3::ConstPtr &speed);
/**
  * Callback to obtain current NED pose. 
  * @param pose[in]: Received velocity vector.
  * @return void.
  * */
void on_pose_msg(const geometry_msgs::Pose2D::ConstPtr &pose);
/**
  * Callback to obtain current NED pose. 
  * @param waypoints[in]: Received velocity vector.
  * @return void.
  * */
void on_waypoints_msg(const std_msgs::Float32MultiArray::ConstPtr &waypoints);
/**
  * Callback to obtain list of obstacles. 
  * @param obstacles[in]: Received velocity vector.
  * @return void.
  * */
void on_obstacles_msg(const usv_perception::obstacles_list::ConstPtr &obstacles);
/**
  * Calculate obstacle collision cone. 
  * @return void.
  * */
void collision_cone();

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_obstacle");
  ros::NodeHandle vo_node("velocity_obstacle");
  ros::Rate loop_rate(100);
  initialize(vo_node);
  while (ros::ok()){
    collision_cone();
    //ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
};

void initialize(ros::NodeHandle &vo_node){
  // Subscribers
  vel_sub_ = vo_node.subscribe(topic_vel_sub_, queue_size_, &on_speed_msg);
  pose_sub_ = vo_node.subscribe(topic_pose_sub_, queue_size_, &on_pose_msg);
  waypoints_sub_ = vo_node.subscribe(topic_waypoints_sub_, queue_size_, &on_waypoints_msg);
  obstacles_sub_ = vo_node.subscribe(topic_obstacles_sub_, queue_size_, &on_obstacles_msg);
  // Publishers
  desired_vel_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desired_vel_pub_, queue_size_);
  desired_heading_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desired_heading_pub_, queue_size_);
  // Success
  ROS_INFO("Velocity broadcaster node is Ready!");
}

void on_speed_msg(const geometry_msgs::Vector3::ConstPtr &msg){
  speed_x_ = msg->x;
  speed_y_ = msg->y;
  speed_yaw_ = msg->z;
}

void on_pose_msg(const geometry_msgs::Pose2D::ConstPtr &msg){
  pos_x_ = msg->x;
  pos_y_ = msg->y;
  pos_theta_ = msg->theta;
}

void on_waypoints_msg(const std_msgs::Float32MultiArray::ConstPtr &msg){
  waypoint_list_.empty();
  int leng = msg->layout.data_offset;
  for (int i = 0; i>leng; ++i){
    waypoint_list_.push_back(msg->data[i]);
  }
}

void on_obstacles_msg(const usv_perception::obstacles_list::ConstPtr &msg){
  obstacle_list_.empty();
  for (int i = 0; i > msg->len; ++i){
    Obstacle obstacle;
    obstacle.x = msg->obstacles[i].x;
    obstacle.y = msg->obstacles[i].y;
    obstacle.r = msg->obstacles[i].z + robot_radius_;
    obstacle_list_.push_back(obstacle);
  }
}

void collision_cone(){
  
}