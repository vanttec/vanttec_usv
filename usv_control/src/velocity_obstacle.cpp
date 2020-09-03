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
struct Coord{ 
  // Position
  double x;
  double y;
};
struct Obstacle{
  // Obstacle position in x
  double x;
  // Obstacle position in y
  double y;
  // Obstacle radius
  double r;
  // Right tangent angle
  Coord tan_r;
  // Left tangent angle
  Coord tan_l;
};
// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * Input Parameters
  * */
double robot_radius_ = 0.5; //meters
double time_horizon_ = 1.0; //seconds
double max_long_acceleration_ = 0.3; //m/s^3
double max_yaw_acceleration_ = 0.1; //rad/s^2
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
Coord tan_r_;
Coord tan_l_;
/**
  * 4 points of reachable velocities cuadrilateral. 
  * */
std::vector<Coord> RV;

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
/**
  * Calculate rechable velocities. 
  * @return void.
  * */
void reachable_velocities();
/**
  * Calculate intersect of two lines taking 4 points as input. 
  * @return 4 Coord points. Point 1 and point 2 belong to line 1. 
  *         Point 3 and point 4 belong to line 2.  
  * */
Coord intersect_two_lines(const Coord& p1, const Coord& p2, const Coord& p3, 
                          const Coord& p4);

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_obstacle");
  ros::NodeHandle vo_node("velocity_obstacle");
  ros::Rate loop_rate(100);
  initialize(vo_node);
  while (ros::ok()){
    collision_cone();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};

void initialize(ros::NodeHandle &vo_node){
  // Subscribers
  vel_sub_ = vo_node.subscribe(topic_vel_sub_, queue_size_, &on_speed_msg);
  pose_sub_ = vo_node.subscribe(topic_pose_sub_, queue_size_, &on_pose_msg);
  waypoints_sub_ = vo_node.subscribe(topic_waypoints_sub_, queue_size_, 
                                    &on_waypoints_msg);
  obstacles_sub_ = vo_node.subscribe(topic_obstacles_sub_, queue_size_, 
                                    &on_obstacles_msg);
  // Publishers
  desired_vel_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desired_vel_pub_, queue_size_);
  desired_heading_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desired_heading_pub_, queue_size_);
  // Success
  ROS_INFO("Velocity obstacle node is Ready!");
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
  for (int i = 0; i < msg->len; ++i){
    Obstacle obstacle;
    obstacle.x = msg->obstacles[i].x;
    obstacle.y = msg->obstacles[i].y;
    obstacle.r = msg->obstacles[i].z + robot_radius_;
    obstacle_list_.push_back(obstacle);
  }
}

void collision_cone(){
  if(0 < obstacle_list_.size()){
    //Find tangent points
    // Boat circle : (y-a)^2 + (x-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (y-c)^2 + (x-d)^2 = r1^2 c=obstacleY d=obstacleX r1=obstacle radius
    // D = distance of circle centers
    int i = 0;
    double a = 0.0;
    double b = 0.0;
    double r0 = sqrt(pow(obstacle_list_[i].x,2)+pow(obstacle_list_[i].y,2));
    double c = obstacle_list_[i].y;
    double d = obstacle_list_[i].x;
    double r1 = obstacle_list_[i].r;
    double D = r0;
    double delta = (1/4)*sqrt((D+r0+r1)*(D+r0-r1)*(D-r0+r1)*(-D+r0+r1));
    obstacle_list_[i].tan_r.x = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) + 2*((a-c)/(D*D))*delta;
    obstacle_list_[i].tan_l.x = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) - 2*((a-c)/(D*D))*delta;
    obstacle_list_[i].tan_r.y = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) + 2*((b-d)/(D*D))*delta;
    obstacle_list_[i].tan_l.y = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) - 2*((b-d)/(D*D))*delta;
    ROS_INFO("Intersection1:%f,%f intersection2:%f,%f", obstacle_list_[i].tan_r.x , obstacle_list_[i].tan_r.y, obstacle_list_[i].tan_l.x , obstacle_list_[i].tan_l.y);
  }
}

void reachable_velocities(){
  // No se considera la velocidad lateral 
  double speed_long = speed_x_; //sqrt(pow(speed_x_,2)+pow(speed_y_,2));
  double v_max = speed_long + max_long_acceleration_*time_horizon_;
  double v_min = speed_long - max_long_acceleration_*time_horizon_;
  double w_max = speed_yaw_ + max_yaw_acceleration_*time_horizon_;
  double w_min = speed_yaw_ - max_yaw_acceleration_*time_horizon_;
  Coord p0 = {v_max, w_min};
  RV.push_back(p0);
  Coord p1 = {v_max, w_max};
  RV.push_back(p1);
  Coord p2 = {v_min, w_max};
  RV.push_back(p2);
  Coord p3 = {v_min, w_min};
  RV.push_back(p3);
}

void reachable_avoidance_velocities(){
  int i = 0;
  //bottom line intersect
  //first line
  Coord p1 = {0,0};
  Coord p2 = obstacle_list_[i].tan_r;
  //second line
  Coord p3 = RV[2];
  Coord p4 = RV[3];
  Coord pbr = intersect_two_lines(p1, p2, p3, p4);
  p2 = obstacle_list_[i].tan_l;
  Coord pbl = intersect_two_lines(p1, p2, p3, p4);
  p3 = RV[0];
  p4 = RV[1];
  Coord ptl = intersect_two_lines(p1, p2, p3, p4);
  p2 = obstacle_list_[i].tan_r;
  Coord ptr = intersect_two_lines(p1, p2, p3, p4);
  p3 = RV[1];
  p4 = RV[2];
  Coord prr = intersect_two_lines(p1, p2, p3, p4);
  p2 = obstacle_list_[i].tan_l;
  Coord prl = intersect_two_lines(p1, p2, p3, p4);
  p3 = RV[0];
  p4 = RV[3];
  Coord pll = intersect_two_lines(p1, p2, p3, p4);
  p2 = obstacle_list_[i].tan_r;
  Coord plr = intersect_two_lines(p1, p2, p3, p4);
}


Coord intersect_two_lines(const Coord& p1, const Coord& p2, const Coord& p3, 
                          const Coord& p4){
  Coord inter;
  inter.x = ((p1.x*p2.y-p1.y*p2.x)*(p3.x-p4.x) - (p1.x - p2.x)*(p3.x*p4.y-p3.y*p4.x))/((p1.x-p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));
  inter.y = ((p1.x*p2.y-p1.y*p2.x)*(p3.y-p4.y) - (p1.y - p2.y)*(p3.x*p4.y-p3.y*p4.x))/((p1.x-p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));
  return inter;
}