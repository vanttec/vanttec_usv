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
 * Update ASMC and dynamic model codes to c++11
 * 
 * Imminent collisions
 * Search using closest to goal heuristic
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <list>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "../include/print_utils.h"
#include "usv_perception/obstacles_list.h"

#include <visualization_msgs/Marker.h>
#define PI 3.14159265
// NAMESPACES ------------------------------------------------------------------
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<Kernel> Polygon_set_2;
typename CGAL::Polygon_2<Kernel>::Vertex_const_iterator vit;
typename CGAL::Polygon_2<Kernel>::Edge_const_iterator eit;
typename CGAL::Polygon_with_holes_2<Kernel>::Hole_const_iterator hit;

// STRUCTS ---------------------------------------------------------------------
struct Coord
{
  // Position
  double x;
  double y;
};
struct Obstacle
{
  // Obstacle position in x
  double x = 0.0;
  // Obstacle position in y
  double y = 0.0;
  // Obstacle radius
  double r = 0.0;
  // Right tangent angle
  Coord tan_r = {0, 0};
  // Left tangent angle
  Coord tan_l = {0, 0};
};
struct Vertex
{
  // Position
  double x = 0;
  double y = 0;
  double goal_dist = INT_MAX;
};

// CLASS DECLARATION -----------------------------------------------------------
// To compare two points
class Comparator
{
public:
  int operator()(const Vertex &v1, const Vertex &v2)
  {
    return v1.goal_dist > v2.goal_dist;
  }
};

// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * Input Parameters
  * */
double robot_radius_ = 0.5;          //meters
double time_horizon_ = 1.0;          //seconds
double max_long_acceleration_ = 0.3; //m/s^3
double max_yaw_acceleration_ = 0.1;  //rad/s^2
double max_vel_ = 1.5;               //m/s
/**
  * Rechable velocities diamond. 
  * */
Polygon_2 RV_;
/**
  * RAV poligon set. 
  * */
Polygon_set_2 RAV_;

Polygon_set_2 CCs_;
/**
  * Size of buffer queue 
  * */
int queue_size_ = 10;
/**
  * Subscribers
  * */
ros::Subscriber vel_sub_;
ros::Subscriber pose_sub_;
ros::Subscriber goal_sub_;
ros::Subscriber dspeed_sub_;
ros::Subscriber dhead_sub_;
// ros::Subscriber waypoints_sub_;
ros::Subscriber obstacles_sub_;
/**
  * Publishers
  * */
ros::Publisher desiered_vel_pub_;
ros::Publisher desiered_heading_pub_;
ros::Publisher marker_pub_;
ros::Publisher collision_pub_;
/**
  * Speed variables
  * */
double los_speed_ = 0.0;
double los_heading_ = 0.0;
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
  * Goal position vector NED
  * */
Eigen::Vector3f goalNED_ = Eigen::Vector3f::Zero();
Eigen::Vector3f goal_body_ = Eigen::Vector3f::Zero();
/**
  * Waypoint list vector
  * */
// std::vector<double> waypoint_list_;
// char waypoint_ref_frame_;
/**
  * Obstacle list vector
  * */
std::vector<Obstacle> obstacle_list_;
/**
  * Cone Marker
  * */
visualization_msgs::Marker marker;
// LAUNCH PARAMETERS ---------------------------------------------------------
/**
  * Topics
  * */
const std::string topic_vel_sub_ = "/vectornav/ins_2d/local_vel";
const std::string topic_pose_sub_ = "/vectornav/ins_2d/NED_pose";
const std::string topic_goal_sub_ = "/usv_control/los/target";
const std::string topic_dspeed_sub_ = "/usv_control/los/desired_speed";
const std::string topic_dhead_sub_ = "/usv_control/los/desired_heading";
// const std::string topic_waypoints_sub_ = "/mission/waypoints";
const std::string topic_obstacles_sub_ = "/usv_perception/lidar_detector/obstacles";
const std::string topic_desiered_vel_pub_ = "/vo/desired_speed";
const std::string topic_desiered_heading_pub_ = "/vo/desired_heading";
const std::string topic_collision_event_ = "/vo/collision_event";
const std::string topic_rviz_cone_ = "/usv_control/cone";
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
void on_dspeed_msg(const std_msgs::Float64::ConstPtr &msg);
void on_dhead_msg(const std_msgs::Float64::ConstPtr &msg);
void on_speed_msg(const geometry_msgs::Vector3::ConstPtr &speed);
/**
  * Callback to obtain current NED pose. 
  * @param pose[in]: Received pose.
  * @return void.
  * */
void on_pose_msg(const geometry_msgs::Pose2D::ConstPtr &pose);
/**
  * Callback to obtain current NED target pose. 
  * @param pose[in]: Recive target pose.
  * @return void.
  * */
void on_goal_msg(const geometry_msgs::Pose2D::ConstPtr &goal);
;
/**
  * Callback to obtain list of obstacles. 
  * @param obstacles[in]: Received velocity vector.
  * @return void.
  * */
void on_obstacles_msg(const usv_perception::obstacles_list::ConstPtr &obstacles);
/**
  * Calculate obstacle collision cone. 
  * @return void.
*/
bool collision_cone();
/**
  * Calculate rechable velocities. 
  * @return void.
  * */
void reachable_velocities();
/**
  * Calculate rechable avoidance velocities. 
  * @return void.
  * */
bool reachable_avoidance_velocities();
/**
  * Calculate rechable avoidance velocities. 
  * @return void.
  * */
void optimal_velocity();
/**
  * Calculate rechable avoidance velocities. 
  * @return void.
  * */
void desiered_velocity(const Vertex &optimal);
/**
  * Rotate position from NED to body. 
  * @return void.
  * */
void NED2body();
/**
  * Draw cone from target to obstacle. 
  * @return void.
  * */
void cone_draw(Polygon_2 C);
/**
 * Check if los desired velocity lies inside the Collision Cone
 * @return bool.
 **/
bool check_vel_collision();
Eigen::Vector3f Body2NED();
// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_obstacle");
  ros::NodeHandle vo_node("velocity_obstacle");
  ros::Rate loop_rate(100);

  std_msgs::Bool collision_flag;
  collision_flag.data = 0;

  initialize(vo_node);
  while (ros::ok())
  {
    if (collision_cone())
    {
      reachable_velocities();
      if (reachable_avoidance_velocities())
      {
        std::cout << "Avoidance maneuvers.\n";
        optimal_velocity();
        collision_flag.data = 1; // Collision
      }
      else
      {
        std::cout << "RV and collision cone do not intersect.\n";
        collision_flag.data = 0; // No collision
      }
      //return 0;
    }
    collision_pub_.publish(collision_flag);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
};

void initialize(ros::NodeHandle &vo_node)
{
  // Subscribers
  vel_sub_ = vo_node.subscribe(topic_vel_sub_, queue_size_, &on_speed_msg);
  pose_sub_ = vo_node.subscribe(topic_pose_sub_, queue_size_, &on_pose_msg);
  goal_sub_ = vo_node.subscribe(topic_goal_sub_, queue_size_, &on_goal_msg);
  dspeed_sub_ = vo_node.subscribe(topic_dspeed_sub_, queue_size_, &on_dspeed_msg);
  dhead_sub_ = vo_node.subscribe(topic_dhead_sub_, queue_size_, &on_dhead_msg);
  // waypoints_sub_ = vo_node.subscribe(topic_waypoints_sub_, queue_size_,
  //  &on_waypoints_msg);
  obstacles_sub_ = vo_node.subscribe(topic_obstacles_sub_, queue_size_,
                                     &on_obstacles_msg);
  // Publishers
  desiered_vel_pub_ = vo_node.advertise<std_msgs::Float64>(
      topic_desiered_vel_pub_, queue_size_);
  desiered_heading_pub_ = vo_node.advertise<std_msgs::Float64>(
      topic_desiered_heading_pub_, queue_size_);
  marker_pub_ = vo_node.advertise<visualization_msgs::Marker>(
      topic_rviz_cone_, queue_size_);
  collision_pub_ = vo_node.advertise<std_msgs::Bool>(
      topic_collision_event_, queue_size_);

  // Success
  ROS_INFO("Velocity obstacle node is Ready!");
}

void on_speed_msg(const geometry_msgs::Vector3::ConstPtr &msg)
{
  speed_x_ = msg->x;
  speed_y_ = msg->y;
  speed_yaw_ = msg->z;
}

void on_pose_msg(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  pos_x_ = msg->x;
  pos_y_ = msg->y;
  pos_theta_ = msg->theta;
}

void on_dspeed_msg(const std_msgs::Float64::ConstPtr &msg)
{
  los_speed_ = msg->data;
}

void on_dhead_msg(const std_msgs::Float64::ConstPtr &msg)
{
  los_heading_ = msg->data;
}

// void on_waypoints_msg(const std_msgs::Float32MultiArray::ConstPtr &msg){
//   waypoint_list_.empty();
//   int leng = msg->layout.data_offset;
//   for (int i = 0; i<leng-2; ++i){
//     waypoint_list_.push_back(msg->data[i]);
//   }
//   waypoint_ref_frame_ = msg->data[leng-1]; // 0 for NED, 1 for GPS, 2 for body
// }

void on_goal_msg(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  goalNED_(0) = msg->x;
  goalNED_(1) = msg->y;
}

void on_obstacles_msg(const usv_perception::obstacles_list::ConstPtr &msg)
{
  obstacle_list_.clear();
  for (int i = 0; i < msg->len; ++i)
  {
    Obstacle obstacle;
    obstacle.x = msg->obstacles[i].x;
    obstacle.y = msg->obstacles[i].y;
    obstacle.r = msg->obstacles[i].z + robot_radius_;
    obstacle_list_.push_back(obstacle);
  }
}

bool collision_cone()
{
  if (0 < obstacle_list_.size())
  {
    //Find tangent points
    // Boat circle : (y-a)^2 + (x-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (y-c)^2 + (x-d)^2 = r1^2 c=obstacleY d=obstacleX r1=obstacle radius
    // D = distance of circle centers
    // Boat circle : (x-a)^2 + (y-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (x-c)^2 + (y-d)^2 = r1^2 c=obstacleX d=obstacleY r1=obstacle radius
    for (int i = 0; i < obstacle_list_.size(); ++i)
    {
      double a = pos_x_;
      double b = pos_y_;
      double c = obstacle_list_[i].x;
      double d = obstacle_list_[i].y;
      double r0 = sqrt(pow(c - a, 2) + pow(d - b, 2));
      double r1 = obstacle_list_[i].r;
      ROS_INFO("Obstacle radius: %f coordinates %f,%f", r1, obstacle_list_[i].x, obstacle_list_[i].y);
      double D = r0;
      double delta = 0.25 * sqrt((D + r0 + r1) * (D + r0 - r1) * (D - r0 + r1) * (-D + r0 + r1));
      ROS_INFO("Obstacle delta: %f", delta);
      obstacle_list_[i].tan_r.y = (b + d) / 2 + ((d - b) * (r0 * r0 - r1 * r1)) / (2 * D * D) - 2 * ((a - c) / (D * D)) * delta;
      ROS_INFO("Obstacle y1 %f", obstacle_list_[i].tan_r.y);
      obstacle_list_[i].tan_l.y = (b + d) / 2 + ((d - b) * (r0 * r0 - r1 * r1)) / (2 * D * D) + 2 * ((a - c) / (D * D)) * delta;
      ROS_INFO("Obstacle y2 %f", obstacle_list_[i].tan_l.y);
      obstacle_list_[i].tan_r.x = (a + c) / 2 + ((c - a) * (r0 * r0 - r1 * r1)) / (2 * D * D) + 2 * ((b - d) / (D * D)) * delta;
      ROS_INFO("Obstacle x1 %f", obstacle_list_[i].tan_r.x);
      obstacle_list_[i].tan_l.x = (a + c) / 2 + ((c - a) * (r0 * r0 - r1 * r1)) / (2 * D * D) - 2 * ((b - d) / (D * D)) * delta;
      ROS_INFO("Obstacle x2 %f", obstacle_list_[i].tan_l.x);
      ROS_INFO("Obstacle %i intersection1:%f,%f intersection2:%f,%f", i, obstacle_list_[i].tan_r.x, obstacle_list_[i].tan_r.y, obstacle_list_[i].tan_l.x, obstacle_list_[i].tan_l.y);
    }
    return 1;
  }
  return 0;
}

void reachable_velocities()
{
  // No se considera la velocidad lateral
  double speed_long = speed_x_; //sqrt(pow(speed_x_,2)+pow(speed_y_,2));
  double v_max = speed_long + max_long_acceleration_ * time_horizon_;
  v_max = (max_vel_ < v_max) ? v_max : max_vel_;
  double v_min = speed_long - max_long_acceleration_ * time_horizon_;
  double w_max = speed_yaw_ + max_yaw_acceleration_ * time_horizon_;
  double w_min = speed_yaw_ - max_yaw_acceleration_ * time_horizon_;
  RV_.clear();
  RV_.push_back(Point_2(v_max, 0));
  RV_.push_back(Point_2(speed_long, w_max));
  RV_.push_back(Point_2(v_min, 0));
  RV_.push_back(Point_2(speed_long, w_min));
  // std::cout << "RV = "; print_polygon (RV_);
}

void cone_draw(Polygon_2 C)
{
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cone_shape";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pos_x_;
  marker.pose.position.y = pos_y_;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.lifetime = ros::Duration();
  geometry_msgs::Point p;
  marker.points.clear();
  for (Polygon_2::Vertex_const_iterator vertex = C.vertices_begin(); vertex != C.vertices_end(); ++vertex)
  {
    p.x = to_double(vertex->hx());
    p.y = to_double(vertex->hy());
    p.z = 0;
    marker.points.push_back(p);
  }
  p.x = to_double(C.vertices_begin()->hx());
  p.y = to_double(C.vertices_begin()->hy());
  p.z = 0;
  marker.points.push_back(p);
  marker_pub_.publish(marker);
}

bool reachable_avoidance_velocities()
{
  Polygon_2 C;
  Polygon_with_holes_2 C_union;
  Coord intersect_l;
  Coord intersect_r;
  Coord vel_th;
  double obstacle_theta = 0.0;
  double speed_th = 0.0;
  double b = 0.0;
  double numerator = 0.0;
  double denominator = 0.0;
  double obs_tang_dist = 0.0;
  double obs_center_dist = 0.0;
  Coord p1;
  Coord p2;
  Coord p3;
  Coord p4;
  Coord usv;
  Coord obstacle;
  double obs_radius = 0.0;
  double off_angle = 0.0;
  double ctr_angle = 0.0;

  double angle = 0.0;
  double ctr_slope = 0.0, upr_slope = 0.0, lwr_slope = 0.0;
  double constant = 0.0, c_up = 0.0, c_down = 0.0;

  CCs_.clear();
  RAV_.clear();

  // Convencion usada por vanttec: x-frente, y-izq (para este metodo)
  for (int i = 0; i < obstacle_list_.size(); ++i)
  {
    //https://www.desmos.com/calculator/keo5d2zdoe
    usv.x = pos_x_;
    usv.y = -pos_y_;
    obstacle.x = obstacle_list_[i].x;
    obstacle.y = -obstacle_list_[i].y;
    obs_radius = obstacle_list_[i].r;
    //distance between obstacle center and usv center

    ctr_slope = (usv.x - obstacle.x) / (usv.y - obstacle.y);
    obs_center_dist = sqrt(pow(obstacle.x - usv.x, 2) + pow(obstacle.y - usv.y, 2));
    obs_tang_dist = sqrt(pow(obs_center_dist, 2) - pow(obs_radius, 2));
    off_angle = asin(obs_radius / obs_center_dist);
    ctr_angle = atan(ctr_slope);

    p1 = usv;
    if (usv.y < obstacle.y)
    {
      p2.x = -(usv.y + cos(ctr_angle + off_angle) * obs_tang_dist);
      p2.y = (usv.x + sin(ctr_angle + off_angle) * obs_tang_dist);
      p3.x = -(usv.y + cos(ctr_angle - off_angle) * obs_tang_dist);
      p3.y = (usv.x + sin(ctr_angle - off_angle) * obs_tang_dist);
    }
    else
    {
      p2.y = -(usv.y - cos(ctr_angle + off_angle) * obs_tang_dist);
      p2.x = (usv.x - sin(ctr_angle + off_angle) * obs_tang_dist);
      p3.y = -(usv.y - cos(ctr_angle - off_angle) * obs_tang_dist);
      p3.x = (usv.x - sin(ctr_angle - off_angle) * obs_tang_dist);
    }
    std::cout << "usv.x " << usv.x << " usv.y " << usv.y << " obstacle.x " << obstacle.x << " obstacle.y " << obstacle.y << std::endl;
    std::cout << "p2 x " << p2.x << " y " << p2.y << " p3 x " << p3.x << " y " << p3.y << std::endl;
    std::cout << "ctr_angle " << ctr_angle << " off_angle " << off_angle << std::endl;

    /* // Calculate VOH
    obs_dist = sqrt(pow(obstacle_list_[i].x-pos_x_,2)+pow(obstacle_list_[i].y-pos_y_,2));// - obstacle_list_[i].r;
    speed_th = (obs_dist - obstacle_list_[i].r) / time_horizon_;
    obstacle_theta = atan2(obstacle_list_[i].x, obstacle_list_[i].y); // Para atan2 derecha es x y y es frente
    // Next> with origin in boat
    vel_th.x = speed_th * sin(obstacle_theta);
    vel_th.y = speed_th * cos(obstacle_theta);
    slope = (vel_th.x - pos_x_) / (vel_th.y - pos_y_); 
    // b = vel_th.x - (slope * vel_th.y);    // (b,0) is a point
    
    slope = -1/slope;
    b = vel_th.x - (slope * vel_th.y);    // (b,0) is a point

    // Intersect tan_l
    p1.x=pos_x_;
    p1.y=pos_y_;
    p2.x=obstacle_list_[i].tan_l.x;
    p2.y=obstacle_list_[i].tan_l.y;
    p3.x=vel_th.x;
    p3.x=vel_th.y;
    p4.x=slope*obstacle_list_[i].tan_l.y+b;
    p4.y=obstacle_list_[i].tan_l.y;
    // numerator = ((p1.y*p2.x-p1.x*p2.y)*(p3.y-p4.y))-((p1.y-p2.y)*(p3.y*p4.x-p3.x*p4.y));
    // denominator = ((p1.y-p2.y)*(p3.x-p4.x))-((p1.x-p2.x)*(p3.y-p4.y));
    numerator = (((obstacle_list_[i].tan_l.x*pos_y_)-(obstacle_list_[i].tan_l.y*pos_x_))*(vel_th.x-b)) - ((obstacle_list_[i].tan_l.x-pos_x_)*((vel_th.x*0)-(vel_th.y*b)));
    denominator = ((obstacle_list_[i].tan_l.x-pos_x_)*(vel_th.y-0)) - ((obstacle_list_[i].tan_l.y-pos_y_)*(vel_th.x-b)); 
    intersect_l.x =  numerator / denominator;
    // numerator = ((p1.y*p2.x-p1.x*p2.y)*(p3.y-p4.y))-((p1.y-p2.y)*(p3.y*p4.x-p3.x*p4.y));
    numerator = (((obstacle_list_[i].tan_l.x*pos_y_)-(obstacle_list_[i].tan_l.y*pos_x_))*(vel_th.y-0)) - ((obstacle_list_[i].tan_l.y-pos_y_)*((vel_th.x*0)-(vel_th.y*b)));
    intersect_l.y =  numerator / denominator;
    // Intersect tan_r
    p2.x=obstacle_list_[i].tan_r.x;
    p2.y=obstacle_list_[i].tan_r.y;
    p4.x=slope*obstacle_list_[i].tan_r.y+b;
    p4.y=obstacle_list_[i].tan_r.y;
    // numerator = ((p1.y*p2.x-p1.x*p2.y)*(p3.y-p4.y))-((p1.y-p2.y)*(p3.y*p4.x-p3.x*p4.y));
    // denominator = ((p1.y-p2.y)*(p3.x-p4.x))-((p1.x-p2.x)*(p3.y-p4.y));
    numerator = (((obstacle_list_[i].tan_r.x*pos_y_)-(obstacle_list_[i].tan_r.y*pos_x_))*(vel_th.x-b)) - ((obstacle_list_[i].tan_r.x-pos_x_)*((vel_th.x*0)-(vel_th.y*b)));
    denominator = ((obstacle_list_[i].tan_r.x-pos_x_)*(vel_th.y-0)) - ((obstacle_list_[i].tan_r.y-pos_y_)*(vel_th.x-b)); 
    intersect_r.x =  numerator / denominator;
    // numerator = ((p1.y*p2.x-p1.x*p2.y)*(p3.y-p4.y))-((p1.y-p2.y)*(p3.y*p4.x-p3.x*p4.y));
    numerator = (((obstacle_list_[i].tan_r.x*pos_y_)-(obstacle_list_[i].tan_r.y*pos_x_))*(vel_th.y-0)) - ((obstacle_list_[i].tan_r.y-pos_y_)*((vel_th.x*0)-(vel_th.y*b)));
    intersect_r.y =  numerator / denominator; */

    // Construct the input cone
    C.clear();
    C.push_back(Point_2(p1.x, p1.y)); // From VOH
    C.push_back(Point_2(p2.x, p2.y)); // Limit of input cone
    C.push_back(Point_2(p3.x, p3.y)); // Limit of input cone
    // Draw cone
    cone_draw(C);
    /* std::cout << "C = "; print_polygon (C);
    
    // Check to see if cone intersercts with RV diamond
    if ((CGAL::do_intersect (C, RV_))){
      std::cout << "The two polygons intersect." << std::endl;
      if(C_union.is_unbounded()){
        std::cout << "Unbounded" << std::endl;
        Polygon_with_holes_2 temp(C); // insert first cone
        C_union = temp;
      }
      else{
        //Join cone with other cones
        CGAL::join (C, C_union, C_union);
        std::cout << "Joined." << std::endl;
      }
      print_polygon_with_holes (C_union);
    }
    else{
      CCs_.join(C);
      std::cout << "The two polygons do not intersect." << std::endl;
    } */
  }
  if (!C_union.is_unbounded())
  {
    // Perform a sequence of operations.
    RAV_.join(C_union);
    RAV_.complement();      // Compute the complement.
    RAV_.intersection(RV_); // Intersect with the clipping rectangle.
    CCs_.join(C_union);
    // Print the result.
    std::cout << "The result contains " << RAV_.number_of_polygons_with_holes()
              << " components:" << std::endl;
    // Get vertices de C_union y llamar a check inside con velocidad deseada
    if (0 < RAV_.number_of_polygons_with_holes())
    {
      return 1;
    }
  }
  // RAV_.join(RV_);
  return 0;
}

void optimal_velocity()
{
  NED2body();
  if (!(0 == goal_body_(0) && 0 == goal_body_(1)))
  {
    Pwh_list_2 res;
    Pwh_list_2::const_iterator it;
    Polygon_with_holes_2 temp;
    Polygon_2 temp_poly;
    Point_2 temp_point;
    // Creates a Min heap of points (order by goal_dist)
    std::priority_queue<Vertex, std::vector<Vertex>, Comparator> queue;
    //Iterate over set of polygon with holes
    // std::cout << "The result contains " << RAV_.number_of_polygons_with_holes()
    // << " components:" << std::endl;
    RAV_.polygons_with_holes(std::back_inserter(res));
    for (it = res.begin(); it != res.end(); ++it)
    {
      // std::cout << "--> ";
      temp = *it;
      // Print polygon outer boundary
      // std::cout << "{ Outer boundary = ";
      // std::cout << "[ " << temp.outer_boundary().size() << " vertices:";
      for (vit = temp.outer_boundary().vertices_begin(); vit != temp.outer_boundary().vertices_end(); ++vit)
      {
        // std::cout << " (" << *vit << ')';
        temp_point = *vit;
        // std::cout << " (" << temp_point.x() << ',' << temp_point.y()<< ')';
        Vertex temp_vertex;
        temp_vertex.x = CGAL::to_double(temp_point.x());
        temp_vertex.y = CGAL::to_double(temp_point.y());
        temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goal_body_(0), 2) + pow(temp_vertex.y - goal_body_(1), 2));
        queue.push(temp_vertex);
      }
      // std::cout << " ]" << std::endl;

      // std::cout << "  " << temp.number_of_holes() << " holes:" << std::endl;
      unsigned int k = 1;
      for (hit = temp.holes_begin(); hit != temp.holes_end(); ++hit, ++k)
      {
        // std::cout << "    Hole #" << k << " = ";
        print_polygon(*hit);
        temp_poly = *hit;
        // std::cout << "[ " << temp_poly.size() << " vertices:";
        for (vit = temp_poly.vertices_begin(); vit != temp_poly.vertices_end(); ++vit)
        {
          // std::cout << " (" << *vit << ')';
          temp_point = *vit;
          // std::cout << " (" << temp_point.x() << ',' << temp_point.y()<< ')';
          Vertex temp_vertex;
          temp_vertex.x = CGAL::to_double(temp_point.x());
          temp_vertex.y = CGAL::to_double(temp_point.y());
          temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goal_body_(0), 2) + pow(temp_vertex.y - goal_body_(1), 2));
          queue.push(temp_vertex);
        }
        // std::cout << " ]" << std::endl;
      }
      // std::cout << " }" << std::endl;

      // std::cout << " queue size: " << queue.size() << std::endl;
      // std::cout << " closest vertex: " << queue.top().x << ',' << queue.top().y << std::endl;
      desiered_velocity(queue.top());
    }
  }
}

void desiered_velocity(const Vertex &optimal)
{
  std_msgs::Float64 desiered_heading;
  std_msgs::Float64 desiered_speed;
  desiered_heading.data = atan2(optimal.y, optimal.x) + pos_theta_;
  desiered_speed.data = sqrt(pow(optimal.x, 2) + pow(optimal.y, 2));
  desiered_heading_pub_.publish(desiered_heading);
  desiered_vel_pub_.publish(desiered_speed);
}

void NED2body()
{
  Eigen::Matrix3f R;
  R << cos(pos_theta_), -sin(pos_theta_), 0,
      sin(pos_theta_), cos(pos_theta_), 0,
      0, 0, 1;
  goal_body_ = R * goalNED_;
}

Eigen::Vector3f Body2NED()
{
  double x = los_speed_ * sin(los_heading_);
  double y = los_speed_ * cos(los_heading_);
  Eigen::Vector3f ptBody(x, y, 0);
  Eigen::Vector3f ptNED;
  Eigen::Matrix3f R;
  R << cos(pos_theta_), -sin(pos_theta_), 0,
      sin(pos_theta_), cos(pos_theta_), 0,
      0, 0, 1;
  ptNED = R.inverse() * ptBody;
  return ptNED;
}

// bool check_vel_collision(){
//   std_msgs::Bool collision_flag;
//   collision_flag.data=0;
//   Pwh_list_2 res;
//   Pwh_list_2::const_iterator it;
//   // Polygon_2::Edge_const  _interator eit;
//   CCs_.polygons_with_holes (std::back_inserter (res));
//   Eigen::Vector3f pt = Body2NED();
//   Kernel::Segment_2 d_velocity(Point_2(pos_y_,pos_x_),Point_2(pt(1),pt(0)));
//   std::cout<<"Vel: "<<d_velocity.source()<<" "<<d_velocity.target()<<"\n";
//   // Polygon_2 d_velocity;
//   // d_velocity.push_back(Point_2(pos_x_,pos_y_));
//   // d_velocity.push_back(Point_2(pos_x_+0.1,pos_y_));
//   // d_velocity.push_back(Point_2(pt(0)+0.1,pt(1)));
//   // d_velocity.push_back(Point_2(pt(0),pt(1)));

//   for (it = res.begin(); it != res.end(); ++it) {
//     std::cout<<"Vel: "<<d_velocity.source()<<" "<<d_velocity.target()<<"\n";

//     // std::cout << "--> ";
//     // print_polygon_with_holes (*it);
//     // std::cout << "Pt vel: " << pt << "\n";

//     // std::cout << CGAL::is_simple_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Kernel()) << "\n";

//     // CGAL uses same coordinate system as RVIZ? front - x left - y
//     // NEgar y cuando se trabaje con rviz
//     // if (CGAL::bounded_side_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Point_2(pt(0),pt(1)), Kernel())
//     //     == CGAL::ON_BOUNDED_SIDE){
//     for (eit = (*it).outer_boundary().edges_begin(); eit != (*it).outer_boundary().edges_end(); ++eit){
//       if (CGAL::do_intersect(d_velocity, *eit)){
//         std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//         collision_flag.data=1;
//         collision_pub_.publish(collision_flag);
//         return 1;
//       } else {
//       std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
//         collision_pub_.publish(collision_flag);
//       }
//     }
//      // if(CGAL::do_intersect((*it).outer_boundary(),d_velocity)){
//       //   std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//       //   collision_flag.data=1;
//       //   collision_pub_.publish(collision_flag);
//       //   return 1;
//       // } else {
//       //   std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
//       //   collision_pub_.publish(collision_flag);
//       // }
//   }
//   return 0;

//   //   switch(CGAL::bounded_side_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Point_2(pt(0),pt(1)), Kernel())) {
//   //     case CGAL::ON_BOUNDED_SIDE:
//   //       std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//   //       collision_pub_.publish(collision_flag);
//   //       return 1;
//   //       break;
//   //     case CGAL::ON_BOUNDARY:
//   //       std::cout << "LOS desired vel and hdng is on the collision cones boundary.\n";
//   //       break;
//   //     case CGAL::ON_UNBOUNDED_SIDE:
//   //       std::cout << "LOS desired vel and hdng is outside the collision cones.\n";
//   //       break;
//   //   }
//   // }
//   // return 0;
// }
