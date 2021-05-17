/** ----------------------------------------------------------------------------
 * @file: velocity_obstacle.cpp
 * @date: September 1, 2020
 * @author: Ivana Collado
 * @email: a00569475@itesm.mx
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: Subscribes to robot, obstacles states and los desired speed and heading,
 * and publishes desired velocity and angle to avoid obatcles using the velocity
 * obstacle model.
 * ---------------------------------------------------------------------------*/
/** ----------------------------------------------------------------------------
 * @todo:
 * Tests with multiple obstacles
 * Refactor of the whole node
 * Check RAV generation. Specially for max angular acceleration (could it be larger?)
 * Check time horizon when < 0.7 sec -- error cuando se genera RV
 * Check max accel min limit. If == 1 RV has self-intersecting edges
 * Check that rviz marker generation is correct (for RV set)
 * Solution applied when obstacle is on/inside the circumference of obstacle the
 * best one?
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/Marker.h>
#define PI 3.14159265
// NAMESPACES ------------------------------------------------------------------
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef CGAL::Polygon_set_2<Kernel> Polygon_set_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typename CGAL::Polygon_2<Kernel>::Vertex_const_iterator vit;
typename CGAL::Polygon_2<Kernel>::Edge_const_iterator eit;
typename CGAL::Polygon_with_holes_2<Kernel>::Hole_const_iterator hit;

// enum Cone_State_ {SAFE,        // Safe: when Vth coord > Obstacle pos coord  (Here the polygon disappears)
//                   COL_RISK,    // Risk of collision: when Boat pos coord <= Vth coord <= Obstacle pos
//                   IMM_COL,     // Imminent collision: when Vth cood < Boat pos coord (Here the polygon becomes a cone)
//                   COL};        // The boat entered the obstacle circumference. Vth <= 0

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
  // Right cut angle
  Coord voh_r = {0, 0};
  // Left cut angle
  Coord voh_l = {0, 0};
  // Cone state for the obstacle
  bool col_state = 0;
  // Distance to the boat
  double boat_distance = 0;
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
class Closest
{
public:
  int operator()(const Vertex &v1, const Vertex &v2)
  {
    return v1.goal_dist > v2.goal_dist;
  }
};

class Farthest // Se puede usar el Closest junto con el [ultimo] elemento del 
               // queue en vez de crear otra
{
public:
  int operator()(const Vertex &v1, const Vertex &v2)
  {
    return v1.goal_dist < v2.goal_dist;
  }
};

// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * Input Parameters
  * */
double robot_radius_ = 0.4;          //meters
double time_horizon_ = 1;            //seconds
double col_time_horizon_ = 7;        //seconds
double max_long_acceleration_ = 0.3; //m/s^3
double max_yaw_acceleration_ = 0.1;  //rad/s^2
double max_vel_ = 1.5;               //m/s
Vertex ftst_chosen_vel;
/**
  * Distance to the closest obstacle
  * */
double min_distance = DBL_MAX;
/**
  * Idex of closest obstacle
  * */
int closest_obst = 0;
/**
  * Imminent collision flag
  * */
bool imminent_collision = 0;
/**
  * Rechable velocities diamond.
  * */
Polygon_2 RV_;
/**
  * RAV polygon set.
  * */
Polygon_set_2 RAV_;
/**
  * Collision cone polygon set.
  * */
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
ros::Publisher desired_vel_pub_;
ros::Publisher desired_heading_pub_;
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
double goal_dist_ = 10.0;
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
const std::string topic_desired_vel_pub_ = "/vo/desired_speed";
const std::string topic_desired_heading_pub_ = "/vo/desired_heading";
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
void on_speed_msg(const geometry_msgs::Vector3::ConstPtr &speed);
/**
  * Callback to obtain los desired speed.
  * @param dspeed[in]: Received speed.
  * @return void.
  * */
void on_dspeed_msg(const std_msgs::Float64::ConstPtr &dspeed);
/**
  * Callback to obtain los desired heading.
  * @param dheading[in]: Received heading.
  * @return void.
  * */
void on_dhead_msg(const std_msgs::Float64::ConstPtr &dheading);
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
void desired_velocity(const Vertex &optimal);
/**
  * Rotate position from NED to body.
  * @return void.
  * */
void NED2body();
/**
  * Draw cone from boat to obstacle.
  * @param C[in]: Polygon.
  * @param p_ref[in]: Reference point.
  * @param ns[in]: Name of marker.
  * @return void.
  * */
void cone_draw(Polygon_2 C, Coord p_ref, double yaw, std::string ns, int i);
/**
  * Draw Vth vector.
  * @param p_begin[in]: Initial point.
  * @param p_end[in]: Final point.
  * @param p_ref[in]: Reference point.
  * @param ns[in]: Name of marker.
  * @param i marker id
  * @return void.
  * */
void line_draw(Coord p_begin, Coord p_end, Coord p_ref, std::string ns);
/**
  * Draw circles around bodies.
  * @param h[in]: X reference coordinate.
  * @param k[in]: Y reference coordinate.
  * @param r[in]: Body radius.
  * @param ns[in]: Name of marker.
  * @param i marker id
  * @return void.
  * */
void circle_draw(double h, double k, double r, std::string ns, int i);
/**
 * Check if los desired velocity lies inside the Collision Cone
 * @return bool.
 **/
bool check_vel_collision(Coord a1, Coord a2, Coord b1, Coord b2, Coord c1);
bool check_point_inside(Polygon_2 pol, Point_2 pt);
bool check_poly_inside(Polygon_2 pol);
void check_cone_state(Coord, Obstacle &, double);
bool check_point_between(Vertex);
Coord NED2body_(Coord &coord);
Coord Body2NED_(Coord &coord);
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
  while (ros::ok() && (goal_dist_ > 0.8))
  {
    //marker.points.clear();
    if (collision_cone())
    {
      reachable_velocities();
      if (reachable_avoidance_velocities())
      { //or check_vel_collision()){
        // ROS_INFO("Avoidance maneuvers");
        optimal_velocity();
        collision_flag.data = 1; // Collision
      }
      else
      {
        // std::cout << "RV and collision cone do not intersect.\n";
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
  obstacles_sub_ = vo_node.subscribe(topic_obstacles_sub_, queue_size_,
                                     &on_obstacles_msg);
  // Publishers
  desired_vel_pub_ = vo_node.advertise<std_msgs::Float64>(
      topic_desired_vel_pub_, queue_size_);
  desired_heading_pub_ = vo_node.advertise<std_msgs::Float64>(
      topic_desired_heading_pub_, queue_size_);
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

void on_goal_msg(const geometry_msgs::Pose2D::ConstPtr &msg)
{
  goalNED_(0) = msg->x;
  goalNED_(1) = msg->y;
  goal_dist_ = sqrt(pow(goalNED_(0) - pos_x_, 2) + pow(goalNED_(1) - pos_y_, 2));
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

// void check_cone_state(Coord vel_th, Obstacle &obs, double obs_circum_dist){
//   if((vel_th.x > obs.x) && (vel_th.y > obs.y)){
//     obs.Col_State_ = SAFE;
//   } else {
//     if((pos_x_< vel_th.x) && (pos_y_< vel_th.y) && (vel_th.x <= obs.x) && (vel_th.y <= obs.y)){
//       obs.Col_State_ = COL_RISK;
//     } else {
//       if((vel_th.x <= pos_x_) && (vel_th.y <= pos_y_)){
//         obs.Col_State_ = IMM_COL;
//       } else {
//         double vth = sqrt(pow(vel_th.x,2)+pow(vel_th.y,2));
//         if(vth < obs_circum_dist){
//           obs.Col_State_ = COL;
//         }
//       }
//     }
//   }
// }

bool collision_cone()
{
  if (0 < obstacle_list_.size())
  {
    // Find tangent points
    // http://paulbourke.net/geometry/circlesphere/
    // Boat circle : (y-a)^2 + (x-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (y-c)^2 + (x-d)^2 = r1^2 c=obstacleY d=obstacleX r1=obstacle radius
    // D = distance of circle centers
    // Boat circle : (x-a)^2 + (y-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (x-c)^2 + (y-d)^2 = r1^2 c=obstacleX d=obstacleY r1=obstacle radius
    Coord p2;
    double a;
    double h;
    double d;
    double r0;
    double r1;
    for (int i = 0; i < obstacle_list_.size(); ++i)
    {
      // double a = pos_x_;
      // double b = pos_y_;
      // double c = obstacle_list_[i].x;
      // double d = obstacle_list_[i].y;
      // double r0 = sqrt(pow(c-a,2)+pow(d-b,2));
      // double r1 = obstacle_list_[i].r;

      r0 = sqrt(pow(obstacle_list_[i].x - pos_x_, 2) + pow(obstacle_list_[i].y - pos_y_, 2));
      r1 = obstacle_list_[i].r;
      d = r0;
      a = (d * d - r1 * r1 + r0 * r0) / (2 * d);
      h = sqrt(r0 * r0 - a * a);
      p2.x = pos_x_ + (a / d) * (obstacle_list_[i].x - pos_x_);
      p2.y = pos_y_ + (a / d) * (obstacle_list_[i].y - pos_y_);
      // ROS_INFO("Obstacle radius: %f coordinates %f,%f, distance: %f", r1, obstacle_list_[i].x, obstacle_list_[i].y, r0);
      // // ROS_INFO("Boat pos: %f, %f",a,b);
      // // double D = r0;
      // // double delta = 0.25*sqrt((D+r0+r1)*(D+r0-r1)*(D-r0+r1)*(-D+r0+r1));
      // // ROS_INFO("Obstacle delta: %f", delta);
      obstacle_list_[i].tan_l.x = p2.x + (h / d) * (obstacle_list_[i].y - pos_y_);
      obstacle_list_[i].tan_l.y = p2.y - (h / d) * (obstacle_list_[i].x - pos_x_);
      obstacle_list_[i].tan_r.x = p2.x - (h / d) * (obstacle_list_[i].y - pos_y_);
      obstacle_list_[i].tan_r.y = p2.y + (h / d) * (obstacle_list_[i].x - pos_x_);
      marker.points.clear();
      // circle_draw(pos_x_, pos_y_, r0, "boat_circle", i); 
      circle_draw(obstacle_list_[i].x, obstacle_list_[i].y, r1, "obstacle_circle", i);
      // ROS_INFO("Obstacle origin: %f, %f",obstacle_list_[i].x,obstacle_list_[i].y);
      // // obstacle_list_[i].tan_r.y = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) - 2*((a-c)/(D*D))*delta;
      // ROS_INFO("Obstacle x1 %f", obstacle_list_[i].tan_r.x);
      // ROS_INFO("Obstacle y1 %f", obstacle_list_[i].tan_r.y);
      // // obstacle_list_[i].tan_l.y = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) + 2*((a-c)/(D*D))*delta;
      // ROS_INFO("Obstacle x2 %f", obstacle_list_[i].tan_l.x);
      // ROS_INFO("Obstacle y2 %f", obstacle_list_[i].tan_l.y);
      // // obstacle_list_[i].tan_r.x = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) + 2*((b-d)/(D*D))*delta;
      // // obstacle_list_[i].tan_l.x = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) - 2*((b-d)/(D*D))*delta;
      // ROS_INFO("Obstacle %i intersection1: %f,%f intersection2: %f,%f", i, obstacle_list_[i].tan_r.x , obstacle_list_[i].tan_r.y, obstacle_list_[i].tan_l.x , obstacle_list_[i].tan_l.y);
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
  v_max = (max_vel_ > v_max) ? v_max : max_vel_;
  double v_min = speed_long - max_long_acceleration_ * time_horizon_;
  double w_max = speed_yaw_ + max_yaw_acceleration_ * time_horizon_;
  double w_min = speed_yaw_ - max_yaw_acceleration_ * time_horizon_;
  RV_.clear();
  Coord v_mx;
  Coord v_mn;
  Coord w_mx;
  Coord w_mn;
  v_mx.x = v_max;
  v_mx.y = 0;
  w_mx.x = speed_long;
  w_mx.y = w_max;
  v_mn.x = v_min;
  v_mn.y = 0;
  w_mn.x = speed_long;
  w_mn.y = w_min;
  v_mx = Body2NED_(v_mx);
  w_mx = Body2NED_(w_mx);
  v_mn = Body2NED_(v_mn);
  w_mn = Body2NED_(w_mn);
  // std::cout << "vmax = " << v_mx.x << " " <<  v_mx.y << "\n";
  // std::cout << "vmin = " << v_mn.x << " " <<  v_mn.y << "\n";
  // std::cout << "wmax = " << w_mx.x << " " <<  w_mx.y << "\n";
  // std::cout << "wmin = " << w_mn.x << " " <<  w_mn.y << "\n";
  // RV_.push_back (Point_2 (v_max, 0));
  // RV_.push_back (Point_2 (speed_long, w_max));
  // RV_.push_back (Point_2 (v_min, 0));
  // RV_.push_back (Point_2 (speed_long, w_min));
  RV_.push_back(Point_2(v_mx.x, v_mx.y));
  RV_.push_back(Point_2(w_mx.x, w_mx.y));
  RV_.push_back(Point_2(v_mn.x, v_mn.y));
  RV_.push_back(Point_2(w_mn.x, w_mn.y));

  Coord p_ref;
  p_ref.x = 0;
  p_ref.y = 0;
  cone_draw(RV_, p_ref, 0, "RV", 0);
  // std::cout << "RV = "; print_polygon (RV_);
}

void cone_draw(Polygon_2 C, Coord p_ref, double yaw, std::string ns, int i)
{
  tf2::Quaternion quat, q2;
  geometry_msgs::Quaternion quat_msg;
  quat.setRPY(0, 0, -yaw);
  quat_msg = tf2::toMsg(quat);

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = i;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = p_ref.x;
  marker.pose.position.y = -p_ref.y;
  marker.pose.position.z = 0;
  marker.pose.orientation = quat_msg;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.lifetime = ros::Duration();
  geometry_msgs::Point p;
  marker.points.clear();
  for (Polygon_2::Vertex_const_iterator vertex = C.vertices_begin(); vertex != C.vertices_end(); ++vertex)
  {
    p.x = to_double(vertex->hx());
    p.y = -to_double(vertex->hy());
    p.z = 0;
    marker.points.push_back(p);
  }
  p.x = to_double(C.vertices_begin()->hx());
  p.y = -to_double(C.vertices_begin()->hy());
  p.z = 0;
  marker.points.push_back(p);
  marker_pub_.publish(marker);
}

void line_draw(Coord p1, Coord p2, Coord org, std::string ns)
{
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 99;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = org.x;
  marker.pose.position.y = -org.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.b = 0.9;
  marker.color.a = 0.9;
  marker.scale.x = 0.1;
  marker.lifetime = ros::Duration();
  geometry_msgs::Point p;
  marker.points.clear();
  p.y = -p1.y;
  p.x = p1.x;
  p.z = 0;
  marker.points.push_back(p);
  p.y = -p2.y;
  p.x = p2.y;
  p.z = 0;
  marker.points.push_back(p);
  marker_pub_.publish(marker);
}

void circle_draw(double h, double k, double r, std::string ns, int i)
{
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = i;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = h;
  marker.pose.position.y = -k;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.lifetime = ros::Duration();
  geometry_msgs::Point p;
  marker.points.clear();
  p.z = 0;
  int numOfPoints = 30;
  for (double i = -r; i <= r; i += 2 * r / numOfPoints)
  {
    p.y = i;
    p.x = sqrt(pow(r, 2) - pow(i, 2));
    marker.points.push_back(p);
  }
  for (double i = r; i >= -r; i -= 2 * r / numOfPoints)
  {
    p.y = i;
    p.x = -sqrt(pow(r, 2) - pow(i, 2));
    marker.points.push_back(p);
  }
  marker_pub_.publish(marker);
}

Coord Body2NED_(Coord &coord)
{
  Coord pt;
  Eigen::Vector3f ptBody(coord.x, coord.y, 0);
  Eigen::Vector3f ptNED;
  Eigen::Matrix3f R;
  R << cos(pos_theta_), -sin(pos_theta_), 0,
      sin(pos_theta_), cos(pos_theta_), 0,
      0, 0, 1;
  ptNED = R * ptBody;
  pt.x = ptNED(0) + pos_x_;
  pt.y = ptNED(1) + pos_y_;
  return pt;
}

Coord NED2body_(Coord &coord)
{
  Coord pt;
  Eigen::Matrix3f R;
  Eigen::Vector3f ptBody;
  Eigen::Vector3f ptNED(coord.x - pos_x_, coord.y - pos_y_, 0);
  R << cos(pos_theta_), -sin(pos_theta_), 0,
      sin(pos_theta_), cos(pos_theta_), 0,
      0, 0, 1;
  ptBody = R.inverse() * ptNED;
  pt.x = ptBody(0) - pos_x_;
  pt.y = ptBody(1) - pos_y_;
  return pt;
}

bool reachable_avoidance_velocities()
{
  Polygon_2 C;
  Polygon_with_holes_2 C_union;
  Coord intersect_l;
  Coord intersect_r;
  Coord vel_th;
  Coord p1;
  Coord p2;
  Coord p3;
  Coord p4;
  Coord p_end;
  Coord p_begin;
  double obstacle_theta = 0.0;
  double speed_th = 0.0;
  double slope, slope1 = 0.0;
  double b = 0.0;
  double numerator = 0.0;
  double denominator = 0.0;
  double obs_dist = 0.0;
  double obs_center_dist = 0.0;
  // CCs_.clear();
  RAV_.clear();
  C.clear();

  for (int i = 0; i < obstacle_list_.size(); i++)
  {
    // Calculate VOH
    obs_center_dist = sqrt(pow(obstacle_list_[i].x - pos_x_, 2) + pow(obstacle_list_[i].y - pos_y_, 2));
    obs_dist = obs_center_dist - obstacle_list_[i].r;
    speed_th = obs_dist / col_time_horizon_;                                            // Body
    obstacle_theta = atan2(obstacle_list_[i].x - pos_x_, obstacle_list_[i].y - pos_y_); // Body
    vel_th.x = speed_th * sin(obstacle_theta) + pos_x_;
    vel_th.y = speed_th * cos(obstacle_theta) + pos_y_;

    p1.x = pos_x_;
    p1.y = pos_y_;
    p_begin.x = 0;
    p_begin.y = 0;
    p_end.x = vel_th.x - pos_x_;
    p_end.y = vel_th.y - pos_y_;
    // line_draw(p_end, p_begin, p1, "V_th"); 

    p1.x = obstacle_list_[i].tan_l.x;
    p1.y = obstacle_list_[i].tan_l.y;
    p2.x = pos_x_;
    p2.y = pos_y_;
    p3.x = vel_th.x - (obstacle_list_[i].tan_r.x - obstacle_list_[i].tan_l.x) / 2;
    p3.y = vel_th.y - (obstacle_list_[i].tan_r.y - obstacle_list_[i].tan_l.y) / 2;
    p4.x = vel_th.x;
    p4.y = vel_th.y;

    // Intersect tan_l
    numerator = (((p1.y * p2.x) - (p1.x * p2.y)) * (p3.y - p4.y)) - ((p1.y - p2.y) * ((p3.y * p4.x) - (p3.x * p4.y)));
    denominator = (p1.y - p2.y) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.y - p4.y);
    intersect_l.y = numerator / denominator;
    numerator = (((p1.y * p2.x) - (p1.x * p2.y)) * (p3.x - p4.x)) - ((p1.x - p2.x) * ((p3.y * p4.x) - (p3.x * p4.y)));
    intersect_l.x = numerator / denominator;

    // Intersect tan_r
    p1.x = obstacle_list_[i].tan_r.x;
    p1.y = obstacle_list_[i].tan_r.y;
    p3.x = vel_th.x;
    p3.y = vel_th.y;
    p4.x = vel_th.x + (obstacle_list_[i].tan_r.x - obstacle_list_[i].tan_l.x) / 2;
    p4.y = vel_th.y + (obstacle_list_[i].tan_r.y - obstacle_list_[i].tan_l.y) / 2;
    numerator = (((p1.y * p2.x) - (p1.x * p2.y)) * (p3.y - p4.y)) - ((p1.y - p2.y) * ((p3.y * p4.x) - (p3.x * p4.y)));
    denominator = (p1.y - p2.y) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.y - p4.y);
    intersect_r.y = numerator / denominator;
    numerator = (((p1.y * p2.x) - (p1.x * p2.y)) * (p3.x - p4.x)) - ((p1.x - p2.x) * ((p3.y * p4.x) - (p3.x * p4.y)));
    intersect_r.x = numerator / denominator;

    obstacle_list_[i].voh_r.x = intersect_r.x;
    obstacle_list_[i].voh_r.y = intersect_r.y;
    obstacle_list_[i].voh_l.x = intersect_l.x;
    obstacle_list_[i].voh_l.y = intersect_l.y;

    // ROS_INFO("Obstacle %i Collision Cone: (%f,%f) (%f,%f) (%f,%f) (%f,%f)", i, obstacle_list_[i].tan_r.x, obstacle_list_[i].tan_r.y,
    //           obstacle_list_[i].tan_l.x , obstacle_list_[i].tan_l.y, intersect_l.x, intersect_l.y, intersect_r.x, intersect_r.y);

    obstacle_list_[i].boat_distance = obs_center_dist;
    // If the boat hasn't collided with circumference
    if (obs_center_dist > (obstacle_list_[i].r) && obs_center_dist < 15)
    {
      imminent_collision = 0;
      // Construct the input cone
      C.clear();
      C.push_back(Point_2(intersect_l.x, intersect_l.y));                         // From VOH
      C.push_back(Point_2(obstacle_list_[i].tan_l.x, obstacle_list_[i].tan_l.y)); // Limit of input cone
      C.push_back(Point_2(obstacle_list_[i].tan_r.x, obstacle_list_[i].tan_r.y)); // Limit of input cone
      C.push_back(Point_2(intersect_r.x, intersect_r.y));                         // From VOH
      p1.x = 0;
      p1.y = 0;
      // int n = 0;
      // std::cout << "cone_shape " << i << std::endl;
      // for (VertexIterator vi = C.vertices_begin(); vi != C.vertices_end(); ++vi)
      //   std::cout << "vertex " << n++ << " = " << *vi << "    ";
      // std::cout << std::endl;
      cone_draw(C, p1, 0, "cone_shape", i);
      // Check to see if cone intersercts with RV diamond
      if ((CGAL::do_intersect(C, RV_)))
      {
        // std::cout << "The two polygons intersect." << std::endl;
        if (C_union.is_unbounded())
        {
          // std::cout << "Unbounded" << std::endl;
          Polygon_with_holes_2 temp(C); // insert first cone
          C_union = temp;
        }
        else
        {
          //Join cone with other cones
          CGAL::join(C, C_union, C_union);
          // std::cout << "Joined." << std::endl;
        }
        // print_polygon_with_holes (C_union);
        obstacle_list_[i].col_state = 1;
      }
      else
      {
        // CCs_.join(C);
        // std::cout << "The two polygons do not intersect." << std::endl;
        // return 0;
        obstacle_list_[i].col_state = 0;
      }
      if(check_poly_inside(C)){
        imminent_collision=1;
        return 1;
      }
    }
    else
    {
      if (obs_center_dist < obstacle_list_[i].r)
      {
        // ROS_WARN("Incoming collision!");
        obstacle_list_[i].col_state = 1;
        imminent_collision = 1;
        return 1;           // Evade at all costs
      }
    }
  }
  for(int i=0; i<obstacle_list_.size(); i++){
    // ROS_INFO("Obstacle (%f,%f)",obstacle_list_[i].x, obstacle_list_[i].y);
    // ROS_INFO("Obstacle %i: distance to obst: %f",i,obstacle_list_[i].boat_distance);
    if(min_distance > obstacle_list_[i].boat_distance)
    { 
      min_distance = obstacle_list_[i].boat_distance;
      closest_obst = i;
    }
    // ROS_INFO("Min distance %f",min_distance);
    // ROS_INFO("Closest obstacle: (%f,%f)",obstacle_list_[closest_obst].x,obstacle_list_[closest_obst].y);

  }
  if (!C_union.is_unbounded())
  {
    // Perform a sequence of operations.
    RAV_.insert(C_union);
    RAV_.complement();      // Compute the complement.
    RAV_.intersection(RV_); // Intersect with the clipping rectangle.
    // CCs_.join(C_union);
    // Print the result.
    // std::cout << "The result contains " << RAV_.number_of_polygons_with_holes()
    // << " components:" << std::endl;
    // Get vertices de C_union y llamar a check inside con velocidad deseada
    Pwh_list_2 res;
    p1.x = 0;
    p1.y = 0;
    if (0 < RAV_.number_of_polygons_with_holes())
    {
      // RAV_.polygons_with_holes(std::back_inserter(res));
      // cone_draw((*res.begin()).outer_boundary(), p1, 0, "RAV", 0);
      return 1;
    }
    // print_polygon_with_holes((*res.begin()).outer_boundary());
  }
  // RAV_.join(RV_);
  return 0;
}

bool check_point_between(Vertex p)
{
  Vertex a, b;
  a.x = obstacle_list_[closest_obst].voh_l.x;
  a.y = obstacle_list_[closest_obst].voh_l.y;
  b.x = obstacle_list_[closest_obst].voh_r.x;
  b.y = obstacle_list_[closest_obst].voh_r.y;
  double crossproduct = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);
  // compare versus epsilon for floating point values, or != 0 if using integers
  // To check if the points are aligned
  if (abs(crossproduct) > std::numeric_limits<double>::epsilon())
    return 0;

  //check if a<p<b

  double dotproduct = (p.x - a.x) * (b.x - a.x) + (p.y - a.y)*(b.y - a.y);
  if (dotproduct < 0)
    return 0;

  double squaredlengthba = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
  if (dotproduct > squaredlengthba)
    return 0;

  return 1;
}

void optimal_velocity()
{
  // NED2body();
  if(ftst_chosen_vel.x == 0 && ftst_chosen_vel.y == 0){
    ftst_chosen_vel.x = obstacle_list_[closest_obst].voh_l.x;
    ftst_chosen_vel.y = obstacle_list_[closest_obst].voh_l.y;
  }
  if(!imminent_collision){
    if (!(0 == (goalNED_(0)-pos_x_) && 0 == (goalNED_(1)-pos_y_)))
    {
      Pwh_list_2 res;
      Pwh_list_2::const_iterator it;
      Polygon_with_holes_2 temp;
      Polygon_2 temp_poly;
      Point_2 temp_point;
      // Creates a Min heap of points (order by goal_dist)
      std::priority_queue<Vertex, std::vector<Vertex>, Closest> queue;
      //Iterate over set of polygon with holes
      // std::cout << "The result contains " << RAV_.number_of_polygons_with_holes()
      // << " components:" << std::endl;
      RAV_.polygons_with_holes(std::back_inserter(res));
      for (it = res.begin(); it != res.end(); ++it)
      {
        // std::cout << "--> ";
        temp = *it;
        // Print polygon outer boundary
        std::cout << "{ RAV Outer boundary = ";
        std::cout << "[ " << temp.outer_boundary().size() << " vertices:";
        for (vit = temp.outer_boundary().vertices_begin(); vit != temp.outer_boundary().vertices_end(); ++vit)
        {
          // std::cout << " (" << *vit << ')';
          temp_point = *vit;
          std::cout << " (" << temp_point.x() << ',' << temp_point.y() << ')';
          Vertex temp_vertex;
          temp_vertex.x = CGAL::to_double(temp_point.x());
          temp_vertex.y = CGAL::to_double(temp_point.y());
          temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goalNED_(0), 2) + pow(temp_vertex.y - goalNED_(1), 2));
          // std::cout << temp_vertex.goal_dist << "\n";
          queue.push(temp_vertex);
        }
        std::cout << " ]" << std::endl;
        // std::cout << "  " << temp.number_of_holes() << " holes:" << std::endl;
        unsigned int k = 1;
        for (hit = temp.holes_begin(); hit != temp.holes_end(); ++hit, ++k)
        {
          // std::cout << "    Hole #" << k << " = ";
          // print_polygon(*hit);
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
            temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goalNED_(0), 2) + pow(temp_vertex.y - goalNED_(1), 2));
            queue.push(temp_vertex);
          }
          // std::cout << " ]" << std::endl;
        }
        // std::cout << " }" << std::endl;

        // std::cout << " queue size: " << queue.size() << std::endl;
        // std::cout << " closest vertex: " << queue.top().x << ',' << queue.top().y << std::endl;

        // ROS_WARN("Left tangent: (%f,%f)",  obstacle_list_[closest_obst].tan_l.x, obstacle_list_[closest_obst].tan_l.y);
        // ROS_WARN("Right tangent: (%f,%f)", obstacle_list_[closest_obst].tan_r.x, obstacle_list_[closest_obst].tan_r.y);
        // double l_angle = -atan2(-(obstacle_list_[closest_obst].tan_l.y-pos_y_),obstacle_list_[closest_obst].tan_l.x-pos_x_);
        // double r_angle = -atan2(-(obstacle_list_[closest_obst].tan_r.y-pos_y_),obstacle_list_[closest_obst].tan_r.x-pos_x_);
        // ROS_WARN("Left angle: %f", l_angle);
        // ROS_WARN("Right angle: %f", r_angle);
        // ROS_WARN("Boat angle: %f", pos_theta_);

        Vertex v1,v2,v3;
        v1 = queue.top();
        queue.pop();
        v2 = queue.top();
        queue.pop();
        double v1_angle = -atan2(-(v1.y-pos_y_),v1.x-pos_x_);
        double v2_angle = -atan2(-(v2.y-pos_y_),v2.x-pos_x_);
        Coord c,d;
        c.x = v1.x + obstacle_list_[closest_obst].boat_distance*cos(v1_angle);
        c.y = v1.y + obstacle_list_[closest_obst].boat_distance*sin(v1_angle);
        // double frtst_vel_angle = -atan2(-(ftst_chosen_vel.y-pos_y_),ftst_chosen_vel.x-pos_x_);
        // double angle_aux1 = l_angle-v1_angle;
        // double angle_aux2 = l_angle-v2_angle;
        // double angle_aux3 = v2_angle-r_angle;
        // double angle_aux4 = v1_angle-r_angle;
        bool intersect1 = check_vel_collision(obstacle_list_[closest_obst].voh_l, obstacle_list_[closest_obst].voh_r,
                                              obstacle_list_[closest_obst].tan_l, obstacle_list_[closest_obst].tan_r,
                                              c);

        c.x = v2.x + obstacle_list_[closest_obst].boat_distance*cos(v2_angle);
        c.y = v2.y + obstacle_list_[closest_obst].boat_distance*sin(v2_angle);

        bool intersect2 = check_vel_collision(obstacle_list_[closest_obst].voh_l, obstacle_list_[closest_obst].voh_r,
                                              obstacle_list_[closest_obst].tan_l, obstacle_list_[closest_obst].tan_r,
                                              c);

        if(intersect1){
          if(intersect2){
            desired_velocity(ftst_chosen_vel);
          } else {
            desired_velocity(v2);
          }
        } else {
          desired_velocity(v1);
        }
        // if(abs(obstacle_list_[closest_obst].voh_l.x) > abs(ftst_chosen_vel.x) 
        //     && abs(obstacle_list_[closest_obst].voh_l.y) >  abs(ftst_chosen_vel.y)){
        //   ftst_chosen_vel.x = obstacle_list_[closest_obst].voh_l.x + obstacle_list_[closest_obst].voh_l.x/abs(obstacle_list_[closest_obst].voh_l.x)*1;
        //   ftst_chosen_vel.y = obstacle_list_[closest_obst].voh_l.y + obstacle_list_[closest_obst].voh_l.y/abs(obstacle_list_[closest_obst].voh_l.y)*1;
        // }

        // ROS_WARN("%f < %f < %f ", l_angle, v1_angle, r_angle);
        // ROS_WARN("%f < %f < %f ", l_angle, v2_angle, r_angle);
        // ROS_WARN("%f < %f < %f ", l_angle, frtst_vel_angle, r_angle);
        // ROS_WARN("V1: (%f,%f)", v1.x, v1.y);
        // ROS_WARN("V2: (%f,%f)", v2.x, v2.y);
        // ROS_WARN("Prev: (%f,%f)", ftst_chosen_vel.x, ftst_chosen_vel.y);

        // if((l_angle <= v1_angle && v1_angle <= r_angle) && (l_angle <= v2_angle && v2_angle <= r_angle)){
        //   // if (l_angle < frtst_vel_angle && frtst_vel_angle < r_angle){
        //   if(angle_aux1 < angle_aux2 && angle_aux3 > angle_aux1){
        //       // V1 is closest to voh_l
        //       ROS_WARN("Left rav angle: %f", v1_angle);
        //       v.x = v1.x;
        //       v.y = v1.y;
        //       v = NED2body_(v);
        //       v.y -= 1;
        //       v = Body2NED_(v);
        //       v3.x = v.x;
        //       v3.y = v.y;
        //   } else {
        //     if(angle_aux2 < angle_aux1 && angle_aux4 > angle_aux2){
        //       // V2 is closest to voh_l
        //       ROS_WARN("Left rav angle: %f", v2_angle);
        //       v.x = v2.x;
        //       v.y = v2.y;
        //       v = NED2body_(v);
        //       v.y -= 1;
        //       v = Body2NED_(v);
        //       v3.x = v.x;
        //       v3.y = v.y;
        //     } else {
        //       // V1 is equally close to voh_l than V2
        //       // if(angle_aux1 == angle_aux2){
        //         ROS_WARN("Left rav angle equal case: %f", v1_angle);
        //         v.x = v1.x;
        //         v.y = v1.y;
        //         v = NED2body_(v);
        //         v.y -= 1;
        //         v = Body2NED_(v);
        //         v3.x = v.x;
        //         v3.y = v.y;
        //       // } else {
        //       //   if(angle_aux3 < angle_aux1){
        //       //     // V2 is closest to voh_r
        //       //     ROS_WARN("Right rav angle: %f", v2_angle);
        //       //     v3 = v2;
        //       //   } else {
        //       //     // V1 is closest to voh_r
        //       //     v3 = v1;
        //       //     ROS_WARN("Right rav angle: %f", v1_angle);
        //       //   }
        //     }
        //   }
        //   // }
        //   // if(abs(obstacle_list_[closest_obst].voh_l.x) > abs(ftst_chosen_vel.x) 
        //   //     && abs(obstacle_list_[closest_obst].voh_l.y) >  abs(ftst_chosen_vel.y)){
        //   //   ftst_chosen_vel.x = obstacle_list_[closest_obst].voh_l.x + obstacle_list_[closest_obst].voh_l.x/abs(obstacle_list_[closest_obst].voh_l.x)*1;
        //   //   ftst_chosen_vel.y = obstacle_list_[closest_obst].voh_l.y + obstacle_list_[closest_obst].voh_l.y/abs(obstacle_list_[closest_obst].voh_l.y)*1;
        //   // }
        //     v3 = queue.top();
        //     ftst_chosen_vel.x = v3.x;
        //     ftst_chosen_vel.y = v3.y;
        //     ROS_WARN("Prev: (%f,%f)", ftst_chosen_vel.x, ftst_chosen_vel.y);
        //     desired_velocity(ftst_chosen_vel);
        //     ROS_WARN("DENTRO");
        //   } else {
        //     if(l_angle < v1_angle && v1_angle < r_angle){
        //       ROS_WARN("FUERA v2");
        //       desired_velocity(v2);
        //     } else {
        //       ROS_WARN("FUERA v1");
        //       desired_velocity(v1);
        //     }
        //   }
        // }
      }
    }
  }
  else
  {
    // Point_2 temp_point;
    // std::priority_queue<Vertex, std::vector<Vertex>, Farthest> queue;
    // for (vit = RV_.vertices_begin(); vit != RV_.vertices_end(); ++vit)
    // {
    //   // std::cout << " (" << *vit << ')';
    //   temp_point = *vit;
    //   // std::cout << " (" << temp_point.x() << ',' << temp_point.y() << ')';
    //   Vertex temp_vertex;
    //   temp_vertex.x = CGAL::to_double(temp_point.x());
    //   temp_vertex.y = CGAL::to_double(temp_point.y());
    //   temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - obstacle_list_[closest_obst].x, 2) + pow(temp_vertex.y - obstacle_list_[closest_obst].y, 2));
    //   // std::cout << temp_vertex.goal_dist << "\n";
    //   queue.push(temp_vertex);
    // }
    // desired_velocity(queue.top());
    desired_velocity(ftst_chosen_vel);
  }
}

void desired_velocity(const Vertex &optimal)
{
  std_msgs::Float64 desired_heading;
  std_msgs::Float64 desired_speed;
  Coord p1, p_begin, p_end;
  p1.x = pos_x_;
  p1.y = pos_y_;
  p_begin.x = 0;
  p_begin.y = 0;
  p_end.x = optimal.x;
  p_end.y = optimal.y;
  p_end = NED2body_(p_end);
  line_draw(p_end,p_begin,p1,"RAV_desired_velocity");
  desired_heading.data = atan2(-(optimal.y-pos_y_),optimal.x-pos_x_);
  desired_speed.data = sqrt(pow(p_end.x, 2) + pow(p_end.y, 2));
  ROS_WARN("RAV choosen vertex: %f,%f", optimal.x, optimal.y);
  // ROS_INFO("Desired vo speed: %f", desired_speed.data);
  // ROS_INFO("Desired vo heading: %f", desired_heading.data);
  desired_heading_pub_.publish(desired_heading);
  desired_vel_pub_.publish(desired_speed);
}

void NED2body()
{
  Eigen::Matrix3f R;
  R << cos(pos_theta_), -sin(pos_theta_), 0,
      sin(pos_theta_), cos(pos_theta_), 0,
      0, 0, 1;
  goal_body_ = R * goalNED_;
  // ROS_INFO("Body goal: %f, %f", goal_body_(0), goal_body_(1));
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
  ptNED = R * ptBody; //falta sumarle la distancia al barco
  return ptNED;
}

bool check_point_inside(Polygon_2 pol, Point_2 pt){
  // std::cout << "The point " << pt;
  switch(CGAL::bounded_side_2(pol.vertices_begin(), pol.vertices_end(), pt, Kernel())){
    case CGAL::ON_BOUNDED_SIDE:
      // std::cout << " is inside the polygon.\n";
      return 1;
      break;
    case CGAL::ON_BOUNDARY:
      // std::cout << " is on the polygon boundary.\n";
      break;
    case CGAL::ON_UNBOUNDED_SIDE:
      // std::cout << " is outside the polygon.\n";
      break;
  }
  return 0;
}

bool check_poly_inside(Polygon_2 pol){
  int counter=0;
  int vertex_num=0;
  Point_2 pt;
  for (Polygon_2::Vertex_const_iterator vertex = RV_.vertices_begin(); vertex != RV_.vertices_end(); ++vertex){
    pt = *vertex;
    // std::cout << "Point " << pt << "\n";
    // std::cout << "Vertex " << vertex->x() << vertex->y() << "\n";
    counter+=check_point_inside(pol,pt);
    vertex_num++;
  }
  // std::cout << counter << " " << vertex_num << "\n";
  if(counter == vertex_num){
    ROS_WARN("RV is completely inside the collision cone");
    return 1;
  }
  return 0;
}

bool check_vel_collision(Coord a1, Coord a2, Coord b1, Coord b2, Coord c1){
  Kernel::Segment_2 d_velocity(Point_2(pos_x_,pos_y_),Point_2(c1.x,c1.y));
  Kernel::Segment_2 short_base(Point_2(a1.x,a1.y),Point_2(a2.x,a2.y));
  Kernel::Segment_2 long_base(Point_2(b1.x,b1.y),Point_2(b2.x,b2.y));
  if (CGAL::do_intersect(long_base, d_velocity)){
    if (CGAL::do_intersect(short_base, d_velocity)){
      // std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
      return 1;
    }
  }
  // std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
  return 0;
}