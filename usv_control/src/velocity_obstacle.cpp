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

#include "print_utils.h"
#include "usv_perception/obstacles_list.h"

// NAMESPACES ------------------------------------------------------------------
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;
typename CGAL::Polygon_2<Kernel>::Vertex_const_iterator   vit;
typename CGAL::Polygon_with_holes_2<Kernel>::Hole_const_iterator  hit;

// STRUCTS ---------------------------------------------------------------------
struct Coord{ 
  // Position
  double x;
  double y;
};
struct Obstacle{
  // Obstacle position in x
  double x = 0.0;
  // Obstacle position in y
  double y = 0.0;
  // Obstacle radius
  double r = 0.0;
  // Right tangent angle
  Coord tan_r = {0,0};
  // Left tangent angle
  Coord tan_l = {0,0};
};
struct Vertex{ 
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
    int operator() (const Vertex& v1, const Vertex& v2) 
    { 
        return v1.goal_dist > v2.goal_dist; 
    } 
};

// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * Input Parameters
  * */
double robot_radius_ = 0.5; //meters
double time_horizon_ = 1.0; //seconds
double max_long_acceleration_ = 0.3; //m/s^3
double max_yaw_acceleration_ = 0.1; //rad/s^2
double max_vel_ = 1.5; //m/s
/**
  * Rechable velocities diamond. 
  * */
Polygon_2 RV_;
/**
  * RAV poligon set. 
  * */
Polygon_set_2 RAV_;
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
//ros::Subscriber waypoints_sub_;
ros::Subscriber obstacles_sub_;
/**
  * Publishers
  * */
ros::Publisher desiered_vel_pub_;
ros::Publisher desiered_heading_pub_;
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
  * Goal position vector NED
  * */
Eigen::Vector3f goalNED_ = Eigen::Vector3f::Zero();
Eigen::Vector3f goal_body_ = Eigen::Vector3f::Zero();
/**
  * Waypoint list vector
  * */
//std::vector<double> waypoint_list_;
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
const std::string topic_goal_sub_ = "/usv_control/los/target";
//const std::string topic_waypoints_sub_ = "/mission/waypoints";
const std::string topic_obstacles_sub_ = "/usv_perception/lidar_detector/obstacles";
const std::string topic_desiered_vel_pub_ = "/guidance/desired_speed";
const std::string topic_desiered_heading_pub_ = "/guidance/desired_heading";
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
  * @param pose[in]: Received pose.
  * @return void.
  * */
void on_pose_msg(const geometry_msgs::Pose2D::ConstPtr &pose);
/**
  * Callback to obtain current NED target pose. 
  * @param pose[in]: Recive target pose.
  * @return void.
  * */
void on_goal_msg(const geometry_msgs::Pose2D::ConstPtr &goal);;
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
void desiered_velocity(const Vertex& optimal);
/**
  * Rotate position from NED to body. 
  * @return void.
  * */
void NED2body();

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_obstacle");
  ros::NodeHandle vo_node("velocity_obstacle");
  ros::Rate loop_rate(100);
  initialize(vo_node);
  while (ros::ok()){
    if(collision_cone()){
      reachable_velocities();
      if(reachable_avoidance_velocities()){
        optimal_velocity();
      }
      return 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};

void initialize(ros::NodeHandle &vo_node){
  // Subscribers
  vel_sub_ = vo_node.subscribe(topic_vel_sub_, queue_size_, &on_speed_msg);
  pose_sub_ = vo_node.subscribe(topic_pose_sub_, queue_size_, &on_pose_msg);
  goal_sub_ = vo_node.subscribe(topic_goal_sub_, queue_size_, &on_goal_msg);
  //waypoints_sub_ = vo_node.subscribe(topic_waypoints_sub_, queue_size_, 
  //                                  &on_waypoints_msg);
  obstacles_sub_ = vo_node.subscribe(topic_obstacles_sub_, queue_size_, 
                                    &on_obstacles_msg);
  // Publishers
  desiered_vel_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desiered_vel_pub_, queue_size_);
  desiered_heading_pub_ = vo_node.advertise<std_msgs::Float64>(
    topic_desiered_heading_pub_, queue_size_);
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

/*void on_waypoints_msg(const std_msgs::Float32MultiArray::ConstPtr &msg){
  waypoint_list_.empty();
  int leng = msg->layout.data_offset;
  for (int i = 0; i>leng; ++i){
    waypoint_list_.push_back(msg->data[i]);
  }
}*/

void on_goal_msg(const geometry_msgs::Pose2D::ConstPtr &msg){
  goalNED_(0) = msg->x;
  goalNED_(1) = msg->y;
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

bool collision_cone(){
  if(0 < obstacle_list_.size()){
    //Find tangent points
    // Boat circle : (y-a)^2 + (x-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (y-c)^2 + (x-d)^2 = r1^2 c=obstacleY d=obstacleX r1=obstacle radius
    // D = distance of circle centers
    // Boat circle : (x-a)^2 + (y-b)^2 = ro^2 a=0 b=0 ro=distance to obstacle
    // Obstacle circle: (x-c)^2 + (y-d)^2 = r1^2 c=obstacleX d=obstacleY r1=obstacle radius
    for(int i = 0; i<obstacle_list_.size(); ++i){
      double a = 0;
      double b = 0;
      double r0 = sqrt(pow(obstacle_list_[i].x,2)+pow(obstacle_list_[i].y,2)); 
      double c = obstacle_list_[i].x;
      double d = obstacle_list_[i].y;
      double r1 = obstacle_list_[i].r;
      ROS_INFO("Obstacle radius: %f coordinates %f,%f", r1, obstacle_list_[i].x, obstacle_list_[i].y);
      double D = r0;
      double delta = 0.25*sqrt((D+r0+r1)*(D+r0-r1)*(D-r0+r1)*(-D+r0+r1));
      ROS_INFO("Obstacle delta: %f", delta);
      obstacle_list_[i].tan_r.y = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) - 2*((a-c)/(D*D))*delta;
      ROS_INFO("Obstacle y1 %f", obstacle_list_[i].tan_r.y); 
      obstacle_list_[i].tan_l.y = (b+d)/2 + ((d-b)*(r0*r0-r1*r1))/(2*D*D) + 2*((a-c)/(D*D))*delta;
      ROS_INFO("Obstacle y2 %f", obstacle_list_[i].tan_l.y); 
      obstacle_list_[i].tan_r.x = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) + 2*((b-d)/(D*D))*delta;
      ROS_INFO("Obstacle x1 %f", obstacle_list_[i].tan_r.x); 
      obstacle_list_[i].tan_l.x = (a+c)/2 + ((c-a)*(r0*r0-r1*r1))/(2*D*D) - 2*((b-d)/(D*D))*delta;
      ROS_INFO("Obstacle x2 %f", obstacle_list_[i].tan_l.x); 
      ROS_INFO("Obstacle %i intersection1:%f,%f intersection2:%f,%f", i, obstacle_list_[i].tan_r.x , obstacle_list_[i].tan_r.y, obstacle_list_[i].tan_l.x , obstacle_list_[i].tan_l.y);
    }
    return 1;
  }
  return 0;
}

void reachable_velocities(){
  // No se considera la velocidad lateral 
  double speed_long = speed_x_; //sqrt(pow(speed_x_,2)+pow(speed_y_,2));
  double v_max = speed_long + max_long_acceleration_*time_horizon_;
  v_max = (max_vel_ < v_max) ? v_max : max_vel_;
  double v_min = speed_long - max_long_acceleration_*time_horizon_;
  double w_max = speed_yaw_ + max_yaw_acceleration_*time_horizon_;
  double w_min = speed_yaw_ - max_yaw_acceleration_*time_horizon_;
  RV_.clear();
  RV_.push_back (Point_2 (v_max, 0));
  RV_.push_back (Point_2 (speed_long, w_max));
  RV_.push_back (Point_2 (v_min, 0));
  RV_.push_back (Point_2 (speed_long, w_min));
  std::cout << "RV = "; print_polygon (RV_);
}

bool reachable_avoidance_velocities(){
  Polygon_2 C;
  Polygon_with_holes_2 C_union;
  //Polygon_with_holes_2* ptr = &C_union;
  for(int i = 0; i<obstacle_list_.size(); ++i){
    // Construct the input cone
    C.clear();
    C.push_back (Point_2 (0, 0));
    C.push_back (Point_2 (obstacle_list_[i].tan_l.x, obstacle_list_[i].tan_l.y));
    C.push_back (Point_2 (obstacle_list_[i].tan_r.x, obstacle_list_[i].tan_r.y));
    std::cout << "C = "; print_polygon (C);
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
      std::cout << "The two polygons do not intersect." << std::endl;
    }
  }
  if(!C_union.is_unbounded()){
    // Perform a sequence of operations.
    RAV_.clear();
    RAV_.insert(C_union);
    RAV_.complement();              // Compute the complement.
    RAV_.intersection (RV_);        // Intersect with the clipping rectangle.
    // Print the result.
    std::cout << "The result contains " << RAV_.number_of_polygons_with_holes()
              << " components:" << std::endl;
    if(0 < RAV_.number_of_polygons_with_holes()){
      return 1;
    }
  }
  return 0;
}

void optimal_velocity(){
  NED2body();
  if (!(0 == goal_body_(0) && 0 == goal_body_(1))){
    Pwh_list_2 res;
    Pwh_list_2::const_iterator it;
    Polygon_with_holes_2 temp;
    Polygon_2 temp_poly;
    Point_2 temp_point;
    // Creates a Min heap of points (order by goal_dist) 
    std::priority_queue<Vertex, std::vector<Vertex>, Comparator> queue;
    //Iterate over set of polygon with holes
    RAV_.polygons_with_holes (std::back_inserter (res));
    for (it = res.begin(); it != res.end(); ++it) {
      std::cout << "--> ";
      temp = *it;
      // Print polygon outer boundary
      std::cout << "{ Outer boundary = ";
      std::cout << "[ " << temp.outer_boundary().size() << " vertices:";
      for (vit = temp.outer_boundary().vertices_begin(); vit != temp.outer_boundary().vertices_end(); ++vit){
        //std::cout << " (" << *vit << ')';
        temp_point = *vit;
        std::cout << " (" << temp_point.x() << ',' << temp_point.y()<< ')';
        Vertex temp_vertex;
        temp_vertex.x = CGAL::to_double(temp_point.x());
        temp_vertex.y = CGAL::to_double(temp_point.y());
        temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goal_body_(0),2)+pow(temp_vertex.y - goal_body_(1),2));
        queue.push(temp_vertex);
      }
      std::cout << " ]" << std::endl;

      std::cout << "  " << temp.number_of_holes() << " holes:" << std::endl;
      unsigned int k = 1;
      for (hit = temp.holes_begin(); hit != temp.holes_end(); ++hit, ++k)
      {
        std::cout << "    Hole #" << k << " = ";
        //print_polygon (*hit);
        temp_poly = *hit;
        std::cout << "[ " << temp_poly.size() << " vertices:";
        for (vit = temp_poly.vertices_begin(); vit != temp_poly.vertices_end(); ++vit){
          //std::cout << " (" << *vit << ')';
          temp_point = *vit;
          std::cout << " (" << temp_point.x() << ',' << temp_point.y()<< ')';
          Vertex temp_vertex;
          temp_vertex.x = CGAL::to_double(temp_point.x());
          temp_vertex.y = CGAL::to_double(temp_point.y());
          temp_vertex.goal_dist = sqrt(pow(temp_vertex.x - goal_body_(0),2)+pow(temp_vertex.y - goal_body_(1),2));
          queue.push(temp_vertex);
        }
        std::cout << " ]" << std::endl;
      }
      std::cout << " }" << std::endl;

      std::cout << " queue size: " << queue.size() << std::endl;
      std::cout << " closest vertex: " << queue.top().x << ',' << queue.top().y << std::endl;
      desiered_velocity(queue.top());
    }
  }
}

void desiered_velocity(const Vertex& optimal){
  std_msgs::Float64 desiered_heading;
  std_msgs::Float64 desiered_speed;
  desiered_heading.data = atan2(optimal.y, optimal.x) + pos_theta_;
  desiered_speed.data = sqrt(pow(optimal.x,2) + pow(optimal.y,2));
  desiered_heading_pub_.publish(desiered_heading);
  desiered_vel_pub_.publish(desiered_speed);
}

void NED2body(){
  Eigen::Matrix3f R;
  R << cos(pos_theta_), -sin(pos_theta_), 0,
  sin(pos_theta_), cos(pos_theta_), 0,
  0, 0, 1;
  goal_body_ = R*goalNED_; 
}
