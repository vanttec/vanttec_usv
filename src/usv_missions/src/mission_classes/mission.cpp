#include "mission.h"
#include <cmath>

Mission::Mission(){
  re_init();
}

Mission::Mission(const Eigen::Vector3f &pose){
  re_init();
  this->pose = pose;
}

Eigen::Vector3f Mission::forward(const Eigen::Vector3f &p_ref, double distance){
  Eigen::Vector3f p;
  p << std::cos(p_ref(2)), std::sin(p_ref(2)), 0;
  return p_ref + distance * p;
}

double Mission::dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2){
  return std::sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

double Mission::dist(double x_diff, double y_diff){
  return std::sqrt(pow(x_diff, 2) + pow(y_diff, 2));
}

double Mission::angle_correct(double ang){
  return std::fmod(ang + M_PI, 2 * M_PI) - M_PI;
}

void Mission::re_init(){
  outMsg.state = 0;
  outMsg.status = 0;
}

bool Mission::unreg(Eigen::Vector3f buoy){
  for(int i = 0 ; i < buoy_reg.size() ; i++) {
    if(dist(buoy, buoy_reg[i]) < 0.8)
      return false;
  }
  return true;
}

void Mission::register_buoy(Eigen::Vector3f buoy){
  buoy_reg.push_back(buoy);
}

// Set extra waypoints for interpolation
std::vector<Eigen::Vector3f> Mission::pack_goal(Eigen::Vector3f wp_base, Eigen::Vector3f wp_goal, double dist){
  std::vector<Eigen::Vector3f> goal_list;
  goal_list.push_back(wp_base);
  // goal_list.push_back(wp_goal);
  // goal_list.push_back(forward(wp_goal, -dist));
  goal_list.push_back(forward(wp_goal, dist));
  return goal_list;
}