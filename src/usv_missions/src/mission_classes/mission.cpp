#include "mission.h"
#include <cmath>

Mission::Mission(){
  re_init();
}

Mission::Mission(const Eigen::Vector3f &pose){
  re_init();
  this->pose = pose;
}

int Mission::get_id(){
  return id;
}

int Mission::set_status(int new_status){
  outMsg.status = new_status;
}

Eigen::Vector3f Mission::forward(const Eigen::Vector3f &goal, double distance){
  Eigen::Vector3f p;
  p << std::cos(goal(2)), std::sin(goal(2)), 0.0;
  return goal + distance * p;
}

Eigen::Vector3f Mission::diagonal(const Eigen::Vector3f &goal, double x_trans, double y_trans){
  Eigen::Matrix3f rotM;
  rotM << std::cos(goal(2)), - std::sin(goal(2)), 0, 
          std::sin(goal(2)), std::cos(goal(2)), 0, 
          0, 0, 1;
  Eigen::Vector3f p;
  p << x_trans, y_trans, 0.0;
  return goal + rotM * p;
}

double Mission::dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2){
  return std::sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

double Mission::dist(double x_diff, double y_diff){
  return std::sqrt(pow(x_diff, 2) + pow(y_diff, 2));
}

double Mission::angle_correct(double ang){
  double out = std::fmod(ang + M_PI , 2*M_PI);
  if (out < 0) {
      out += 2*M_PI;
  }
  return out - M_PI;
}

double Mission::angle_diff(double ang1, double ang2){
  double out = std::fabs(ang1 - ang2);
  if(out > M_PI){
    return M_PI*2 - out;
  } else {
    return out;
  }
}

void Mission::re_init(){
  outMsg.state = 0;
  outMsg.status = 0;
  outMsg.goals.clear();
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

Eigen::Vector3f Mission::rotate_goal(const Eigen::Vector3f &goal, double ang){
  return Eigen::Vector3f{goal(0), goal(1), angle_correct(goal(2)+ang)};
}

Eigen::Vector3f Mission::tf_body_to_world(Eigen::Vector3f p, Eigen::Vector3f relative){
  Eigen::Matrix3f rotM;
  rotM << std::cos(p(2)), - std::sin(p(2)), 0, 
          std::sin(p(2)), std::cos(p(2)), 0, 
          0, 0, 1;
  return p + rotM * relative;
}
