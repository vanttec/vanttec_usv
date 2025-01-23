#ifndef MISSION_H
#define MISSION_H

// Mission parent class

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <array>
#include <algorithm>


struct USVPose {
  double x, y;
  double theta;
};

struct Obstacle {
  double x, y;
  std::string type;
  int color; // 0 = red, 1 = green, 2 = blue, 3 = yellow, 4 = black
};

struct State {
  int state;
  int status;
  std::vector<Eigen::Vector3f> goals;
};

struct USVUpdate {
  std::vector<Obstacle> obs_list;
  int docking_color_choice{1};
  bool wp_arrived;
};

struct USVOutput {
  int state; // State Machine
  int status; // Mission complete
  std::vector<Eigen::Vector3f> goals;
};

struct Gate {
  double x1, y1, x2, y2;
};

class Mission {
 public:
  Mission();
  Mission(const Eigen::Vector3f &pose);
  void re_init();
  virtual ~Mission() = default;
  virtual USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) = 0;
  int get_id();
  int set_status(int new_status);

 protected:
  std::vector<Obstacle> obs_list;
  int id{0};
  double max_ = std::numeric_limits<double>::max();
  Eigen::Vector3f pose;
  Eigen::Vector3f last_goal;
  USVOutput outMsg;
  std::vector<Eigen::Vector3f> yellow_buoy_list;
  std::vector<Eigen::Vector3f> black_buoy_list;
  std::vector<Eigen::Vector3f> buoy_reg;

  Eigen::Vector3f forward(const Eigen::Vector3f &goal, double distance);
  Eigen::Vector3f diagonal(const Eigen::Vector3f &goal, double x_trans, double y_trans);
  Eigen::Vector3f rotate_goal(const Eigen::Vector3f &goal, double ang);
  double dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);
  double dist(double x_diff, double y_diff);
  double angle_correct(double ang);
  double angle_diff(double ang1, double ang2);
  bool unreg(Eigen::Vector3f buoy);
  void register_buoy(Eigen::Vector3f buoy);
  std::vector<Eigen::Vector3f> pack_goal(Eigen::Vector3f wp_base, 
                                Eigen::Vector3f wp_goal, double dist);
  Eigen::Vector3f tf_body_to_world(Eigen::Vector3f p, Eigen::Vector3f relative);
};

#endif  // MISSION_H
