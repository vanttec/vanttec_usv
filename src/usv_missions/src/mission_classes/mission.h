#ifndef MISSION_H
#define MISSION_H

// Mission parent class

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>

struct USVPose {
  double x, y;
  double theta;
};

struct Obstacle {
  double x, y;
  std::string type;
  int color; // 0 = red, 1 = green, 2 = blue, 3 = yellow, 4 = black
};

struct USVUpdate {
  std::vector<Obstacle> obs_list;
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

 protected:
  std::vector<Obstacle> obs_list;
  int mission_num{0};
  Eigen::Vector3f pose;
  Eigen::Vector3f last_goal;
  USVOutput outMsg;
  std::vector<Eigen::Vector3f> yellow_buoy_list;
  std::vector<Eigen::Vector3f> black_buoy_list;
  std::vector<Eigen::Vector3f> buoy_reg;

  Eigen::Vector3f forward(const Eigen::Vector3f &goal, double distance);
  double dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);
  double dist(double x_diff, double y_diff);
  double angle_correct(double ang);
  bool unreg(Eigen::Vector3f buoy);
  void register_buoy(Eigen::Vector3f buoy);
  std::vector<Eigen::Vector3f> pack_goal(Eigen::Vector3f wp_base, 
                                Eigen::Vector3f wp_goal, double dist);
};

#endif  // MISSION_H
