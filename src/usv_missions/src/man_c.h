#ifndef MAN_C_H
#define MAN_C_H

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

struct USVParams {
  int mission_num;
};

struct USVPose {
  float x, y;
  float theta;
};

struct Obstacle {
  float x, y;
  std::string type;
  int color; // 0 = red, 1 = green, 2 = blue, 3 = yellow, 4 = black
};

struct USVUpdate {
  std::vector<Obstacle> obs_list;
  bool goal_reached;
};

struct USVOutput {
  int state; // State Machine of the mission
  int status; // Final status of the mission
  std::vector<USVPose> goals;
};

struct Gate {
  float x1, y1, x2, y2;
};


class MAN_C{
 public:
  MAN_C();
  MAN_C(const USVPose &pose);
  USVOutput update(const USVPose &pose, const  USVUpdate &params);

 private:
  std::vector<Obstacle> obs_list;
  float x{0}, y{0}, theta{0};
  int mission_num{1}, state{0}, status{0}, pivots_to_do{-1};
  USVPose pose, goal, last_goal;
  std::vector<USVPose> goals;
  USVOutput outMsg;
  std::vector<USVPose> yellow_found_list;
  std::vector<USVPose> black_found_list;
  USVPose last_r, last_g;

  USVPose get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose);
  USVPose forward(const USVPose &goal, float distance);
};

#endif  // MAN_C_H
