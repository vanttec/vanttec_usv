#ifndef USV_H
#define USV_H

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
  USVPose goal;
  int yellow_found;
  int black_found;
};

struct Gate {
  float x1, y1, x2, y2;
};


class USV{
 public:
  USV();
  USV(const USVPose &pose);
  USVOutput update(const USVPose &pose, const  USVUpdate &params);

 private:
  std::vector<Obstacle> obs_list;
  float x{0}, y{0}, theta{0};
  int mission_num{2}, state{0}, status{0};
  USVPose pose, goal;
  USVOutput outMsg;
  int yellow_found;
  int black_found;

  USVPose get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose);
  USVPose forward(int distance);
};

#endif  // USV_H
