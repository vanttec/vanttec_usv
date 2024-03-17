#ifndef FTP_H
#define FTP_H

// Follow The Path

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
  int pivots_done;
};

struct USVOutput {
  int state; // State Machine of the mission
  int status; // Final status of the mission
  std::vector<USVPose> goals;
  int yellow_found;
  int black_found;
  int pivots_to_do;
};

struct Gate {
  float x1, y1, x2, y2;
};


class FTP{
 public:
  FTP();
  FTP(const USVPose &pose);
  USVOutput update(const USVPose &pose, const  USVUpdate &params);

 private:
  std::vector<Obstacle> obs_list;
  float x{0}, y{0}, theta{0};
  int mission_num{2}, state{0}, status{0}, pivots_to_do{-1};
  USVPose pose, goal, last_goal;
  std::vector<USVPose> goals;
  USVOutput outMsg;
  std::vector<USVPose> yellow_found_list;
  std::vector<USVPose> black_found_list;
  USVPose last_r, last_g;

  USVPose get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose);
  USVPose forward(const USVPose &goal, float distance);
  USVPose intercepts(const USVPose &pose, const USVPose &goal, const std::vector<USVPose> &black_l, const std::vector<USVPose> &yellow_l);
  float calculate_c_e(const USVPose &pose, const USVPose &goal, const USVPose &obj);
  float dist(const USVPose &p1, const USVPose &p2);
  bool is_new_buoy(std::vector<USVPose> reg, const USVPose &buoy);
};

#endif  // FTP_H
