#ifndef M3_H
#define M3_H

// Docking Challenge

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "mission.h"

class M3 : public Mission {
 public:
  M3();
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;

 private:  
  Eigen::Vector3f get_goal(std::vector<Obstacle> obs_list, int choice);
  bool is_occupied(const Eigen::Vector3f &picture, std::vector<Obstacle> boat_reg);
};

#endif  // M3_H
