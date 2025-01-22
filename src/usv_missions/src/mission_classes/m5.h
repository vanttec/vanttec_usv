#ifndef M5_H
#define M5_H

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "mission.h"

class M5 : public Mission {
 public:
  M5();
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;

 private:  
  Eigen::Vector3f get_goal(std::vector<Obstacle> obs_list);
};

#endif  // M5_HF