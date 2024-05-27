#ifndef M2_H
#define M2_H

// Follow the Path Challenge

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "mission.h"

class M2 : public Mission {
 public:
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;

 private:  
  Eigen::Vector3f get_goal(std::vector<Obstacle> obs_list);
};

#endif  // M2_H
