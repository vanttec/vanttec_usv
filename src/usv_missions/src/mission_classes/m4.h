#ifndef M4_H
#define M4_H

// Speed Challenge

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "mission.h"

class M4 : public Mission {
 public:
  M4();
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;

 private:  
  Eigen::Vector3f first_goal, second_goal;
  Eigen::Vector3f get_goal(std::vector<Obstacle> obs_list);
  Eigen::Vector3f get_blue_buoy_goal(std::vector<Obstacle> obs_list);
  std::vector<Eigen::Vector3f> round_pack_goal(Eigen::Vector3f wp_base, 
        Eigen::Vector3f wp_goal, double dist);
};

#endif  // M4_H
