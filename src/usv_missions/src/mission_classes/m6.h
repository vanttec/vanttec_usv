#ifndef M6_H
#define M6_H

// Return to Home Challenge

#include "mission.h"

class M6 : public Mission {
 public:
  M6();
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;

 private:  
  void push_buoy(std::array<Eigen::Vector3f, 2> &og_buoys, Eigen::Vector3f new_buoy);
  Eigen::Vector3f get_goal(std::vector<Obstacle> obs_list);
};

#endif  // M6_H
