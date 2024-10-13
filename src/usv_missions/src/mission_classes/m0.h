#ifndef M0_H
#define M0_H

#include "mission.h"

class M0 : public Mission {
 public:
  M0();
  USVOutput update(const Eigen::Vector3f &pose, const  USVUpdate &params) override;
};

#endif  // M0_H
