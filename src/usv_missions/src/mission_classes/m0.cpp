#include "m0.h"
#include <cmath>

M0::M0(){
  id = 0;
  re_init();
}

USVOutput M0::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  if(dist(params.last_goal, pose) < 0.8){
    outMsg.status = 1;
  }
  return outMsg;
}