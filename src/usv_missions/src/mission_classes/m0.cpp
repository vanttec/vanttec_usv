#include "m0.h"
#include <cmath>

M0::M0(){
  id = 0;
  re_init();
}

USVOutput M0::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  if(params.wp_arrived){
    outMsg.status = 1;
  }
  return outMsg;
}