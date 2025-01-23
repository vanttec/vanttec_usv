#include "m2.h"
#include <cmath>

// Follow the Path Challenge

M2::M2(){
  id = 2;
  re_init();
}

USVOutput M2::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  outMsg.goals.clear();
  Eigen::Vector3f goal;
  this->pose = pose;

  switch(outMsg.state){
    case 0: // Initial state, look for a gate to travel to
      goal = get_goal(params.obs_list);

      if(goal.norm() > 0.01){
        outMsg.state = 1;
        outMsg.goals = pack_goal(pose, goal, 1.);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
      }
      break;
    case 1: // Finding next gate
      goal = get_goal(params.obs_list);

      if(goal.norm() > 0.0001){ // If a goal was actually found
        outMsg.goals = pack_goal(last_goal, goal, 1.);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
      } else if (dist(pose, last_goal) < 0.5) {
        outMsg.state = 2;
        outMsg.status = 1;
      }
      break;
  }
  
  return outMsg;
}

Eigen::Vector3f M2::get_goal(std::vector<Obstacle> obs_list){
  double r_dist{0.0}, min_r_dist{-1.0};
  double g_dist{0.0}, min_g_dist{-1.0};
  Eigen::Vector3f red_buoy{0, 0, 0}, green_buoy{0, 0, 0};
  Eigen::Vector3f goal{0, 0, 0}, tmp{0, 0, 0};

  float x_diff, y_diff, theta, m;

  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR, poseRB, poseGB, diff;
  rotM << std::cos(pose(2)), - std::sin(pose(2)), 0, 
          std::sin(pose(2)), std::cos(pose(2)), 0, 
          0, 0, 1;
  
  // 1. Search for the nearest red & green unregistered buoys
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "round"){
      tmp << obs_list[i].x, obs_list[i].y, 0.0;
      switch(obs_list[i].color){
        case 0: //red
          r_dist = tmp.norm();
          if((r_dist < min_r_dist || min_r_dist == -1) && unreg(pose + rotM*tmp)){
            red_buoy = tmp;
            min_r_dist = r_dist;
          }
          break;

        case 1: //green
          g_dist = tmp.norm();
          if((g_dist < min_g_dist || min_g_dist == -1) && unreg(pose + rotM*tmp)){
            green_buoy = tmp;
            min_g_dist = g_dist;
          }
          break;
      }
    }
  }

  if(red_buoy.norm() < 0.05 || green_buoy.norm() < 0.05)
    return goal;

  // 2. Set main waypoint
  po = (red_buoy + green_buoy) / 2;
  poseR   = rotM * po;
  poseRB  = rotM * red_buoy;
  poseGB  = rotM * green_buoy;
  register_buoy(pose + poseRB);
  register_buoy(pose + poseGB);

  goal = pose + poseR;
  diff = poseRB - poseGB;

  // initial calculation of the gate direction
  goal(2) = angle_correct(-std::atan2(diff(0), diff(1)) + 2 * M_PI);

  // flip direction if it's pointing backwards
  if(std::fabs(angle_correct(std::fabs(goal(2) - pose(2)))) > M_PI_2)
    goal(2) = angle_correct(goal(2) + M_PI);

  // Debug
  // std::cout << "POSE: " << pose(0) << "," << pose(1) << ", " << pose(2) << std::endl;
  // std::cout << "RED FROM BODY: " << red_buoy(0) << "," << red_buoy(1) << std::endl;
  // std::cout << "RED FROM NED : " << poseRB(0)+pose(0) << "," << poseRB(1)+pose(1) << std::endl;
  // std::cout << "GREEN FROM BODY: " << green_buoy(0) << "," << green_buoy(1) << std::endl;
  // std::cout << "GREEN FROM NED: " << poseGB(0)+pose(0) << "," << poseGB(1)+pose(1) << std::endl;
  // std::cout << "GATE FROM BODY: " << po(0) << "," << po(1) << std::endl;
  // std::cout << "GOAL: " << goal(0) << "," << goal(1) << ", " << goal(2) << std::endl;

  return goal;
}
