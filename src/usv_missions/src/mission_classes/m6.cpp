#include "m6.h"

// Return to Home Challenge

M6::M6(){
  id = 6;
  re_init();
}

USVOutput M6::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  Eigen::Vector3f goal;
  this->pose = pose;

  switch(outMsg.state){
    case 0: // Look for the final black gate to cross
      goal = get_goal(params.obs_list);

      if(goal.norm() > 0.01){
        outMsg.state = 1;
        outMsg.status = 1;
        outMsg.goals = pack_goal(pose, goal, 1.);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
      }
      break;
  }
  
  return outMsg;
}

void M6::push_buoy(std::array<Eigen::Vector3f, 2> &og_buoys, Eigen::Vector3f new_buoy) {
  og_buoys[1] = new_buoy;
  if(og_buoys[1].norm() < og_buoys[0].norm()){
    Eigen::Vector3f tmp_buoy;
    tmp_buoy = og_buoys[0];
    og_buoys[0] = og_buoys[1];
    og_buoys[1] = tmp_buoy;
  }
}

Eigen::Vector3f M6::get_goal(std::vector<Obstacle> obs_list){
  double min_b_dist[2]{max_, max_};
  std::array<Eigen::Vector3f, 2> b_buoys{
    Eigen::Vector3f{max_, max_, 0.}, 
    Eigen::Vector3f{max_, max_, 0.}};

  Eigen::Vector3f goal{0, 0, 0}, tmp{0, 0, 0};

  float x_diff, y_diff, theta, m;

  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR, poseB1, poseB2, diff;
  rotM << std::cos(pose(2)), - std::sin(pose(2)), 0, 
          std::sin(pose(2)), std::cos(pose(2)), 0, 
          0, 0, 1;
  
  // 1. Search for the black unregistered buoys
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "round"){
      tmp << obs_list[i].x, obs_list[i].y, 0.0;
      switch(obs_list[i].color){
        case 4: //black
          if((tmp.norm() < min_b_dist[1]) && unreg(pose + rotM*tmp)){
            push_buoy(b_buoys, tmp);
            min_b_dist[0] = b_buoys[0].norm();
            min_b_dist[1] = b_buoys[1].norm();
          }
          break;
      }
    }
  }

  std::cout << min_b_dist[0] << "," << min_b_dist[1] << std::endl;

  if(min_b_dist[0] < 0.01 || min_b_dist[1] < 0.01)
    return goal;

  // 2. Set main waypoint
  po = (b_buoys[0] + b_buoys[1]) / 2;
  poseR   = rotM * po;
  poseB1  = rotM * b_buoys[0];
  poseB2  = rotM * b_buoys[1];
  register_buoy(pose + poseB1);
  register_buoy(pose + poseB2);

  goal = pose + poseR;
  diff = poseB1 - poseB2;

  // initial calculation of the gate direction
  goal(2) = angle_correct(-std::atan2(diff(0), diff(1)) + 2 * M_PI);

  // flip direction if it's pointing backwards
  if(std::fabs(angle_correct(std::fabs(goal(2) - pose(2)))) > M_PI_2)
    goal(2) = angle_correct(goal(2) + M_PI);

  // Debug
  // std::cout << "POSE: " << pose(0) << "," << pose(1) << ", " << pose(2) << std::endl;
  // std::cout << "RED FROM BODY: " << red_buoy(0) << "," << red_buoy(1) << std::endl;
  // std::cout << "RED FROM NED : " << poseB1(0)+pose(0) << "," << poseB1(1)+pose(1) << std::endl;
  // std::cout << "GREEN FROM BODY: " << green_buoy(0) << "," << green_buoy(1) << std::endl;
  // std::cout << "GREEN FROM NED: " << poseB2(0)+pose(0) << "," << poseB2(1)+pose(1) << std::endl;
  // std::cout << "GATE FROM BODY: " << po(0) << "," << po(1) << std::endl;
  // std::cout << "GOAL: " << goal(0) << "," << goal(1) << ", " << goal(2) << std::endl;

  return goal;
}
