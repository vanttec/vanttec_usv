#include "m4.h"
#include <cmath>

// Speed Challenge

M4::M4(){
  id = 4;
  re_init();
}

USVOutput M4::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  outMsg.goals.clear();
  Eigen::Vector3f goal;
  this->pose = pose;

  switch(outMsg.state){
    case 0: // Initial state, look for first gate to travel to
      goal = get_goal(params.obs_list);

      if(goal.norm() > 0.01){
        outMsg.state = 1;
        outMsg.goals = pack_goal(pose, goal, 1.);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
        first_goal = last_goal;
      }
      break;
    case 1: // Finding second gate
      if(params.green_light){
      goal = get_goal(params.obs_list);

      if(goal.norm() > 0.0001){ // If a goal was actually found
        outMsg.state = 2;
        outMsg.goals = pack_goal(last_goal, goal, 1.);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
        second_goal = last_goal;
      }
      }
      break;
    case 2: // Finding blue buoy to round
      goal = get_blue_buoy_goal(params.obs_list);

      if(goal.norm() > 0.0001){ // If a goal was actually found
        outMsg.state = 3;
        outMsg.status = 1;
        outMsg.goals = round_pack_goal(last_goal, goal, 2.);
        outMsg.goals.push_back(rotate_goal(second_goal, M_PI));
        outMsg.goals.push_back(forward(rotate_goal(first_goal, M_PI), 2.));
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
      } else if(dist(last_goal, pose) < 1) {
        outMsg.goals = pack_goal(last_goal, forward(last_goal, 0.15), 0.35);
        last_goal = outMsg.goals[outMsg.goals.size() - 1];
      }
      break;
  }
  
  return outMsg;
}

Eigen::Vector3f M4::get_goal(std::vector<Obstacle> obs_list){
  double r_dist{0.0}, min_r_dist{max_};
  double g_dist{0.0}, min_g_dist{max_};
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
          if((r_dist < min_r_dist) && unreg(pose + rotM*tmp)){
            red_buoy = tmp;
            min_r_dist = r_dist;
          }
          break;

        case 1: //green
          g_dist = tmp.norm();
          if((g_dist < min_g_dist) && unreg(pose + rotM*tmp)){
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

Eigen::Vector3f M4::get_blue_buoy_goal(std::vector<Obstacle> obs_list){
  double b_dist{0.0}, min_b_dist{max_};
  Eigen::Vector3f b_buoy{0, 0, 0};
  Eigen::Vector3f goal{0, 0, 0}, tmp{0, 0, 0};

  float x_diff, y_diff, theta, m;

  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR;
  rotM << std::cos(pose(2)), - std::sin(pose(2)), 0, 
          std::sin(pose(2)), std::cos(pose(2)), 0, 
          0, 0, 1;
  
  // 1. Search for the nearest blue unregistered buoy
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "round"){
      tmp << obs_list[i].x, obs_list[i].y, 0.0;
      switch(obs_list[i].color){
        case 2: //blue
          b_dist = tmp.norm();
          if((b_dist < min_b_dist) && unreg(pose + rotM*tmp)){
            b_buoy = tmp;
            min_b_dist = b_dist;
          }
          break;
      }
    }
  }

  if(b_buoy.norm() < 0.05)
    return goal;

  // 2. Set main waypoint
  po = b_buoy;
  poseR   = rotM * po;
  register_buoy(pose + poseR);

  goal = pose + poseR;
  goal(2) = last_goal(2);
  return goal;
}

// Set extra waypoints for interpolation
std::vector<Eigen::Vector3f> M4::round_pack_goal(
  Eigen::Vector3f wp_base, Eigen::Vector3f wp_goal, double dist){
  std::vector<Eigen::Vector3f> goal_list;
  goal_list.push_back(wp_base);
  // goal_list.push_back(rotate_goal(diagonal(wp_goal, -2*dist, 0.), M_PI/8));
  goal_list.push_back(rotate_goal(diagonal(wp_goal, 0, dist), 0.));
  goal_list.push_back(rotate_goal(diagonal(wp_goal, dist, 0.), -M_PI_2));
  goal_list.push_back(rotate_goal(diagonal(wp_goal, 0., -dist), M_PI));
  return goal_list;
}