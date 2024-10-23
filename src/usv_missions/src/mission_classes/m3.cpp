#include "m3.h"
#include <cmath>

// Docking Challenge

M3::M3(){
  id = 3;
  re_init();
}

USVOutput M3::update(const Eigen::Vector3f &pose, const  USVUpdate &params)
{
  Eigen::Vector3f goal;
  this->pose = pose;

  switch(outMsg.state){
    case 0: // Initial state, look for slot to travel to
      goal = get_goal(params.obs_list, params.docking_color_choice);

      if(goal.norm() > 0.01){
        if(std::fabs(angle_diff(pose(2),goal(2))) < M_PI_2){
          outMsg.state = 1;
          outMsg.status = 1;
        }
        outMsg.goals.clear();
        outMsg.goals.push_back(goal);
      }
      break;
  }
  
  return outMsg;
}

Eigen::Vector3f M3::get_goal(std::vector<Obstacle> obs_list, int choice){
  Eigen::Vector3f goal{0, 0, 0}, tmp{0, 0, 0};
  float min_y_dist{0.}, max_y_dist{0.};
  int min_y_id{-1}, max_y_id{-1}, picture_count{0};
  
  float x_diff, y_diff, theta, m;

  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR, poseRB, poseGB, diff;
  rotM << std::cos(pose(2)), - std::sin(pose(2)), 0, 
          std::sin(pose(2)), std::cos(pose(2)), 0, 
          0, 0, 1;
  
  // 1. Load boat reg
  std::vector<Obstacle> boat_reg;
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "boat"){
      boat_reg.push_back(obs_list[i]);
    }
  }

  // 2. Search for the dock orientation
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "picture"){
      picture_count++;
      if(min_y_id == -1 || obs_list[i].y < min_y_dist){
        min_y_dist = obs_list[i].y;
        min_y_id = i;
      }
      if(max_y_id == -1 || obs_list[i].y > max_y_dist){
        max_y_dist = obs_list[i].y;
        max_y_id = i;
      }
    }
  }

  if(min_y_id == max_y_id || picture_count != 3){
    return goal;
  }

  double docking_angle = 
    std::atan2(
      obs_list[min_y_id].y - obs_list[max_y_id].y,
      obs_list[min_y_id].x - obs_list[max_y_id].x
    );
  double normal_angle = angle_correct(docking_angle + M_PI_2);
  // flip direction if it's pointing backwards
  if(std::fabs(angle_correct(std::fabs(normal_angle - pose(2)))) > M_PI_2){
    normal_angle = angle_correct(normal_angle + M_PI_2);
  }

  // 3. Search for the needed picture
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "picture" && 
    obs_list[i].color == choice){
      tmp << obs_list[i].x, obs_list[i].y, normal_angle;
      tmp = forward(tmp, -1);
      if(is_occupied(tmp, boat_reg)){
        tmp << (obs_list[min_y_id].x + obs_list[max_y_id].x) / 2,
        (obs_list[min_y_id].y + obs_list[max_y_id].y) / 2,
        angle_correct(normal_angle + M_PI);
        tmp = forward(tmp, -5.);
      }
      goal = pose + rotM * tmp;
    }
  }
  return goal;
}

bool M3::is_occupied(const Eigen::Vector3f &picture, std::vector<Obstacle> boat_reg){
  for(int i = 0 ; i < boat_reg.size() ; i++){
    if(dist(picture(0) - boat_reg[i].x, picture(1) - boat_reg[i].y) < 1.){
      return true;
    }
  }
  return false;
}
