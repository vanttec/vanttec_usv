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
  outMsg.goals.clear();

  switch(outMsg.state){
    case 0: // Initial state, look for slot to travel to
      goal = get_goal(params.obs_list, params.docking_color_choice);

      if(goal.norm() > 0.01){
        std::cout << "psi: " << pose(2) << ", gol: " << goal(2) << ", diff: " << angle_diff(pose(2),goal(2)) << std::endl;
        std::cout << "goal: " << goal(0) << "," << goal(1) << std::endl;
        Eigen::Vector3f goal2 = forward(goal, 1.0);
        std::cout << "goal2: " << goal2(0) << "," << goal2(1) << std::endl;
        if(std::fabs(angle_diff(pose(2),goal(2))) < M_PI_2){
          outMsg.state = 2;
        } else {
          outMsg.state = 1;
          std::vector<int> limit_id = get_min_max_pic_ids(params.obs_list);
          Eigen::Vector3f docking_center_relative = get_docking_face_middlepoint(params.obs_list, limit_id[0], limit_id[1], pose(2));
          Eigen::Vector3f docking_center = tf_body_to_world(pose, docking_center_relative);
          std::cout << "relative center: " << docking_center_relative(0) << "," << docking_center_relative(1) << "," << docking_center_relative(2) << std::endl;
          std::cout << "real center: " << docking_center(0) << "," << docking_center(1) << "," << docking_center(2) << std::endl;
          outMsg.goals.push_back(diagonal(docking_center, -1.0, 5.0));
          outMsg.goals.push_back(diagonal(docking_center, 1.0, 5.0));
        }
        outMsg.goals.push_back(goal);
      }
      std::cout << "fail" << std::endl;

      break;
    case 1: // Slot occupied, turn around to the other face of the docking station
      if(params.wp_arrived){
        outMsg.state = 0;
      }
      break;
    case 2:
      if(params.wp_arrived){
        outMsg.status = 1;
      }
      break;
  }
  
  return outMsg;
}

std::vector<int> M3::get_min_max_pic_ids(std::vector<Obstacle> obs_list){
  int min_y_id{-1}, max_y_id{-1}, picture_count{0};
  float min_y_dist{0.}, max_y_dist{0.};

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
  std::cout << "pic_count: " << picture_count << std::endl;
  if(picture_count != 3){
    min_y_id = max_y_id;
  }

  return std::vector<int>{min_y_id, max_y_id};
}

Eigen::Vector3f M3::get_docking_face_middlepoint(std::vector<Obstacle> obs_list, int min_id, int max_id, double psi){
  int middle_id = 3 - (min_id + max_id);

  double docking_angle =
    std::atan2(
      obs_list[min_id].y - obs_list[max_id].y,
      obs_list[min_id].x - obs_list[max_id].x
    );
  std::cout << "docking angle: " << docking_angle << std::endl;

  double normal_angle = angle_correct(docking_angle + M_PI_2);
  std::cout << "normal angle: " << normal_angle << std::endl;
  // flip direction if it's pointing backwards
  if(std::fabs(normal_angle) > M_PI_2){
    normal_angle = angle_correct(normal_angle + M_PI);
  }

  Eigen::Vector3f out{obs_list[middle_id].x, obs_list[middle_id].y, normal_angle};
  return out;
}

Eigen::Vector3f M3::get_goal(std::vector<Obstacle> obs_list, int choice){
  Eigen::Vector3f goal{0, 0, 0}, tmp{0, 0, 0};  
  float x_diff, y_diff, theta, m;
  Eigen::Vector3f po, poseR, poseRB, poseGB, diff;
  
  // 1. Load boats registry
  std::vector<Obstacle> boat_reg;
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "boat"){
      boat_reg.push_back(obs_list[i]);
    }
  }

  // 2. Search for the dock orientation
  std::vector<int> limit_id = get_min_max_pic_ids(obs_list);
  if(limit_id[0] == limit_id[1]){
    return goal;
  }
  Eigen::Vector3f docking_center = get_docking_face_middlepoint(obs_list, limit_id[0], limit_id[1], pose(2));
  std::cout << "docking psi: " << docking_center(2) << std::endl;

  // 3. Search for the needed picture
  for(int i = 0 ; i < obs_list.size() ; i++){
    if(obs_list[i].type == "picture" && 
    obs_list[i].color == choice){
      tmp << obs_list[i].x, obs_list[i].y, docking_center(2);
      tmp = forward(tmp, -1);
      std::cout << "is occ? " << tmp(0) << "," << tmp(1) << std::endl;
      if(is_occupied(tmp, boat_reg)){
        tmp << (obs_list[limit_id[0]].x + obs_list[limit_id[1]].x) / 2,
        (obs_list[limit_id[0]].y + obs_list[limit_id[1]].y) / 2,
        angle_correct(docking_center(2) + M_PI);
        tmp = forward(tmp, -5.);
      } else {
        std::cout << "not occupied" << std::endl;
      }
      goal = tf_body_to_world(pose,tmp);
      
    }
  }
  return goal;
}

bool M3::is_occupied(const Eigen::Vector3f &picture, std::vector<Obstacle> boat_reg){
  for(int i = 0 ; i < boat_reg.size() ; i++){
    if(dist(picture(0) - boat_reg[i].x, picture(1) - boat_reg[i].y) < 1.9){
      return true;
    }
  }
  return false;
}
