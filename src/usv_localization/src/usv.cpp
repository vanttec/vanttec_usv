#include "usv.h"
#include <cmath>

USV::USV(){
  this->pose.x = 0.0;
  this->pose.y = 0.0;
  this->pose.theta = 0.0;
}

USV::USV(const USVPose &pose){
  this->pose.x = pose.x;
  this->pose.y = pose.y;
  this->pose.theta = pose.theta;
}

USVOutput USV::update(const USVPose &pose, const  USVUpdate &params){
  switch(this->state){
    case 0:
      if(params.obs_list.size() > 1){
        std::cout << "A" << std::endl;
        this->goal = get_goal(params.obs_list, 0, pose);
        if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0)){
          this->state = 1;
        }
      } else {
        std::cout << "HUH" << std::endl;
        if(params.obs_list.size() == 1){
          std::cout << params.obs_list[0].x << std::endl;
        } else 
          std::cout << "PINI" << params.obs_list.size() << " " << std::endl;
      }
      break;
    case 1:
      if(params.goal_reached){
        this->state = 2;
        this->goal = forward(1);
      }
      break;
    case 2:
      if(!params.goal_reached){
        this->goal = get_goal(params.obs_list, 0, pose);
        if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0)){
          this->state = 1;
        } else {
          this->state = 3;
          // this->goal = forward(1);
        }
      }
      break;
    case 3:
      if(params.goal_reached){
        this->state = 4;
        this->status = 1;
      }
      break;
  }
  outMsg.black_found = this->black_found;
  outMsg.yellow_found = this->yellow_found;
  outMsg.goal.x = this->goal.x;
  outMsg.goal.y = this->goal.y;
  outMsg.goal.theta = this->goal.theta;
  outMsg.state = this->state;
  outMsg.status = this->status;
  return outMsg;
}

USVPose USV::get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose){
  bool gate_found = false;
  Gate g{0,0,0,0};
  USVPose p{0,0,0};
  for(int i = 0 ; i < obs_list.size() && !gate_found ; i++){
    // if(obs_list[i].type == "round"){
    switch(obs_list[i].color){
      case 0: // red
        g.x1 = obs_list[i].x;
        g.y1 = obs_list[i].y;
        if(g.x2 != 0 || g.y2 != 0)
          gate_found = true;
        break;
      case 1: // green
        g.x2 = obs_list[i].x;
        g.y2 = obs_list[i].y;
        if(g.x1 != 0 || g.y1 != 0)
          gate_found = true;
        break;
      case 2: // blue
        break;
      case 3: // yellow
        //TODO: Validation, for yellow and black buoys to not add repeated
        // this->yellow_found++;
        break;
      case 4: // black
        // this->black_found++;
        break;
    }
    // }
  }

  std::cout << "FOUND: " << gate_found << std::endl;

  if(!gate_found)
    return USVPose{0,0,0};

  p.theta = std::atan2((g.x2 - g.x1), (g.y2 - g.y1));
  // p.x = (g.x1 + g.x2) / 2 + added_distance * std::cos(p.theta);
  // p.y = (g.y1 + g.y2) / 2 + added_distance * std::sin(p.theta);

  Eigen::Matrix3f rotM;
  rotM << std::cos(pose.theta), - std::sin(pose.theta), 0, 
          std::sin(pose.theta), std::cos(pose.theta), 0, 
          0, 0, 1;
  Eigen::Vector3f po, poseR;
  po << (g.x1 + g.x2) / 2, (g.y1 + g.y2) / 2, pose.theta;
  poseR = rotM.inverse()*po;


  std::cout << "Goal: " << poseR(0) << "," << poseR(1) << std::endl;
  // p.x = pose.x + (g.x1 + g.x2) / 2;
  // p.y = pose.y + (g.y1 + g.y2) / 2;

  p.x = poseR(0);
  p.y = poseR(1);
  
  // p.x = this->pose.x + p.x * std::cos(this->pose.theta) - p.y * std::sin(this->pose.theta);
  // p.y = this->pose.y + p.x * std::sin(this->pose.theta) + p.y * std::cos(this->pose.theta);
  // p.x = p.x * std::cos(-this->pose.theta) - p.y * std::sin(-this->pose.theta);
  // p.y = p.x * std::sin(-this->pose.theta) + p.y * std::cos(-this->pose.theta);
  
  // // y = mx + b
  // float m = (g.x2 - g.x1) / (g.y2 - g.y1);
  // float n = -1/m;
  // float b = p.x - n * p.y;

  return p;
}

USVPose USV::forward(int distance){
  USVPose p{0,0,0};
  p.theta = this->goal.theta;
  p.x = this->goal.x + distance * std::cos(p.theta);
  p.y = this->goal.y + distance * std::sin(p.theta);
  return p;
}