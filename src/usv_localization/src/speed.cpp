#include "speed.h"
#include <cmath>

SPEED::SPEED(){
  this->pose.x = 0.0;
  this->pose.y = 0.0;
  this->pose.theta = 0.0;
  this->state = 0;
  this->status = 0;
}

SPEED::SPEED(const USVPose &pose){
  this->pose.x = pose.x;
  this->pose.y = pose.y;
  this->pose.theta = pose.theta;
  this->state = 0;
  this->status = 0;
}

USVOutput SPEED::update(const USVPose &pose, const  USVUpdate &params){
  this->goals.clear();
  this->pose = pose;
  switch(this->state){
    case 0:
        this->goal = get_goal(params.obs_list, 0, pose);
        if(this->goal.x != 0 && this->goal.y != 0 && this->goal.theta != 0){
	        this->starting_gate = this->goal;
          this->goals.push_back(this->goal);
          this->goal = goal_trans(this->goal, 3.5, 0);
          this->state = 1;
          std::cout << "STATE 0->1" << std::endl;
        }
      break;
    case 1:
      if(params.goal_reached){
        this->goal = get_goal(params.obs_list, 0, pose);
        if(this->yellow_found){
          this->goals.push_back(goal_trans(this->yellow_buoy, -2.5,0));
          this->goals.push_back(goal_trans(this->yellow_buoy, 0,2.5));
          this->goals.push_back(goal_trans(this->yellow_buoy, 5.0,0));
          this->goals.push_back(goal_trans(this->yellow_buoy, 0,-2.5));
          this->goals.push_back(this->starting_gate);
          this->goal = goal_trans(this->starting_gate, -2.5,0);
          // this->goal = this->starting_gate;
          std::cout << "ROUNDING YELLOW BUOY!" << std::endl;
          this->state = 2;
          std::cout << "STATE 1->2" << std::endl;
        } else{
          this->goal = goal_trans(this->last_goal, 2.5, 0);
          std::cout << "FINDING YELLOW BUOY... at " << this->goal.x << ", " << this->goal.y << std::endl;

        }
      }
      break;
    case 2:
      if(params.goal_reached){
        this->state = 3;
        std::cout << "STATE 2->3" << std::endl;
        this->status = 1;
      }
      break;
  }

  if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0)){
    this->last_goal = this->goal;
    // std::cout << "CORRECTLY SAVED LAST GOAL: " << last_goal.x << ", " << last_goal.y << ", " << last_goal.theta << std::endl;
  }

  this->goals.push_back(this->goal);
  
  outMsg.goals = this->goals;
  outMsg.state = this->state;
  outMsg.status = this->status;
  return outMsg;
}

USVPose SPEED::get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose){
  bool gate_found = false;
  Gate g{0,0,0,0};
  USVPose p{0,0,0};
  USVPose buoy_t;
  float x_diff, y_diff, theta, m;

  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR, rv, gv, poseRB, poseGB;
  rotM << std::cos(pose.theta), - std::sin(pose.theta), 0, 
          std::sin(pose.theta), std::cos(pose.theta), 0, 
          0, 0, 1;



  float min_r_dist{-1}, min_g_dist{-1}, r_dist, g_dist;

  for(int i = 0 ; i < obs_list.size() && !gate_found ; i++){
    if(obs_list[i].type == "round"){
      switch(obs_list[i].color){
        case 0: // red
          r_dist = std::sqrt(obs_list[i].x*obs_list[i].x + obs_list[i].y*obs_list[i].y);
          if(r_dist < min_r_dist || min_r_dist == -1){
            g.x2 = obs_list[i].x;
            g.y2 = obs_list[i].y;
            min_r_dist = r_dist;
          }
          break;
        case 1: // green
          g_dist = std::sqrt(obs_list[i].x*obs_list[i].x + obs_list[i].y*obs_list[i].y);
          if(g_dist < min_g_dist || min_g_dist == -1){
            g.x1 = obs_list[i].x;
            g.y1 = obs_list[i].y;
            min_g_dist = g_dist;
          }
          break;
        case 2: // blue
          // DONT CARE
          break;
        case 3: // yellow
          po << obs_list[i].x, obs_list[i].y, 0;
          poseR = rotM*po;
          buoy_t.x = pose.x + poseR(0);
          buoy_t.y = pose.y + poseR(1);
          std::cout << "Yellow buoy " << buoy_t.x << "," << buoy_t.y << std::endl;
          this->yellow_found = true;
          this->yellow_buoy=buoy_t;
          this->yellow_buoy.theta = this->starting_gate.theta;
          break;
        case 4: // black
          // why? huh
          break;
      }
    }
  }

  if((g.x1 != 0 || g.y1 != 0) && (g.x2 != 0 || g.y2 != 0))
    gate_found = true;
  
  if(!gate_found)
    return USVPose{0,0,0};

  po << (g.x1 + g.x2) / 2, (g.y1 + g.y2) / 2, 0;
  poseR = rotM*po;

  rv << g.x1, g.y1, 0;
  poseRB = rotM*rv;
  gv << g.x2, g.y2, 0;
  poseGB = rotM*gv;

   std::cout << "POSE: " << pose.x << "," << pose.y << ", " << pose.theta << std::endl;
   std::cout << "RED FROM BODY: " << g.x1 << "," << g.y1 << std::endl;
   std::cout << "RED FROM NED: " << poseRB(0)+pose.x << "," << poseRB(1)+pose.y << std::endl;
   std::cout << "GREEN FROM BODY: " << g.x2 << "," << g.y2 << std::endl;
   std::cout << "GREEN FROM NED: " << poseGB(0)+pose.x << "," << poseGB(1)+pose.y << std::endl;
   std::cout << "GATE FROM BODY: " << po(0) << "," << po(1) << std::endl;

  p.x = pose.x + poseR(0);
  p.y = pose.y + poseR(1);
  std::cout << "GATE NED: " << p.x << "," << p.y << std::endl;

  x_diff = poseRB(0) - poseGB(0);
  y_diff = poseRB(1) - poseGB(1);
  y_diff *= -1;

  // std::cout << "x_diff: " << x_diff << ", y_diff: " << y_diff << std::endl;

  p.theta = std::atan2(std::fabs(x_diff), std::fabs(y_diff) + 0.001);
  std::cout << "FIRST THETA: " << p.theta << std::endl;
  if(x_diff < 0)
    p.theta = M_PI - p.theta;
  if(y_diff < 0)
    p.theta *= -1;

  std::cout << "THETA ASSIGNED IN QUADRANT: " << p.theta << std::endl;

  // FIX 180° TURN IN DIRECTION
  theta = pose.theta;
  if(std::fabs(theta - p.theta) > M_PI){
    if(theta > p.theta)
      theta -= 2*M_PI;
    else
      theta += 2*M_PI;
  }
  if(std::fabs(theta - p.theta) > M_PI/2){
    if(p.theta < 0)
      p.theta+=M_PI;
    else
      p.theta-=M_PI;
  }
  std::cout << "THETA FIXED FOR 180°: " << p.theta << std::endl;
  
  return p;
}

USVPose SPEED::goal_trans(const USVPose &huh, float x_t, float y_t){
  USVPose p{0,0,0};
  p.theta = huh.theta;
  if(x_t != 0){
    p.x = huh.x + x_t*std::cos(p.theta);
    p.y = huh.y + x_t*std::sin(p.theta);
  } else {
    p.x = huh.x - y_t*std::sin(p.theta);
    p.y = huh.y + y_t*std::cos(p.theta);
  }
  std::cout << "GOAL TRANSLATION TRAVEL: \n From " << huh.x << ", " << huh.y << "\n To " << p.x << ", " << p.y << ", " << p.theta << std::endl;
  return p;
}

// BODY TO NED -> MAT*VEC
// NED TO BODY -> MAT_INV*VEC
