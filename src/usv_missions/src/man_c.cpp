#include "man_c.h"
#include <cmath>

MAN_C::MAN_C(){
  this->pose.x = 0.0;
  this->pose.y = 0.0;
  this->pose.theta = 0.0;
}

MAN_C::MAN_C(const USVPose &pose){
  this->pose.x = pose.x;
  this->pose.y = pose.y;
  this->pose.theta = pose.theta;
}

USVOutput MAN_C::update(const USVPose &pose, const  USVUpdate &params){
  this->goals.clear();
  this->pose = pose;
  switch(this->state){
    case 0:
      this->goal = get_goal(params.obs_list, 0, pose);
      if(!(this->goal.x == 0 && this->goal.y == 0))
        this->state = 1;
      break;
    case 1:
      if(params.goal_reached){
        this->goal = forward(this->last_goal, 1.0);
        this->state = 2;
      }
      std::cout << "GOAL REACHED: " << params.goal_reached << std::endl;
      break;
    case 2:
      if(params.goal_reached){
        this->goal = get_goal(params.obs_list, 0, pose);
        if(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0){
          this->goal = forward(this->last_goal, 1.0);
        } else {
          this->state = 3;
          std::cout << "FOUND OTHER GATE" << this->goal.x << "," << this->goal.y << "," << this->goal.theta << std::endl;
        }
      }
      break;
    case 3:
      if(params.goal_reached){
        this->goal = forward(this->last_goal, 1.0);
        this->state = 4;
      }
      break;
    case 4:
      if(params.goal_reached){
        this->state = 5;
        this->status = 1;
      }
      break;
  }


  if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0))
    this->last_goal = this->goal;

  this->goals.push_back(this->goal);
  
  outMsg.goals = this->goals;
  outMsg.state = this->state;
  outMsg.status = this->status;
  return outMsg;
}

USVPose MAN_C::get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose){
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
    // if(obs_list[i].type == "marker_buoy"){
    switch(obs_list[i].color){
      case 0: // red
        r_dist = std::sqrt(obs_list[i].x*obs_list[i].x + obs_list[i].y*obs_list[i].y);
        if(r_dist < min_r_dist || min_r_dist == -1){
          g.x1 = obs_list[i].x;
          g.y1 = obs_list[i].y;
          min_r_dist = r_dist;
        }
        break;
      case 1: // green
        g_dist = std::sqrt(obs_list[i].x*obs_list[i].x + obs_list[i].y*obs_list[i].y);
        if(g_dist < min_g_dist || min_g_dist == -1){
          g.x2 = obs_list[i].x;
          g.y2 = obs_list[i].y;
          min_g_dist = g_dist;
        }
        break;
      case 2: // blue
        // why huh?
        break;
      case 3: // yellow
        // why huh?
        break;
      case 4: // black
        // why huh?
        break;
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
  this->last_r = USVPose{pose.x + poseRB(0), pose.y + poseRB(1), poseRB(2)};
  this->last_g = USVPose{pose.x + poseGB(0), pose.y + poseGB(1), poseGB(2)};

  // std::cout << "POSE: " << pose.x << "," << pose.y << ", " << pose.theta << std::endl;
  // std::cout << "RED FROM BODY: " << g.x1 << "," << g.y1 << std::endl;
  // std::cout << "RED FROM NED: " << poseRB(0)+pose.x << "," << poseRB(1)+pose.y << std::endl;
  // std::cout << "GREEN FROM BODY: " << g.x2 << "," << g.y2 << std::endl;
  // std::cout << "GREEN FROM NED: " << poseGB(0)+pose.x << "," << poseGB(1)+pose.y << std::endl;
  // std::cout << "GATE FROM BODY: " << po(0) << "," << po(1) << std::endl;

  p.x = pose.x + poseR(0);
  p.y = pose.y + poseR(1);
  // std::cout << "GATE NED: " << p.x << "," << p.y << std::endl;

  x_diff = poseRB(0) - poseGB(0);
  y_diff = poseRB(1) - poseGB(1);
  y_diff *= -1;

  // std::cout << "x_diff: " << x_diff << ", y_diff: " << y_diff << std::endl;

  p.theta = std::atan2(std::fabs(x_diff), std::fabs(y_diff) + 0.001);
  // std::cout << "FIRST THETA: " << p.theta << std::endl;
  if(x_diff < 0)
    p.theta = M_PI - p.theta;
  if(y_diff < 0)
    p.theta *= -1;

  // std::cout << "THETA ASSIGNED IN QUADRANT: " << p.theta << std::endl;

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
  // std::cout << "THETA FIXED FOR 180°: " << p.theta << std::endl;
  
  return p;
}

USVPose MAN_C::forward(const USVPose &goal, float distance){
  USVPose p{0,0,0};
  p.theta = goal.theta;
  p.x = goal.x + distance * std::cos(p.theta);
  p.y = goal.y + distance * std::sin(p.theta);
  std::cout << "FORWARD TRAVEL: " << p.x << ", " << p.y << ", " << p.theta << std::endl;
  return p;
}

// BODY TO NED -> MAT*VEC
// NED TO BODY -> MAT_INV*VEC