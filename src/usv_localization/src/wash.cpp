#include "wash.h"
#include <cmath>

WASH::WASH(){
  this->pose.x = 0.0;
  this->pose.y = 0.0;
  this->pose.theta = 0.0;
  this->state = 0;
  this->status = 0;
}

WASH::WASH(const USVPose &pose){
  this->pose.x = pose.x;
  this->pose.y = pose.y;
  this->pose.theta = pose.theta;
  this->state = 0;
  this->status = 0;
}

USVOutput WASH::update(const USVPose &pose, const  USVUpdate &params){
  this->goals.clear();
  this->pose = pose;
  switch(this->state){
    case 0:
      // if(params.obs_list.size() > 1){
        this->shoot = false;
        this->goal = get_goal(params.obs_list, 0, pose);
        if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0)){
          // this->goal = forward(this->goal, -4.0);
          this->goal = forward(this->goal, -1.0);
          this->goals.push_back(this->goal);
          this->shooting_wp = this->goal;
          this->state = 1;
        }
      // }
      break;
    case 1:
      // if(params.goal_reached || true){
        if(this->shoot == false){
          this->shoot = true;
        }
        // this->shooting_ang = this->shooting_wp.theta;
        if(params.time_is_up){
          this->shoot = false;
          this->state = 2;
        }
      // }
      break;
  }
  
  outMsg.goals = this->goals;
  outMsg.state = this->state;
  outMsg.shooting_ang = this->shooting_ang;
  outMsg.shoot = this->shoot;
  outMsg.status = this->status;
  outMsg.pivots_to_do = this->pivots_to_do;
  // if(this->goals.size() > 0)
  //   std::cout << "x:" << outMsg.goals[0].x << ", y:" << outMsg.goals[0].y << std::endl;
  return outMsg;
}

USVPose WASH::get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose){
  Gate g{0,0,0,0};
  USVPose p{0,0,0};
  USVPose buoy_t;
  float x_diff, y_diff, theta, m;


  Eigen::Matrix3f rotM;
  Eigen::Vector3f po, poseR, v1, v2, v3, pose1, pose2, pose3;
  std::vector<Eigen::Vector3f> poses;
  rotM << std::cos(pose.theta), - std::sin(pose.theta), 0, 
          std::sin(pose.theta), std::cos(pose.theta), 0, 
          0, 0, 1;


  float min_r_dist{-1}, min_g_dist{-1}, r_dist, g_dist;

  if(obs_list.size() == 3){
    g.x1 = obs_list[0].x;
    g.y1 = obs_list[0].y;  
    g.x2 = obs_list[2].x;
    g.y2 = obs_list[2].y;  
  }

  if(g.x1 == 0 && g.y1 == 0 && g.x2 == 0 && g.y2 == 0)
    return USVPose{0,0,0};

  po << (g.x1 + g.x2) / 2, (g.y1 + g.y2) / 2, 0;
  poseR = rotM*po;

  v1 << obs_list[0].x, obs_list[0].y, 0;
  v2 << obs_list[1].x, obs_list[1].y, 0;
  v3 << obs_list[2].x, obs_list[2].y, 0;
  
  pose1 = rotM*v1;
  pose2 = rotM*v2;
  pose3 = rotM*v3;
  poses.clear();
  poses.push_back(pose1);
  poses.push_back(pose2);
  poses.push_back(pose3);

  std::cout << "POSE: " << pose.x << "," << pose.y << ", " << pose.theta << std::endl;
  std::cout << "FIRST OBJ FROM BODY: " << g.x1 << "," << g.y1 << std::endl;
  std::cout << "FIRST OBJ FROM NED: " << pose1(0)+pose.x << "," << pose1(1)+pose.y << std::endl;
  std::cout << "SECOND OBJ FROM BODY: " << obs_list[1].x << "," << obs_list[1].y << std::endl;
  std::cout << "SECOND OBJ FROM NED: " << pose2(0)+pose.x << "," << pose2(1)+pose.y << std::endl;
  std::cout << "THIRD OBJ FROM BODY: " << g.x2 << "," << g.y2 << std::endl;
  std::cout << "THIRD OBJ FROM NED: " << pose3(0)+pose.x << "," << pose3(1)+pose.y << std::endl;
  std::cout << "GATE FROM BODY: " << po(0) << "," << po(1) << std::endl;

  p.x = pose.x + poseR(0);
  p.y = pose.y + poseR(1);
  std::cout << "GATE NED: " << p.x << "," << p.y << std::endl;

  x_diff = pose1(0) - pose3(0);
  y_diff = pose1(1) - pose3(1);
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

  // SHAPE CHOICES: 0 = circle, 1 = duck, 2 = plus, 3 = triangle
  // COLOR CHOICES: 0 = RED, 1 = GREEN, 2 = BLUE, 3 = YELLOW

  int ind = get_index("shape", 1, obs_list);
  p.x = pose.x + poses[ind](0);
  p.y = pose.y + poses[ind](1);
    
  return p;
}

USVPose WASH::forward(const USVPose &goal, float distance){
  USVPose p{0,0,0};
  p.theta = goal.theta;
  p.x = goal.x + distance * std::cos(p.theta);
  p.y = goal.y + distance * std::sin(p.theta);
  std::cout << "FORWARD TRAVEL: " << p.x << ", " << p.y << ", " << p.theta << std::endl;
  return p;
}

int WASH::get_index(std::string choice_type, int choice, std::vector<Obstacle> obs_list){
  std::string type_list[] = {"circle", "duck", "plus", "triangle"};
  if(choice_type == "color"){
    for(int i = 0 ; i < 3 ; i++){
      if(obs_list[i].color == choice)
        return i;
    }
  } else if(choice_type == "shape"){
    for(int i = 0 ; i < 3 ; i++){
      if(choice > 3)
        return -1;
      if(obs_list[i].type == type_list[choice])
        return i;
    }
  }
  return -1;
}



// BODY TO NED -> MAT*VEC
// NED TO BODY -> MAT_INV*VEC