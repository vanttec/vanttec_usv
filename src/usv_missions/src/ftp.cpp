#include "ftp.h"
#include <cmath>

FTP::FTP(){
  this->pose.x = 0.0;
  this->pose.y = 0.0;
  this->pose.theta = 0.0;
}

FTP::FTP(const USVPose &pose){
  this->pose.x = pose.x;
  this->pose.y = pose.y;
  this->pose.theta = pose.theta;
}

USVOutput FTP::update(const USVPose &pose, const  USVUpdate &params){
  this->goals.clear();
  this->pose = pose;
  switch(this->state){
    case 0:
      // if(params.obs_list.size() > 1){
        this->goal = get_goal(params.obs_list, 0, pose);
        if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0))
          this->state = 1;
      // }
      break;
    case 1:
      if(params.goal_reached){
        this->last_g.x = 0.0;
        this->last_g.y = 0.0;
        this->last_r.x = 0.0;
        this->last_r.y = 0.0;
        this->goal = forward(this->last_goal, 1.0);
        this->state = 2;
      }
      break;
    case 2:
      if(params.goal_reached){
        this->last_g.x = 0.0;
        this->last_g.y = 0.0;
        this->last_r.x = 0.0;
        this->last_r.y = 0.0;
        this->goal = get_goal(params.obs_list, 0, pose);
        if(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0){
          this->goal = forward(this->last_goal, 1.0);
          this->state = 3;
        } else {
          this->state = 1;
          std::cout << "FOUND OTHER GATE" << this->goal.x << "," << this->goal.y << "," << this->goal.theta << std::endl;
        }
      }
      break;
    case 3:
      if(params.goal_reached){
        if(this->yellow_found_list.size() > 0)
          this->pivots_to_do = this->yellow_found_list.size();
        this->state = 4;
      }
      break;
    case 4:
      std::cout << params.pivots_done << " todo: " << this->pivots_to_do << std::endl;
      if(params.pivots_done){
          this->state = 5;
          this->status = 1;
          this->pivots_to_do = -1;
      }
    break;
  }


  if(!(this->goal.x == 0 && this->goal.y == 0 && this->goal.theta == 0) && this->state < 3){
    this->last_goal = this->goal;
    // std::cout << "CORRECTLY SAVED LAST GOAL: " << last_goal.x << ", " << last_goal.y << ", " << last_goal.theta << std::endl;

    USVPose int_buoy = intercepts(this->pose, this->goal, this->black_found_list, this->yellow_found_list);
    USVPose mid_goal;
    if(!(int_buoy.x == 0 && int_buoy.y == 0 && int_buoy.theta == 0)){
      // std::cout << "INTERCEPTING BUOY: " << int_buoy.x << ", " << int_buoy.y << std::endl;
      int choice = 0;
      // if(dist(int_buoy, this->last_g) < dist(int_buoy, this->last_r) && dist(int_buoy, this->last_g) > 1.0)
      //   choice = 2;
      // else if(dist(int_buoy, this->last_r) < dist(int_buoy, this->last_g) && dist(int_buoy, this->last_r) > 1.0)
      //   choice = 1;
      if(dist(int_buoy, this->last_g) > dist(int_buoy, this->last_r))
        choice = 2;
      else if(dist(int_buoy, this->last_r) > dist(int_buoy, this->last_g))
        choice = 1;
      switch(choice){
        case 1:
          if((this->last_r.x != 0) && (this->last_r.y != 0)){
            mid_goal = USVPose{(int_buoy.x+this->last_r.x)/2,(int_buoy.y+this->last_r.y)/2,0};
            this->goals.push_back(mid_goal);
          }
          break;
        case 2:
          if((this->last_g.x != 0) && (this->last_g.y != 0)){
            mid_goal = USVPose{(int_buoy.x+this->last_g.x)/2,(int_buoy.y+this->last_g.y)/2,0};
            this->goals.push_back(mid_goal);
          }
          break;
      }
    }
  }

  this->goals.push_back(this->goal);
  

  outMsg.black_found = this->black_found_list.size();
  outMsg.yellow_found = this->yellow_found_list.size();
  outMsg.goals = this->goals;
  outMsg.state = this->state;
  outMsg.status = this->status;
  outMsg.pivots_to_do = this->pivots_to_do;
  // if(this->goals.size() > 0)
  //   std::cout << "x:" << outMsg.goals[0].x << ", y:" << outMsg.goals[0].y << std::endl;
  return outMsg;
}

USVPose FTP::get_goal(std::vector<Obstacle> obs_list, int added_distance, const USVPose &pose){
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
    // if(obs_list[i].type == "round"){
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
        po << obs_list[i].x, obs_list[i].y, 0;
        poseR = rotM*po;
        buoy_t.x = pose.x + poseR(0);
        buoy_t.y = pose.y + poseR(1);
        std::cout << "Yellow buoy " << buoy_t.x << "," << buoy_t.y << std::endl;
        if(is_new_buoy(this->yellow_found_list, buoy_t))
          this->yellow_found_list.push_back(buoy_t);
        break;
      case 4: // black
        po << obs_list[i].x, obs_list[i].y, 0;
        poseR = rotM*po;
        buoy_t.x = pose.x + poseR(0);
        buoy_t.y = pose.y + poseR(1);
        if(is_new_buoy(this->black_found_list, buoy_t))
          this->black_found_list.push_back(buoy_t);
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

USVPose FTP::forward(const USVPose &goal, float distance){
  USVPose p{0,0,0};
  p.theta = goal.theta;
  p.x = goal.x + distance * std::cos(p.theta);
  p.y = goal.y + distance * std::sin(p.theta);
  std::cout << "FORWARD TRAVEL: " << p.x << ", " << p.y << ", " << p.theta << std::endl;
  return p;
}

bool FTP::is_new_buoy(std::vector<USVPose> reg, const USVPose &buoy){
  for(int i = 0 ; i < reg.size() ; i++){
    if(std::sqrt(std::pow(reg[i].x - buoy.x,2) + std::pow(reg[i].y - buoy.y,2)) < 0.5)
      return false;
  }
  return true;
}

USVPose FTP::intercepts(const USVPose &pose, const USVPose &goal, const std::vector<USVPose> &black_l, const std::vector<USVPose> &yellow_l){
  float c_t;
  for(int i = 0 ; i < black_l.size() ; i++){
    // std::cout << "BLACK BUOYS: " << black_l[i].x << "," << black_l[i].y << std::endl;
    if(dist(pose, black_l[i]) < dist(pose, goal)){
      c_t = calculate_c_e(pose, goal, black_l[i]);
      if(c_t < 0.7)
        return black_l[i];
    }
  }
  for(int i = 0 ; i < yellow_l.size() ; i++){
    // std::cout << "YELLOW BUOYS: " << yellow_l[i].x << ","  << yellow_l[i].y << std::endl;
    if(dist(pose, yellow_l[i]) < dist(pose, goal)){
      c_t = calculate_c_e(pose, goal, yellow_l[i]);
      if(c_t < 0.7)
        return yellow_l[i];
    }
  }
  return USVPose{0,0,0};
}

float FTP::dist(const USVPose &p1, const USVPose &p2){
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

float FTP::calculate_c_e(const USVPose &pose, const USVPose &goal, const USVPose &obj){
  float x, y, m, b, n, c, xp, yp;
  x = goal.x - pose.x;
  y = goal.y - pose.y;
  m = x/y;
  b = x - m*y;
  n = -1/m;
  c = obj.x - n * obj.y;
  yp = (c - b) / (m - n);
  xp = m*yp + b;
  return std::sqrt((obj.x - xp)*(obj.x - xp) + (obj.y - yp)*(obj.y - yp));
}


// BODY TO NED -> MAT*VEC
// NED TO BODY -> MAT_INV*VEC