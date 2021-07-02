#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <ros/console.h>
#include <usv_control/trajectory.h>

/**
 * Represents a 3x3 matrix
 * */
typedef Eigen::Matrix<double, 3, 3> xMatrix3;
/**
 * Represents a 3x1 vector
 * */
typedef Eigen::Matrix<double, 3, 1> xVector3;

using namespace Eigen;
using std::endl;

class AdaptiveSlidingModeControl
{
public:
  //Thruster outputs
  float starboard_t;
  float port_t;

  //Sensor feedback
  float x;
  float y;
  float psi;
  float u;
  float v;
  float r;

  static const float integral_step = 0.01;

  int testing;
  int arduino;
  float starting;
  int traj_counter;
  int leng;

  //Tracking variables
  float x_d;
  float y_d;
  float xdot_d;
  float ydot_d;
  float psi_d;
  float r_d;
  float past_psi_d;
  float psi_d_dif;

  //Auxiliry variables
  float e_x;
  float e_y;
  float e_x_dot;
  float e_y_dot;
  float s_x;
  float s_y;

  int sign_x_sm;
  int sign_y_sm;
  int sign_sx;
  int sign_sy;

  //Model pysical parameters
  float Xu;
  float Yv;
  float Nr;
  static const float X_u_dot = -2.25;
  static const float Y_v_dot = -23.13;
  static const float N_r_dot = -2.79;
  float Xuu;
  static const float Yvv = -99.99;
  static const float Nrr = -3.49;
  static const float m = 30;
  static const float Iz = 4.1;
  static const float B = 0.41;
  static const float c = 0.78;
  static const float l = 0.5;

  float f_u;
  float g_u;
  float f_v;
  float f_r;
  float g_r;
  
  float Ka_x;
  float Ka_y;
  float Ka_dot_x;
  float Ka_dot_y;
  float Ka_dot_last_x;
  float Ka_dot_last_y;
  float ua_x;
  float ua_y;
  float Tx;
  float Tz;

  //Controller gains
  float k_x;
  float k_y;
  float kmin_x;
  float kmin_y;
  float k2_x;
  float k2_y;
  float mu_x;
  float mu_y;
  float lambda_x;
  float lambda_y;

  Vector2f u_xi;
  Matrix2f f_1;
  Vector2f f_2;
  Vector2f f_xi;
  Matrix2f g_xi;
  Vector2f lambda_e_xi_dot;
  Vector2f ua_xi;

  std::vector<double> x_d_list;
  std::vector<double> y_d_list;
  std::vector<double> starting_list;
  std::vector<double> xdot_d_list;
  std::vector<double> ydot_d_list;
  //std::vector<double> x_d_last_list;

  std_msgs::Float64 right_thruster;
  std_msgs::Float64 left_thruster;

  std_msgs::Float64 x_gain;
  std_msgs::Float64 y_gain;

  std_msgs::Float64 x_error;
  std_msgs::Float64 y_error;

  std_msgs::Float64 x_sigma;
  std_msgs::Float64 y_sigma;

  geometry_msgs::Pose2D ctrl_input;
  geometry_msgs::Pose2D target;

  AdaptiveSlidingModeControl()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1);
    left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1);
    x_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/x_gain", 1);
    x_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/x_error", 1);
    x_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/x_sigma", 1);
    y_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/y_sigma", 1);
    y_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/y_gain", 1);
    y_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/y_error", 1);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 1);
    target_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/target", 1);

    //ROS Subscribers
    desired_trajectory_sub = n.subscribe("/mission/trajectory", 1, &AdaptiveSlidingModeControl::desiredTrajCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1, &AdaptiveSlidingModeControl::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1, &AdaptiveSlidingModeControl::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1, &AdaptiveSlidingModeControl::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1, &AdaptiveSlidingModeControl::arduinoCallback, this);

    static const float dk_x = 0.1;
    static const float dk_y = 0.1;
    static const float dkmin_x = 0.001;
    static const float dkmin_y = 0.001;
    static const float dk2_x = 0.001;
    static const float dk2_y = 0.001;
    static const float dmu_x = 0.001;
    static const float dmu_y = 0.001;
    static const float dlambda_x = 1.0;
    static const float dlambda_y = 1.0;

    n.param("/tracking_control_asmc/k_x", k_x, dk_x);
    n.param("/tracking_control_asmc/k_y", k_y, dk_y);
    n.param("/tracking_control_asmc/kmin_x", kmin_x, dkmin_x);
    n.param("/tracking_control_asmc/kmin_y", kmin_y, dkmin_y);
    n.param("/tracking_control_asmc/k2_x", k2_x, dk2_x);
    n.param("/tracking_control_asmc/k2_y", k2_y, dk2_y);
    n.param("/tracking_control_asmc/mu_x", mu_x, dmu_x);
    n.param("/tracking_control_asmc/mu_y", mu_y, dmu_y);
    n.param("/tracking_control_asmc/lambda_x", lambda_x, dlambda_x);
    n.param("/tracking_control_asmc/lambda_y", lambda_y, dlambda_y);

    g_u = (1 / (m - X_u_dot));
    g_r = (1 / (Iz - N_r_dot));
    x_d = 0;
    y_d = 0;
    testing = 0;
    arduino = 0;
    starting = 0;
    past_psi_d = 0;
    traj_counter = 0;
    x_d_list.clear();
    y_d_list.clear();
    starting_list.clear();
    xdot_d_list.clear();
    ydot_d_list.clear();
    leng = 0;
  }

  void desiredTrajCallback(const usv_control::trajectory::ConstPtr& _pd)
  {
    traj_counter = 0;
    x_d_list.clear();
    y_d_list.clear();
    starting_list.clear();
    xdot_d_list.clear();
    ydot_d_list.clear();
    leng = _pd->len;
    for (int i=0; i<(leng); i++)
    {
        x_d_list.push_back(_pd -> trajectory[i].x);
        y_d_list.push_back(_pd -> trajectory[i].y);
        starting_list.push_back(_pd -> trajectory[i].z);
        xdot_d_list.push_back(_pd -> trajectory_dot[i].x);
        ydot_d_list.push_back(_pd -> trajectory_dot[i].y);
    }
    //ROS_INFO_STREAM("callback in");
  }

  void insCallback(const geometry_msgs::Pose2D::ConstPtr& _ins)
  {
    x = _ins -> x;
    y = _ins -> y;
    psi = _ins -> theta;
  }

  void velocityCallback(const geometry_msgs::Vector3::ConstPtr& _vel)
  {
    u = _vel -> x;
    v = _vel -> y;
    r = _vel -> z;
  }

  void flagCallback(const std_msgs::UInt8::ConstPtr& _flag)
  {
    testing = _flag -> data;
  }

  void arduinoCallback(const std_msgs::UInt8::ConstPtr& _ardu)
  {
    arduino = _ardu -> data;
  }

  void control()
  {
    if (testing == 1 && arduino == 1){
      Xu = -25;
      Xuu = 0;
      if (std::abs(u) > 1.2){
        Xu = 64.55;
        Xuu = -70.92;
      }

      Yv = (-2840)*std::abs(v);
      Nr = (-0.52)*pow(pow(u,2) + pow(v,2),0.5);

      f_u = (((m - Y_v_dot)*v*r + (Xuu*std::abs(u)*u + Xu*u)) / (m - X_u_dot));
      f_v = (((-m + X_u_dot)*u*r + (Yvv*std::abs(v)*v + Yv*v)) / (m - Y_v_dot));
      f_r = (((-X_u_dot + Y_v_dot)*u*v + (Nrr*std::abs(r)*r + Nr*r)) / (Iz - N_r_dot));

      if (leng > 0){
        //ROS_INFO_STREAM("leng larger than zero " << x_d_list[0] << endl);
        x_d = x_d_list[traj_counter];
        y_d = y_d_list[traj_counter];
        starting = starting_list[traj_counter];
        xdot_d = xdot_d_list[traj_counter];
        ydot_d = ydot_d_list[traj_counter];
        psi_d = std::atan2(ydot_d, xdot_d);
        if (traj_counter < (leng-1)){
        traj_counter = traj_counter + 1;
        }
        //ROS_INFO_STREAM("traj counter " << traj_counter << endl);
      }
      else{
        x_d = 0.0;
        y_d = 0.0;
        starting = 0;
        xdot_d = 0.0;
        ydot_d = 0.0;
      }

      e_x = (x_d + l*cos(psi_d)) - (x + l*cos(psi));
      e_y = (y_d + l*sin(psi_d)) - (y + l*sin(psi));

      psi_d_dif = psi_d - past_psi_d;
      if (std::abs(psi_d_dif) > 3.141592){
          psi_d_dif = (psi_d_dif/std::abs(psi_d_dif))*(std::abs(psi_d_dif) - 2*3.141592);
      }
      r_d = (psi_d_dif)/integral_step;
      e_x_dot = xdot_d - l*sin(psi_d)*r_d - (u*cos(psi) - v*sin(psi) - l*sin(psi)*r);
      e_y_dot = ydot_d + l*cos(psi_d)*r_d - (u*sin(psi) + v*cos(psi) + l*cos(psi)*r);
      past_psi_d = psi_d;

      s_x = e_x_dot + lambda_x*e_x;
      s_y = e_y_dot + lambda_y*e_y;

      if (Ka_x > kmin_x){
          float signvar = std::abs(s_x) - mu_x;
          if (signvar == 0){
            sign_x_sm = 0;
          }
          else {
            sign_x_sm = copysign(1,signvar);
          }
          Ka_dot_x = k_x * sign_x_sm;
      }
      else{
        Ka_dot_x = kmin_x;
      } 

      Ka_x = (integral_step)*(Ka_dot_x + Ka_dot_last_x)/2 + Ka_x; //integral to get the x adaptative gain
      Ka_dot_last_x = Ka_dot_x;

      if (Ka_y > kmin_y){
          float signvar = std::abs(s_y) - mu_y;
          if (signvar == 0){
            sign_y_sm = 0;
          }
          else {
            sign_y_sm = copysign(1,signvar);
          }
          Ka_dot_y = k_y * sign_y_sm;
      }
      else{
        Ka_dot_y = kmin_y;
      }

      Ka_y = (integral_step)*(Ka_dot_y + Ka_dot_last_y)/2 + Ka_y; //integral to get the y adaptative gain
      Ka_dot_last_y = Ka_dot_y;

      if (s_x == 0){
        sign_sx = 0;
      }
      else {
        sign_sx = copysign(1,s_x);
      }
      ua_x = ((-Ka_x) * pow(std::abs(s_x),0.5) * sign_sx) - (k2_x*s_x);

      if (s_y == 0){
        sign_sy = 0;
      }
      else {
        sign_sy = copysign(1,s_y);
      }
      ua_y = ((-Ka_y) * pow(std::abs(s_y),0.5) * sign_sy) - (k2_y*s_y);

      ua_xi << ua_x,
              ua_y;

      lambda_e_xi_dot << lambda_x * e_x,
                       lambda_y * e_y;

      g_xi << g_u*cos(psi), -g_r*sin(psi),
           g_u*sin(psi), g_r*cos(psi);

      f_1 << cos(psi), sin(psi),
           sin(psi), cos(psi);

      f_2 << f_u - v*r - l*r*r,
           u*r + f_v + l*f_r;
      
      f_xi << f_1 * f_2;

      u_xi << g_xi.inverse()*(lambda_e_xi_dot - ua_xi - f_xi);

      Tx = u_xi(0); //surge force
      Tz = u_xi(1); //yaw rate moment
      
      if (Tx > 73){
        Tx = 73;
      }
      else if (Tx < -60){
        Tx = -60;
      }
      if (Tz > 14){
        Tz = 14;
      }
      else if (Tz < -14){
        Tz = -14;
      }
      
      if (starting == 0){
        Tx = 0;
        Tz = 0;
        Ka_x = kmin_x;
        Ka_dot_last_x = 0;
        Ka_y = kmin_y;
        Ka_dot_last_y = 0;
        past_psi_d = 0;
        traj_counter = 0;
        leng = 0;
        x_d_list.clear();
        y_d_list.clear();
        starting_list.clear();
        xdot_d_list.clear();
        ydot_d_list.clear();
      }

      port_t = (Tx / 2) + (Tz / B);
      starboard_t = (Tx / (2*c)) - (Tz / (B*c));

      if (starboard_t > 36.5){
        starboard_t = 36.5;
      }
      else if (starboard_t < -30){
        starboard_t = -30;
      }
      if (port_t > 36.5){
        port_t = 36.5;
      }
      else if (port_t < -30){
        port_t = -30;
      }

      //Data publishing
      right_thruster.data = starboard_t;
      left_thruster.data = port_t;

      x_gain.data = Ka_x;
      y_gain.data = Ka_y;

      x_error.data = e_x;
      y_error.data = e_y;

      x_sigma.data = s_x;
      y_sigma.data = s_y;
    
      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      target.x = x_d;
      target.y = y_d;

      right_thruster_pub.publish(right_thruster);
      left_thruster_pub.publish(left_thruster);

      x_gain_pub.publish(x_gain);
      x_error_pub.publish(x_error);
      x_sigma_pub.publish(x_sigma);
      y_gain_pub.publish(y_gain);
      y_error_pub.publish(y_error);
      y_sigma_pub.publish(y_sigma);

      control_input_pub.publish(ctrl_input);
      target_pub.publish(target);
    }
  }

private:
  ros::NodeHandle n;

  ros::Publisher right_thruster_pub;
  ros::Publisher left_thruster_pub;
  ros::Publisher control_input_pub;
  ros::Publisher x_gain_pub;
  ros::Publisher x_error_pub;
  ros::Publisher x_sigma_pub;
  ros::Publisher y_sigma_pub;
  ros::Publisher y_gain_pub;
  ros::Publisher y_error_pub;
  ros::Publisher target_pub;

  ros::Subscriber desired_trajectory_sub;
  ros::Subscriber ins_pose_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tracking_control_asmc");
  AdaptiveSlidingModeControl adaptiveSlidingModeControl;
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    adaptiveSlidingModeControl.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}