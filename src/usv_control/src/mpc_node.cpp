#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <algorithm>
#include <cstdio>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <chrono>
#include <stack>
#include <cmath>
#include <string> 
#include <fatrop/fatrop.hpp>
#include <optional>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "usv_interfaces/msg/waypoint_list.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

struct Wp {
  double x, y, theta;
};

struct MPC_State {
  double x{0.}, y{0.}, yaw{0.};
  double u{0.1}, r{0.1};
  double tp{0.}, ts{0.}, ds{0.};
  std::array<double, 10> obs{0., 0., 
    0., 0., 0., 0.};

  std::array<double, 15>
    get_state_vector() const {
    return {x, y, yaw, 
            u, r, obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8], obs[9]};
    }
};

class MPCNode : public rclcpp::Node {
public:
  MPCNode() : Node("mpc_node") {

    this->declare_parameter("path_tracking", rclcpp::PARAMETER_DOUBLE_ARRAY);
    path_tracking_weights = this->get_parameter("path_tracking").as_double_array();
    this->declare_parameter("speed", rclcpp::PARAMETER_DOUBLE_ARRAY);
    speed_weights = this->get_parameter("speed").as_double_array();
    this->declare_parameter("avoidance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    avoidance_weights = this->get_parameter("avoidance").as_double_array();
    this->declare_parameter("dyn_avoidance", rclcpp::PARAMETER_DOUBLE_ARRAY);
    dyn_avoidance_weights = this->get_parameter("dyn_avoidance").as_double_array();
    
    auto package_share_directory = boost::filesystem::path(
        ament_index_cpp::get_package_share_directory("usv_control"));
    std::string fatrop_shared_library_path =
        (package_share_directory / boost::filesystem::path("config/code_gen/casadi_codegen.so"))
            .string();

    std::string fatrop_json_path =
        (package_share_directory /
         boost::filesystem::path("config/code_gen/casadi_codegen.json"))
            .string();
    app_ = std::make_unique<fatrop::StageOCPApplication>(
        fatrop::StageOCPApplicationFactory::from_rockit_interface(
            fatrop_shared_library_path, fatrop_json_path));

    RCLCPP_INFO(this->get_logger(), "Loaded app from %s and %s",
                fatrop_shared_library_path.c_str(), fatrop_json_path.c_str());

    RCLCPP_INFO(this->get_logger(), "Available parameter names: %s",
                boost::algorithm::join(app_->parameter_names(), ", ").c_str());

    RCLCPP_INFO(
        this->get_logger(), "Available expression names: %s",
        boost::algorithm::join(app_->stage_expression_names(), ", ").c_str());

    // app_->optimize();
    app_->set_option("tol", 1e-3);
    app_->set_option("mu_init", 1e-3);
    app_->set_option("bound_push", 1e-7); // all *_bound_push variables
    app_->set_option("warm_start_mult_bound_push", 1e-7);
    app_->set_option("accept_every_trial_step", false);
    app_->set_option("warm_start_init_point", true);
    app_->set_option("print_level", 0);
    RCLCPP_INFO(this->get_logger(), "Initialized app");

    state_ = MPC_State();

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "usv/state/velocity", 10, [this](const geometry_msgs::msg::Vector3 &msg) {
          if(std::fabs(msg.x < 100)){
          state_->u = msg.x;
          // state_->v = msg.y;
          state_->r = msg.z;
          }
        });

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "usv/state/pose", 10,
        [this](const geometry_msgs::msg::Pose2D &msg) {
          pose = msg;
          state_->x = msg.x;
          state_->y = msg.y;
          state_->yaw = msg.theta;
        });

    left_thruster_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "usv/left_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { 
          state_->tp = msg.data; 
        });

    right_thruster_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "usv/right_thruster", 10,
        [this](const std_msgs::msg::Float64 &msg) { 
          state_->ts = msg.data; 
          });

    ref_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/usv/current_path_ref", 10,
        [this](const nav_msgs::msg::Path &msg) { 
          base_wp.x = msg.poses[0].pose.position.x;
          base_wp.y = msg.poses[0].pose.position.y;
          next_wp.x = msg.poses[1].pose.position.x;
          next_wp.y = msg.poses[1].pose.position.y;

          psi_d = atan2(next_wp.y - base_wp.y, next_wp.x - base_wp.x);
          });

    obstacle_list_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
        "/obj_n_nearest_list", 10,
        [this](const usv_interfaces::msg::ObjectList &msg){
          int n_size = 3;
          for(int i = 0 ; i < n_size; i++){
            obs_arr[i*2] = msg.obj_list[i].x;
            obs_arr[i*2+1] = msg.obj_list[i].y;
            dobs_arr[i*2] = msg.obj_list[i].v_x;
            dobs_arr[i*2+1] = msg.obj_list[i].v_y;
           }
           state_->obs = obs_arr;
         });

    mission_id_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/usv/mission/id", 10,
        [this](const std_msgs::msg::Int8 &msg){
          switch(msg.data){
            case 3:
              primary_weights = speed_weights;
              // primary_weights = path_tracking_weights;
              secondary_weights = path_tracking_weights;
              break;
            case 4:
              primary_weights = speed_weights;
              secondary_weights = dyn_avoidance_weights;
              break;
            default:
              primary_weights = speed_weights;
              // primary_weights = path_tracking_weights;
              // secondary_weights = avoidance_weights;
              secondary_weights = dyn_avoidance_weights;
              break;
          }
        });

    ang_vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_angular_velocity", 10);

    vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_velocity", 10);

    heading_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_heading", 10);

    ang_vel_setpoint_unfiltered_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_angular_velocity_unfiltered", 10);

    vel_setpoint_unfiltered_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_velocity_unfiltered", 10);

    heading_setpoint_unfiltered_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/guidance/desired_heading_unfiltered", 10);

    ye_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/mpc/debug/ye", 10);

    psie_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/mpc/debug/psie", 10);

    obs_cost_debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/mpc/debug/obs_cost", 10);

    qs_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/mpc/debug/qs", 10);

    mpc_projection_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/mpc/projection_path", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&MPCNode::update, this));

    primary_weights = path_tracking_weights;
    // secondary_weights = path_tracking_weights;
    secondary_weights = avoidance_weights;
    
  }

private:
  std::unique_ptr<fatrop::StageOCPApplication> app_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_thruster_sub_, right_thruster_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_id_sub_;
  rclcpp::Subscription<usv_interfaces::msg::WaypointList>::SharedPtr goals_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_sub_;
  rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr obstacle_list_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_vel_setpoint_pub_,vel_setpoint_pub_, heading_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_vel_setpoint_unfiltered_pub_,vel_setpoint_unfiltered_pub_, heading_setpoint_unfiltered_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ye_debug_pub_, psie_debug_pub_, obs_cost_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr qs_debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_projection_path_pub_;

  std_msgs::msg::Float64 ang_vel_setpoint_msg, vel_setpoint_msg, heading_setpoint_msg;
  std_msgs::msg::Float64 ang_vel_setpoint_unfiltered_msg, vel_setpoint_unfiltered_msg, heading_setpoint_unfiltered_msg;
  geometry_msgs::msg::PoseStamped pose_stamped_tmp_;

  std_msgs::msg::Float64 ye_debug_msg, psie_debug_msg, obs_cost_debug_msg;
  std_msgs::msg::Float64MultiArray qs_debug_msg;
  nav_msgs::msg::Path mpc_projection_path_msg;

  geometry_msgs::msg::Pose2D pose;

  std::optional<MPC_State> state_;

  std::vector<Wp> wp_vec;
  std::array<double, 10> obs_arr{1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.};
  std::array<double, 10> dobs_arr{0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

  // xe, ye, psie, u, r, ds
  std::vector<double> path_tracking_weights         {100., 300., 100., 1000., 10., 0.};
  std::vector<double> speed_weights                 {100., 300., 100., 1., 1., 0.};
  std::vector<double> avoidance_weights             {100., 70., 20., 1000., 10., 1.5};
  // std::vector<double> dyn_avoidance_weights         {100., 40., 25., 1000., 10., 1.};
  std::vector<double> dyn_avoidance_weights         {100., 70., 20., 0., 10., 1.};
  std::vector<double> primary_weights;
  std::vector<double> secondary_weights;

  int wp_i{0};

  Wp next_wp{0., 0.}, base_wp{0., 0.};

  double last_u{0.}, last_r{0.}, last_last_r{0.}, last_psi{0.};
  // The closer to 1, the slower it will reach the actual value.
  double alpha_u{0.9}, alpha_r{0.9}, alpha_psi{0.9};
  // double alpha_u{0.99}, alpha_r{0.94}, alpha_psi{0.9};
  // double alpha_u{0.}, alpha_r{0.}, alpha_psi{0.};

  double psi_d{0.};

  double integral_step{0.01};

  void update() {
    if (!state_.has_value()) {
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "debug0");
    // RCLCPP_INFO(this->get_logger(), "state: %f,%f,%f,%f,%f,%f,%f", 
    //   state_->x,state_->y,state_->yaw,state_->u,state_->r,
    //   state_->tp,state_->ts);

    auto initial_state = app_->get_parameter_setter("X_0");
    initial_state.set_value(state_->get_state_vector().data());

    auto target_setter = app_->get_parameter_setter("target");
    target_setter.set_value({base_wp.x, base_wp.y, next_wp.x, next_wp.y}); 

    auto psi_d_setter = app_->get_parameter_setter("psi_d");
    psi_d_setter.set_value({psi_d});    

    auto dobs_setter = app_->get_parameter_setter("dobs");
    dobs_setter.set_value(dobs_arr.data());

    std::vector<double> desired_weights = weight_calculator(obj_dist(obs_arr[0], obs_arr[1], pose));

    qs_debug_msg.data.clear();
    for(int i = 0 ; i < desired_weights.size() ; i++){
      qs_debug_msg.data.push_back(desired_weights[i]);
    }
    // qs_setter.set_value(avoidance_weights.data());

    auto qs_setter = app_->get_parameter_setter("qs");
    qs_setter.set_value(desired_weights.data());

    app_->optimize();

    auto eval_x = app_->get_expression("state_x1").at_tk(1);
    std::vector<double> x_result(1);
    auto eval_y = app_->get_expression("state_x2").at_tk(1);
    std::vector<double> y_result(1);
    auto eval_psi = app_->get_expression("state_x3").at_tk(1);
    std::vector<double> psi_result(1);
    auto eval_u = app_->get_expression("state_x4").at_tk(1);
    std::vector<double> u_result(1);
    auto eval_r = app_->get_expression("state_x5").at_tk(1);
    std::vector<double> r_result(1);

    auto eval_xe = app_->get_expression("xe").at_tk(1);
    std::vector<double> xe_result(1);
    auto eval_ye = app_->get_expression("ye").at_tk(1);
    std::vector<double> ye_result(1);
    auto eval_psie = app_->get_expression("psie").at_tk(1);
    std::vector<double> psie_result(1);
    auto eval_gamma_p = app_->get_expression("gamma_p").at_tk(1);
    std::vector<double> gamma_p_result(1);
    auto eval_obs_cost = app_->get_expression("obs_cost").at_tk(1);
    std::vector<double> obs_cost_result(1);

    auto eval_nhor = app_->get_expression("n_horizon").at_tk(1);
    std::vector<double> nhor_result(1);

    app_->last_solution().evaluate(eval_x, x_result);
    app_->last_solution().evaluate(eval_y, y_result);
    app_->last_solution().evaluate(eval_psi, psi_result);
    app_->last_solution().evaluate(eval_u, u_result);
    app_->last_solution().evaluate(eval_r, r_result);
    app_->last_solution().evaluate(eval_xe, xe_result);
    app_->last_solution().evaluate(eval_ye, ye_result);
    app_->last_solution().evaluate(eval_psie, psie_result);
    app_->last_solution().evaluate(eval_gamma_p, gamma_p_result);
    app_->last_solution().evaluate(eval_obs_cost, obs_cost_result);
    app_->last_solution().evaluate(eval_nhor, nhor_result);

    // Get Projection Path
    int projection_n = nhor_result[0];

    mpc_projection_path_msg.header.frame_id = "world";
    mpc_projection_path_msg.header.stamp = MPCNode::get_clock()->now();
    mpc_projection_path_msg.poses.clear();

    if(projection_n > 0){
      for(int i = 0 ; i < projection_n ; i++){
        // Get projected states
        auto eval_x_path = app_->get_expression("state_x1").at_tk(i+1);
        std::vector<double> x_result_path(1);
        auto eval_y_path = app_->get_expression("state_x2").at_tk(i+1);
        std::vector<double> y_result_path(1);
        app_->last_solution().evaluate(eval_x_path, x_result_path);
        app_->last_solution().evaluate(eval_y_path, y_result_path);

        // Fill ros msg
        geometry_msgs::msg::PoseStamped tmp_pose_stamped;
        tmp_pose_stamped.pose.position.x = x_result_path[0];
        tmp_pose_stamped.pose.position.y = y_result_path[0];
        mpc_projection_path_msg.poses.push_back(tmp_pose_stamped);
      }
    }

    // app_->last_solution().evaluate(eval_r, r_result);

    // RCLCPP_ERROR(this->get_logger(), "ye: %f, psie: %f, gamma_p: %f", 
    //   ye_result[0], psie_result[0], gamma_p_result[0]);
    // RCLCPP_ERROR(this->get_logger(), "qs_e: %f", obj_dist(obs_arr[0], obs_arr[1], pose));

    last_u = alpha_u*last_u + (1-alpha_u)*u_result[0];
    last_last_r = last_r;
    last_r = alpha_r*last_r + (1-alpha_r)*r_result[0];

    last_psi = integral_step * (last_r + last_last_r) / 2. + last_psi;
    // last_psi = alpha_psi*last_psi + (1-alpha_psi)*psi_result[0];
    last_psi = normalize_angle(last_psi);

    vel_setpoint_msg.data = last_u;
    ang_vel_setpoint_msg.data = last_r;
    heading_setpoint_msg.data = last_psi;
    vel_setpoint_unfiltered_msg.data = u_result[0];
    ang_vel_setpoint_unfiltered_msg.data = r_result[0];
    heading_setpoint_unfiltered_msg.data = psi_result[0];
    // RCLCPP_INFO(this->get_logger(), "setpoint: %f, %f, %f, %f, %f", 
    //             x_result[0], y_result[0], psi_result[0], u_result[0], r_result[0]);

    app_->set_initial(app_->last_solution());

    ang_vel_setpoint_pub_->publish(ang_vel_setpoint_msg);
    vel_setpoint_pub_->publish(vel_setpoint_msg);
    heading_setpoint_pub_->publish(heading_setpoint_msg);
    ang_vel_setpoint_unfiltered_pub_->publish(ang_vel_setpoint_unfiltered_msg);
    vel_setpoint_unfiltered_pub_->publish(vel_setpoint_unfiltered_msg);
    heading_setpoint_unfiltered_pub_->publish(heading_setpoint_unfiltered_msg);

    ye_debug_msg.data = ye_result[0];
    psie_debug_msg.data = psie_result[0];
    obs_cost_debug_msg.data = obs_cost_result[0];
    
    ye_debug_pub_->publish(ye_debug_msg);
    psie_debug_pub_->publish(psie_debug_msg);
    qs_debug_pub_->publish(qs_debug_msg);
    obs_cost_debug_pub_->publish(obs_cost_debug_msg);
    mpc_projection_path_pub_->publish(mpc_projection_path_msg);
  }

  double obj_dist(double obj_x, double obj_y, geometry_msgs::msg::Pose2D p){
    return sqrt((obj_x-p.x)*(obj_x-p.x) + (obj_y-p.y)*(obj_y-p.y));
  }

  // xe, ye, psi, u, r, ds
  std::vector<double> weight_calculator(double dist){

    // When dist <= 2.0 -> use policy #1
    // When dist >= 5.0 -> use policy #2
    // When in-between  -> use the interpolation of both
    double dist_p1{2.}, dist_p2{3.};
    double dist_sat = std::clamp(dist - dist_p1, 0.0, dist_p2-dist_p1) / (dist_p2-dist_p1);
    std::vector<double> out;
    for(int i = 0 ; i < 6 ; i++){
      out.push_back(primary_weights[i]*dist_sat + secondary_weights[i]*(1-dist_sat));
    }

    // Prueba para speed challenge
    double speed_e_sat = std::clamp(sqrt(pow(psie_debug_msg.data,2)+pow(ye_debug_msg.data,2)), 0., 0.5) / 0.5;
    // double qu_safe = 5000;
    double qu_safe = out[3]*10;
    out[3] = out[3]*(1-speed_e_sat) + qu_safe*speed_e_sat;
    
    return out;
  }

  double normalize_angle(double angle_in){
    double angle_out = std::fmod(angle_in + M_PI, 2 * M_PI);
    if(angle_out < 0){
      angle_out += 2*M_PI;
    }
    return angle_out - M_PI;
  }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}