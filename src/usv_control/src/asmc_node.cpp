#include <algorithm>
#include <cmath>
#include <cstdio>

#include "control/ASMC.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class AsmcNode : public rclcpp::Node {
 public:
  AsmcNode() : Node("ASMC_Node") {
    using namespace std::placeholders;
    params = initialize_params();
    controller = ASMC(params);

    velocitySetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/velocity", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->u_d = msg.data; });

    headingSetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/heading", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->psi_d = msg.data; });

    pivotSetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/pivot", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->pivot_e = msg.data; });

    velocitySub = this->create_subscription<geometry_msgs::msg::Vector3>(
        "usv/state/velocity", 1, [this](const geometry_msgs::msg::Vector3 &msg) {
          this->velocity = msg;
        });

    poseSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "usv/state/pose", 1,
        [this](const geometry_msgs::msg::Pose2D &msg) { this->pose = msg; });

    arrivedSub = this->create_subscription<std_msgs::msg::Bool>(
        "usv/waypoint/arrived", 1,
        [this](const std_msgs::msg::Bool &msg) { this->wp_arrived.data = msg.data; });

    autoSub = this->create_subscription<std_msgs::msg::UInt16>(
        "usv/op_mode", 1,
        [this](const std_msgs::msg::UInt16 &msg) {
            this->auto_mode.data = msg.data; 
        });

    rightThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "usv/right_thruster", 10);
    leftThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "usv/left_thruster", 10);

    speedGainPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_gain", 10);

    speedErrorPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_error", 10);

    speedSigmaPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speedSigma", 10);

    headingSigmaPub = this->create_publisher<std_msgs::msg::Float64>(
        "debug/headingSigma", 10);

    headingGainPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/headingGain", 10);

    headingErrorPub = this->create_publisher<std_msgs::msg::Float64>(
        "debug/headingError", 10);

    txPub = this->create_publisher<std_msgs::msg::Float64>("debug/Tx", 10);
    tzPub = this->create_publisher<std_msgs::msg::Float64>("debug/Tz", 10);

    updateTimer =
        this->create_wall_timer(10ms, std::bind(&AsmcNode::update, this));
  }

 private:
  ASMCParams params;

  ASMC controller{ASMC::defaultParams()};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocitySetpointSub,
      headingSetpointSub, pivotSetpointSub;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr autoSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrivedSub;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySub;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightThrusterPub,
      leftThrusterPub, speedGainPub, speedErrorPub, speedSigmaPub,
      headingSigmaPub, headingGainPub, headingErrorPub, txPub, tzPub;

  rclcpp::TimerBase::SharedPtr updateTimer;

  double u_d{0}, psi_d{0}, pivot_e{0};
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Vector3 velocity;
  std_msgs::msg::Bool wp_arrived;
  std_msgs::msg::UInt16 auto_mode, previous_mode;

  ASMCParams initialize_params() {
    // TODO define good defaults
    auto defaultParams = ASMC::defaultParams();
    auto params = std::map<std::string, double>({{"k_u", defaultParams.k_u},
                                                 {"k_psi", defaultParams.k_psi},
                                                 {"kmin_u", defaultParams.kmin_u},
                                                 {"kmin_psi", defaultParams.kmin_psi},
                                                 {"k2_u", defaultParams.k2_u},
                                                 {"k2_psi", defaultParams.k2_psi},
                                                 {"mu_u", defaultParams.mu_u},
                                                 {"mu_psi", defaultParams.mu_psi},
                                                 {"lambda_u", defaultParams.lambda_u},
                                                 {"lambda_psi", defaultParams.lambda_psi}});
    this->declare_parameters("", params);

    ASMCParams p;
    p.k_u = this->get_parameter("k_u").as_double();
    p.k_psi = this->get_parameter("k_psi").as_double();
    p.kmin_u = this->get_parameter("kmin_u").as_double();
    p.kmin_psi = this->get_parameter("kmin_psi").as_double();
    p.k2_u = this->get_parameter("k2_u").as_double();
    p.k2_psi = this->get_parameter("k2_psi").as_double();
    p.mu_u = this->get_parameter("mu_u").as_double();
    p.mu_psi = this->get_parameter("mu_psi").as_double();
    p.lambda_u = this->get_parameter("lambda_u").as_double();
    p.lambda_psi = this->get_parameter("lambda_psi").as_double();
    return p;
  }

  void update() {
    ASMCState state;
    state.vel_x = velocity.x;
    state.vel_y = velocity.y;
    state.vel_r = velocity.z;
    state.theta = pose.theta;

    ASMCSetpoint setpoint;
    setpoint.heading_setpoint = psi_d;
    setpoint.velocity_setpoint = u_d;
    setpoint.pivot_enabled = pivot_e;

    std_msgs::msg::Float64 rt, lt, sg, hg, eu, epsi, su, sp, txMsg, tzMsg;

    ASMCOutput out;

    // If first in auton
    if(previous_mode.data != 0 && auto_mode.data == 0){
        RCLCPP_INFO(this->get_logger(), "Initializing ASMC controller.");
        controller = ASMC(params);
    }

    // If autonom
    if(auto_mode.data == 0 && !wp_arrived.data){
        out = controller.update(state, setpoint);
    // if(!wp_arrived.data){
        rt.data = out.right_thruster;
        lt.data = out.left_thruster;
    } else {
        rt.data = 0.0;
        lt.data = 0.0;
    }
    this->previous_mode = auto_mode;
    sg.data = out.speed_gain;
    hg.data = out.heading_gain;
    eu.data = out.speed_error;
    epsi.data = out.heading_error;
    su.data = out.speed_sigma;
    sp.data = out.heading_sigma;
    txMsg.data = out.Tx;
    tzMsg.data = out.Tz;

    rightThrusterPub->publish(rt);
    leftThrusterPub->publish(lt);

    speedGainPub->publish(sg);
    speedErrorPub->publish(eu);
    speedSigmaPub->publish(su);
    headingGainPub->publish(hg);
    headingErrorPub->publish(epsi);
    headingSigmaPub->publish(sp);
    txPub->publish(txMsg);
    tzPub->publish(tzMsg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsmcNode>());
  rclcpp::shutdown();
  return 0;
}
