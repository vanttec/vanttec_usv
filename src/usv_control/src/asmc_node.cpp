#include <algorithm>
#include <cmath>
#include <cstdio>

#include "control/ASMC.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class AsmcNode : public rclcpp::Node {
 public:
  AsmcNode() : Node("ASMC_Node") {
    using namespace std::placeholders;
    auto params = initialize_params();
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
        "input/velocity", 1, [this](const geometry_msgs::msg::Vector3 &msg) {
          this->velocity = msg;
        });

    poseSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "input/pose", 1,
        [this](const geometry_msgs::msg::Pose2D &msg) { this->pose = msg; });

    rightThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "output/right_thruster", 10);
    leftThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "output/left_thruster", 10);

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
  ASMC controller{ASMC::defaultParams()};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocitySetpointSub,
<<<<<<< HEAD
      headingSetpointSub;
=======
      headingSetpointSub, pivotSetpointSub;
>>>>>>> 6d598a80353d0f7243d7e25fa9a3a43d0b958e0a

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySub;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightThrusterPub,
      leftThrusterPub, speedGainPub, speedErrorPub, speedSigmaPub,
      headingSigmaPub, headingGainPub, headingErrorPub, txPub, tzPub;

  rclcpp::TimerBase::SharedPtr updateTimer;

<<<<<<< HEAD
  double u_d{0}, psi_d{0};
=======
  double u_d{0}, psi_d{0}, pivot_e{0};
>>>>>>> 6d598a80353d0f7243d7e25fa9a3a43d0b958e0a
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Vector3 velocity;

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
<<<<<<< HEAD
=======
    setpoint.pivot_enabled = pivot_e;
>>>>>>> 6d598a80353d0f7243d7e25fa9a3a43d0b958e0a

    auto out = controller.update(state, setpoint);

    std_msgs::msg::Float64 rt, lt, sg, hg, eu, epsi, su, sp, txMsg, tzMsg;
    rt.data = out.right_thruster;
    lt.data = out.left_thruster;
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
