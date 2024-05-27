#include <cmath>

#include "control/AITSMC.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

class AitsmcNode : public rclcpp::Node {
 public:
  AitsmcNode() : Node("AITSMC_Node") {
    using namespace std::placeholders;
    auto params = initialize_params();
    controller = AITSMC(params);

    velocitySetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/velocity", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->u_d = msg.data; });

    headingSetpointSub = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/angular_velocity", 10,
        [this](const std_msgs::msg::Float64 &msg) { this->r_d = msg.data; });

    velocitySub = this->create_subscription<geometry_msgs::msg::Vector3>(
        "in/velocity", 10, [this](const geometry_msgs::msg::Vector3 &msg) {
          this->velocity = msg;
        });

    poseSub = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "in/pose", 10,
        [this](const geometry_msgs::msg::Pose2D &msg) { this->pose = msg; });

    rightThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "output/right_thruster", 10);
    leftThrusterPub = this->create_publisher<std_msgs::msg::Float64>(
        "output/left_thruster", 10);

    bool reset_on_mode_change =
        this->declare_parameter<bool>("reset_on_mode_change", true);
    if (reset_on_mode_change) {
      modeSub = this->create_subscription<std_msgs::msg::UInt16>(
          "in/mode", 10, [this](const std_msgs::msg::UInt16 &msg) {
            // Trigger reset on change to autonomous
            // 0 -> autonomous
            if (msg.data != this->lastMode && msg.data == 0) {
              this->reset_controller();
            }
            lastMode = msg.data;
          });
    }

    speedGainPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_gain", 10);

    speedErrorPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_error", 10);

    speedSigmaPub =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_sigma", 10);

    headingSigmaPub = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_sigma", 10);

    headingGainPub = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_gain", 10);

    headingErrorPub = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_error", 10);

    txPub = this->create_publisher<std_msgs::msg::Float64>("debug/Tx", 10);
    tzPub = this->create_publisher<std_msgs::msg::Float64>("debug/Tz", 10);

    updateTimer =
        this->create_wall_timer(10ms, std::bind(&AitsmcNode::update, this));
  }

 private:
  AITSMC controller{AITSMC::defaultParams()};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocitySetpointSub,
      headingSetpointSub;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySub;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rightThrusterPub,
      leftThrusterPub, speedGainPub, speedErrorPub, speedSigmaPub,
      headingSigmaPub, headingGainPub, headingErrorPub, txPub, tzPub;

  double u_d{0}, r_d{0};
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Vector3 velocity;

  rclcpp::TimerBase::SharedPtr updateTimer;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr modeSub;
  uint16_t lastMode;

  AITSMCParams initialize_params(bool declare_parameters=true) {
    if(declare_parameters){
          auto defaultParams = AITSMC::defaultParams();
    auto params =
        std::map<std::string, double>({{"k_u", defaultParams.k_u},
                                       {"k_r", defaultParams.k_r},
                                       {"kmin_u", defaultParams.kmin_u},
                                       {"kmin_r", defaultParams.kmin_r},
                                       {"k2_u", defaultParams.k2_u},
                                       {"k2_r", defaultParams.k2_r},
                                       {"mu_u", defaultParams.mu_u},
                                       {"mu_r", defaultParams.mu_r},
                                       {"tc_u", defaultParams.tc_u},
                                       {"tc_r", defaultParams.tc_r},
                                       {"q_u", defaultParams.q_u},
                                       {"q_r", defaultParams.q_r},
                                       {"p_u", defaultParams.p_u},
                                       {"p_r", defaultParams.p_r}});

    this->declare_parameters("", params);

    }
    AITSMCParams p;
    p.k_u = this->get_parameter("k_u").as_double();
    p.k_r = this->get_parameter("k_r").as_double();
    p.kmin_u = this->get_parameter("kmin_u").as_double();
    p.kmin_r = this->get_parameter("kmin_r").as_double();
    p.k2_u = this->get_parameter("k2_u").as_double();
    p.k2_r = this->get_parameter("k2_r").as_double();
    p.mu_u = this->get_parameter("mu_u").as_double();
    p.mu_r = this->get_parameter("mu_r").as_double();
    p.tc_u = this->get_parameter("tc_u").as_double();
    p.tc_r = this->get_parameter("tc_r").as_double();
    p.q_u = this->get_parameter("q_u").as_double();
    p.q_r = this->get_parameter("q_r").as_double();
    p.p_u = this->get_parameter("p_u").as_double();
    p.p_r = this->get_parameter("p_r").as_double();
    return p;
  }

  void update() {
    vanttec::ControllerState state;
    state.u = velocity.x;
    state.v = velocity.y;
    state.r = velocity.z;
    state.psi = pose.theta;

    AITSMCSetpoint setpoint;
    setpoint.u = u_d;
    setpoint.r = r_d;
    setpoint.dot_u = 0;
    setpoint.dot_r = 0;

    auto out = controller.update(state, setpoint);
    auto debug = controller.getDebugData();

    std_msgs::msg::Float64 rt, lt, sg, hg, eu, epsi, su, sp, txMsg, tzMsg;
    // if(!(setpoint.u == 0 && setpoint.r == 0)){
      rt.data = out.right_thruster;
      lt.data = out.left_thruster;
    // } else {
    //   rt.data = 0.0;
    //   lt.data = 0.0;
    // }
    sg.data = debug.Ka_u;
    hg.data = debug.Ka_r;
    eu.data = debug.e_u;
    epsi.data = debug.e_r;
    su.data = debug.s_u;
    sp.data = debug.s_r;
    txMsg.data = debug.Tx;
    tzMsg.data = debug.Tz;

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

  void reset_controller() {
    // auto params = initialize_params(false);
    // controller = AITSMC(params);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AitsmcNode>());
  rclcpp::shutdown();
  return 0;
}