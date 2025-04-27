#include <cmath>

#include "control/PID.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std::chrono_literals;

class PIDNode : public rclcpp::Node {
public:
  PIDNode() : Node("pid_node") {
    using namespace std::placeholders;
    auto params = initialize_params();
    controller = PID(params);

    velocity_setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/velocity", 10,
        [this](const std_msgs::msg::Float64 &msg) { u_d = msg.data; });

    angular_velocity_setpoint_sub_ =
        this->create_subscription<std_msgs::msg::Float64>(
            "setpoint/angular_velocity", 10,
            [this](const std_msgs::msg::Float64 &msg) {
              r_d_last_last = r_d_last;
              r_d_last = r_d;
              r_d = msg.data;
            });

    heading_setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "setpoint/heading", 10,
        [this](const std_msgs::msg::Float64 &msg) { 
          psi_d = msg.data;
          virgin_heading = false;
        });

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "usv/state/velocity", 10,
        [this](const geometry_msgs::msg::Vector3 &msg) {
          this->velocity = msg;
        });

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "usv/state/pose", 10,
        [this](const geometry_msgs::msg::Pose2D &msg) {
          this->pose = msg;
          if(virgin_heading){
            psi_d = msg.theta;
          }
        });

    right_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "usv/right_thruster", 10);
    left_thruster_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("usv/left_thruster", 10);

    bool reset_on_mode_change =
        this->declare_parameter<bool>("reset_on_mode_change", true);
    if (reset_on_mode_change) {
      mode_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
          "usv/op_mode", 10, [this](const std_msgs::msg::UInt16 &msg) {
            // Trigger reset on change to autonomous
            // 0 -> autonomous
            if (msg.data != this->lastMode && msg.data == 0) {
              this->reset_controller();
            }
            lastMode = msg.data;
          });
    }

    speedGain_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_gain", 10);

    speedError_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_error", 10);

    speedSigma_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("debug/speed_sigma", 10);

    headingSigma_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_sigma", 10);

    headingGain_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_gain", 10);

    headingError_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_error", 10);

    headingDotError_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_dot_error", 10);

    headingIError_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "debug/heading_i_error", 10);

    tx_pub_ = this->create_publisher<std_msgs::msg::Float64>("debug/Tx", 10);
    tz_pub_ = this->create_publisher<std_msgs::msg::Float64>("debug/Tz", 10);

    updateTimer =
        this->create_wall_timer(10ms, std::bind(&PIDNode::update, this));
  }

private:
  PID controller{PID::defaultParams()};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      velocity_setpoint_sub_,
      heading_setpoint_sub_, angular_velocity_setpoint_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thruster_pub_,
      left_thruster_pub_, speedGain_pub_, speedError_pub_, speedSigma_pub_,
      headingSigma_pub_, headingGain_pub_, headingError_pub_,
      headingDotError_pub_, headingIError_pub_, tx_pub_, tz_pub_;

  double u_d{0}, r_d{0}, psi_d{0}, r_d_last{0}, r_d_last_last{0}, rdot_d{0};
  bool virgin_heading{true};
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Vector3 velocity;

  rclcpp::TimerBase::SharedPtr updateTimer;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr mode_sub_;
  uint16_t lastMode;

  PIDParams initialize_params(bool declare_parameters = true) {
    if (declare_parameters) {
      auto defaultParams = PID::defaultParams();
      auto params = std::map<std::string, double>({
          {"pk_u", defaultParams.pk_u},
          {"i_u", defaultParams.i_u},
          {"d_u", defaultParams.d_u},
          {"pk_psi", defaultParams.pk_psi},
          {"i_psi", defaultParams.i_psi},
          {"d_psi", defaultParams.d_psi},
          {"tc_u", defaultParams.tc_u},
          {"tc_psi", defaultParams.tc_psi},
          {"q_u", defaultParams.q_u},
          {"q_psi", defaultParams.q_psi},
          {"p_u", defaultParams.p_u},
          {"p_psi", defaultParams.p_psi},
      });

      this->declare_parameters("", params);
    }
    PIDParams p;
    p.pk_u = this->get_parameter("pk_u").as_double();
    p.i_u = this->get_parameter("i_u").as_double();
    p.d_u = this->get_parameter("d_u").as_double();
    p.pk_psi = this->get_parameter("pk_psi").as_double();
    p.i_psi = this->get_parameter("i_psi").as_double();
    p.d_psi = this->get_parameter("d_psi").as_double();
    p.tc_u = this->get_parameter("tc_u").as_double();
    p.tc_psi = this->get_parameter("tc_psi").as_double();
    p.q_u = this->get_parameter("q_u").as_double();
    p.q_psi = this->get_parameter("q_psi").as_double();
    p.p_u = this->get_parameter("p_u").as_double();
    p.p_psi = this->get_parameter("p_psi").as_double();
    return p;
  }

  void update() {
    rdot_d = (1.5 * r_d - 2 * r_d_last + 0.5 * r_d_last_last) / 0.01;

    vanttec::ControllerState state;
    state.u = velocity.x;
    state.v = velocity.y;
    state.r = velocity.z;
    state.psi = pose.theta;

    PIDSetpoint setpoint;
    setpoint.u = u_d;
    // setpoint.psi_dot = r_d;
    setpoint.psi = psi_d; // Control heading (second order ss)
    // setpoint.psi = r_d; // Control yaw rate (first order ss)
    setpoint.dot_u = 0.;
    // setpoint.dot_r = rdot_d;
    setpoint.dot_r = 0.;

    auto out = controller.update(state, setpoint);
    auto debug = controller.getDebugData();

    std_msgs::msg::Float64 rt, lt, sg, hg, eu, epsi, edotpsi, eipsi, su, sp,
        txMsg, tzMsg;
    // if(!(setpoint.u == 0 && setpoint.r == 0)){
    rt.data = out.right_thruster;
    lt.data = out.left_thruster;
    // } else {
    //   rt.data = 0.0;
    //   lt.data = 0.0;
    // }
    eu.data = debug.e_u;
    epsi.data = debug.e_psi;
    edotpsi.data = debug.edot_psi;
    eipsi.data = debug.ei_psi;
    txMsg.data = debug.Tx;
    tzMsg.data = debug.Tz;

    right_thruster_pub_->publish(rt);
    left_thruster_pub_->publish(lt);

    speedError_pub_->publish(eu);
    headingError_pub_->publish(epsi);
    headingDotError_pub_->publish(edotpsi);
    headingIError_pub_->publish(eipsi);

    tx_pub_->publish(txMsg);
    tz_pub_->publish(tzMsg);
  }

  void reset_controller() {
    auto params = initialize_params(false);
    controller = PID(params);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDNode>());
  rclcpp::shutdown();
  return 0;
}
