#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include "control/ClassicPID.h"

class ClassicPIDNode : public rclcpp::Node{
    public:
        ClassicPIDNode() : Node("classic_pid_node"){
            subscription_sp_heading = this->create_subscription<std_msgs::msg::Float64>(
                "setpoint/heading", 10, std::bind(&ClassicPIDNode::sp_heading_callback, this, std::placeholders::_1));

            subscription_sp_velocity = this->create_subscription<std_msgs::msg::Float64>(
                "setpoint/velocity", 10, std::bind(&ClassicPIDNode::sp_velocity_callback, this, std::placeholders::_1));

            subscription_pose = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "usv/state/pose", 10, std::bind(&ClassicPIDNode::pose_callback, this, std::placeholders::_1));

            subscription_vel = this->create_subscription<geometry_msgs::msg::Vector3>(
                "usv/state/velocity", 10, std::bind(&ClassicPIDNode::vel_callback, this, std::placeholders::_1));

            publisher_left = this->create_publisher<std_msgs::msg::Float64>("usv/left_thruster", 10);
            publisher_right = this->create_publisher<std_msgs::msg::Float64>("usv/right_thruster", 10);

            double rate = this->declare_parameter<double>("rate",50.0);
            
            if (rate <= 0.0) {
                RCLCPP_ERROR(this->get_logger(), "rate debe ser positivo, usando 50 Hz");
                rate = 50.0;
            }

            double kp_psi = this->declare_parameter<double>("kp_psi", 4.0);
            double ki_psi = this->declare_parameter<double>("ki_psi", 0.0);
            double kd_psi = this->declare_parameter<double>("kd_psi", 0.0);

            double kp_u = this->declare_parameter<double>("kp_u", 12.0);
            double ki_u = this->declare_parameter<double>("ki_u", 0.0);
            double kd_u = this->declare_parameter<double>("kd_u", 0.0);

            double psi_out_max = this->declare_parameter<double>("psi_out_max", 8.0);
            double u_out_max = this->declare_parameter<double>("u_out_max", 12.0);
            double u_out_min = this->declare_parameter<double>("u_out_min", 0.0);

            thruster_min = this->declare_parameter<double>("thruster_min", -20.0);
            thruster_max = this->declare_parameter<double>("thruster_max", 20.0);
            state_timeout = this->declare_parameter<double>("state_timeout", 0.5);

            heading_pid = ClassicPID(kp_psi, ki_psi, kd_psi, -psi_out_max, psi_out_max);
            velocity_pid = ClassicPID(kp_u, ki_u, kd_u, u_out_min, u_out_max);

            last_loop_time = this->now();

            timer = this->create_wall_timer(
                std::chrono::duration<double>(1.0/rate),
                std::bind(&ClassicPIDNode::control_loop, this));
        }
    private:
        //Subscriptions
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_sp_heading;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_sp_velocity;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_pose;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_vel;

        //Publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_left;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_right;

        // Setpoints
        double psi_d = 0.0;
        double u_d = 0.0;

        // Actual State, from localization_node
        double psi = 0.0;
        double u = 0.0;
        double r = 0.0;

        // Flags
        bool heading_sp_received = false;
        bool pose_received = false;
        bool vel_received = false;

        //Time
        rclcpp::Time last_pose_time;
        rclcpp::Time last_vel_time;
        rclcpp::Time last_loop_time;

        //Timer
        rclcpp::TimerBase::SharedPtr timer;

        //PID
        ClassicPID heading_pid{0.0,0.0,0.0,0.0,0.0};
        ClassicPID velocity_pid{0.0,0.0,0.0,0.0,0.0};

        double thruster_min = 0.0;
        double thruster_max = 0.0;
        double state_timeout = 0.0;

        void sp_heading_callback(const std_msgs::msg::Float64::SharedPtr msg){
            psi_d = msg->data;
            heading_sp_received = true;
        }

        void sp_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg){
            u_d = msg->data;
        }

        void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
            psi = msg->theta;
            last_pose_time = this->now();
            pose_received = true;

            if (!heading_sp_received){
                psi_d = psi;
            }
        }
        void vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
            u = msg->x;
            r = msg->z;
            last_vel_time = this->now();
            vel_received = true;
        }

        void publish_thrust(double left, double right){
            std_msgs::msg::Float64 left_msg;
            std_msgs::msg::Float64 right_msg;
            left_msg.data = left;
            right_msg.data = right;
            publisher_left->publish(left_msg);
            publisher_right->publish(right_msg);
        }

        void control_loop(){

            rclcpp::Time now = this->now();
            double dt = (now - last_loop_time).seconds();
            last_loop_time = now;

            if(!pose_received || !vel_received){
                publish_thrust(0.0,0.0);
                heading_pid.reset();
                velocity_pid.reset();
                return;
            }
            double pose_age = (now - last_pose_time).seconds();
            double vel_age = (now - last_vel_time).seconds();

            if(pose_age > state_timeout || vel_age > state_timeout){
                publish_thrust(0.0,0.0);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "pose_age: %.3f | vel_age: %.3f", pose_age, vel_age);
                heading_pid.reset();
                velocity_pid.reset();
                return; 
            }

            double e_psi = std::atan2(std::sin(psi_d - psi),std::cos(psi_d - psi));
            double e_u = u_d - u;

            double t_turn = heading_pid.compute(e_psi,dt,-r);
            double t_forward = velocity_pid.compute(e_u,dt);

            double left = t_forward + t_turn;
            double right = t_forward - t_turn;

            double left_cmd = std::clamp(left, thruster_min, thruster_max);
            double right_cmd = std::clamp(right, thruster_min, thruster_max);
            publish_thrust(left_cmd, right_cmd);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "psi: %.3f | u: %.3f | r: %.3f | psi_d: %.3f | u_d: %.3f | e_psi: %.3f | e_u: %.3f | t_forward: %.3f | t_turn: %.3f | left: %.3f | right: %.3f | left_cmd: %.3f | right_cmd: %.3f", 
            psi, u, r, psi_d, u_d, e_psi, e_u, t_forward, t_turn, left, right, left_cmd, right_cmd);
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClassicPIDNode>());
  rclcpp::shutdown();
  return 0;
}