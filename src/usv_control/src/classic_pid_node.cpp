#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <chrono>

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

        //Timer
        rclcpp::TimerBase::SharedPtr timer;


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

        void control_loop(){
            std_msgs::msg::Float64 left_msg;
            std_msgs::msg::Float64 right_msg;
            left_msg.data = 0.0;
            right_msg.data = 0.0;
            publisher_left->publish(left_msg);
            publisher_right->publish(right_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "psi: %.3f | u: %.3f | r: %.3f | psi_d: %.3f | u_d: %.3f", psi, u, r, psi_d, u_d);
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClassicPIDNode>());
  rclcpp::shutdown();
  return 0;
}