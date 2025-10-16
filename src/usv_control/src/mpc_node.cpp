#include <chrono>
#include <functional>
#include <memory>
#include <cassert>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Added for Path
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"             // Added for Path
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "../scripts/mpc/asv.h"

using namespace std::chrono_literals;

class MpcNode : public rclcpp::Node
{
public:
    MpcNode() : Node("mpc_node")
    {
        // Get CasADi function dimensions
        casadi_int n_in = opti_func_n_in();
        casadi_int n_out = opti_func_n_out();

        casadi_int sz_arg = n_in, sz_res = n_out, sz_iw = 0, sz_w = 0;
        opti_func_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

        RCLCPP_INFO(this->get_logger(), "CasADi function initialized:");
        RCLCPP_INFO(this->get_logger(), "  n_in=%lld, n_out=%lld", n_in, n_out);
        RCLCPP_INFO(this->get_logger(), "  sz_arg=%lld, sz_res=%lld, sz_iw=%lld, sz_w=%lld", 
                    sz_arg, sz_res, sz_iw, sz_w);

        // Allocate work vectors
        arg_ = new const double*[sz_arg];
        res_ = new double*[sz_res];
        iw_ = new casadi_int[sz_iw];
        w_ = new double[sz_w];

        // Allocate result buffer (MUST BE CLASS MEMBER, NOT LOCAL VARIABLE)
        res_buffer_.resize(opti_func_sparsity_out(0)[0]);
        res_[0] = res_buffer_.data();

        // Allocate CasADi memory (thread-safe)
        opti_func_incref();
        mem_ = opti_func_checkout();

        RCLCPP_INFO(this->get_logger(), "Result buffer size: %zu", res_buffer_.size());

        // Set up initial pointers
        arg_[0] = start_state_.data();
        arg_[1] = goal_state_.data();

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/usv/state/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                auto& q = msg->pose.pose.orientation;
                
                start_state_[0] = msg->pose.pose.position.x;
                start_state_[1] = msg->pose.pose.position.y;
                start_state_[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                start_state_[3] = msg->twist.twist.linear.x;
                start_state_[4] = msg->twist.twist.angular.z;
                
                odom_received_ = true;
            });

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/mpc/goal", 10, 
            [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
                goal_state_[0] = msg->x;
                goal_state_[1] = msg->y;
                goal_state_[2] = msg->theta;
                goal_state_[3] = 0.0;
                goal_state_[4] = 0.0;

                goal_received_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal received: x=%.2f, y=%.2f, theta=%.2f", 
                           msg->x, msg->y, msg->theta);
            });

        // Publisher for control commands
        control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/usv/control/cmd_vel", 10);

        // Publisher for MPC path visualization (NEW)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/mpc/trajectory", 10);

        // Timer for MPC update loop
        timer_ = this->create_wall_timer(100ms, std::bind(&MpcNode::update, this));
        
        RCLCPP_INFO(this->get_logger(), "MPC Node initialized. Waiting for odometry and goal...");
    }

    ~MpcNode()
    {
        // Clean up CasADi memory
        if (mem_ >= 0) {
            opti_func_release(mem_);
            opti_func_decref();
        }

        // Clean up work vectors
        delete[] arg_;
        delete[] res_;
        delete[] iw_;
        delete[] w_;
    }

private:
    void update()
    {
        // Check if we have required data
        if (!odom_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for odometry...");
            return;
        }

        if (!goal_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for goal...");
            return;
        }

        // Solve MPC problem
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (opti_func(arg_, res_, iw_, w_, mem_))
        {
            RCLCPP_ERROR(this->get_logger(), "MPC optimization failed!");
            return;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // Unpack the solution
        const int N = 100;
        const int K = N + 1;
        const int nx = 5;
        const int nu = 2;

        int idx = 0;
        std::vector<std::vector<double>> X(K, std::vector<double>(nx));
        std::vector<std::vector<double>> U(N, std::vector<double>(nu));

        // Create Path message (NEW)
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        for (int k = 0; k < K; k++)
        {
            // Unpack state X
            for (int i = 0; i < nx; i++)
            {
                X[k][i] = res_buffer_[idx++];
            }

            // Populate the Path message (NEW)
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = X[k][0]; // x position
            pose_stamped.pose.position.y = X[k][1]; // y position
            pose_stamped.pose.position.z = 0.0;

            // Optional: Set orientation (from state X[k][2] - yaw)
            tf2::Quaternion q;
            q.setRPY(0, 0, X[k][2]); // Assuming X[k][2] is yaw
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);


            // Unpack control U
            if (k < N)
            {
                for (int i = 0; i < nu; i++)
                {
                    U[k][i] = res_buffer_[idx++];
                }
            }
        }
        
        // Publish the Path message (NEW)
        path_pub_->publish(path_msg);

        // Apply first control action (MPC receding horizon)
        double tx = U[0][0];  // Surge thrust
        double tz = U[0][1];  // Yaw torque

        // Publish control command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = tx;   // You may need to convert thrust to velocity
        cmd_msg.angular.z = tz;  // You may need to convert torque to angular velocity
        control_pub_->publish(cmd_msg);

        // Log info periodically
        static int counter = 0;
        if (++counter % 10 == 0) {
            double pos_error = std::sqrt(
                std::pow(X[K-1][0] - goal_state_[0], 2) + 
                std::pow(X[K-1][1] - goal_state_[1], 2)
            );
            RCLCPP_INFO(this->get_logger(), "MPC Solved (%.2f ms). Traj end pos error: %.2f", 
                        (double)duration.count() / 1000.0, pos_error);
        }
    }

    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // NEW Path Publisher

    // State variables
    std::vector<double> start_state_{0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> goal_state_{1.0, 0.0, 0.0, 0.0, 0.0};
    
    bool odom_received_ = false;
    bool goal_received_ = false;

    // CasADi work variables (MUST BE CLASS MEMBERS)
    const double** arg_ = nullptr;
    double** res_ = nullptr;
    casadi_int* iw_ = nullptr;
    double* w_ = nullptr;
    int mem_ = -1;
    std::vector<double> res_buffer_;  // CRITICAL: Must be class member!
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcNode>());
    rclcpp::shutdown();
    return 0;
}