#include <chrono>
#include <functional>
#include <memory>
#include <cassert>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>
#include <type_traits>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <fstream>

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
        arg_ = new const double *[sz_arg];
        res_ = new double *[sz_res];
        iw_ = new casadi_int[sz_iw];
        w_ = new double[sz_w];

        X_.resize(101, std::vector<double>(5));
        U_.resize(100, std::vector<double>(2));

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
        arg_[2] = time_horizon_.data();
        arg_[3] = weights_.data();

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/usv/state/odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                auto &q = msg->pose.pose.orientation;

                start_state_[0] = msg->pose.pose.position.x;
                start_state_[1] = msg->pose.pose.position.y;
                start_state_[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                start_state_[3] = msg->twist.twist.linear.x;
                start_state_[4] = msg->twist.twist.angular.z;

                odom_received_ = true;
            });

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/usv/mpc/goal", 1,
            [this](const geometry_msgs::msg::Pose2D::SharedPtr msg)
            {
                goal_state_[0] = msg->x;
                goal_state_[1] = msg->y;
                goal_state_[2] = msg->theta;
                goal_state_[3] = 0.0;
                goal_state_[4] = 0.0;

                goal_received_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal received: x=%.2f, y=%.2f, theta=%.2f",
                            msg->x, msg->y, msg->theta);
            });

        // Goal as a PoseStamped msg (for RViz)
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                auto &q = msg->pose.orientation;

                goal_state_[0] = msg->pose.position.x;
                goal_state_[1] = msg->pose.position.y;
                goal_state_[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                goal_state_[3] = 0.0;
                goal_state_[4] = 0.0;

                goal_received_ = true;
            });

        time_horizon_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/usv/mpc/time_horizon", 1,
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                time_horizon_[0] = msg->data;
            });

        heading_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_heading", 10);

        ang_vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_angular_velocity", 10);

        vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_velocity", 10);

        // Publisher for MPC path visualization (NEW)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/usv/mpc/trajectory", 10);

        // Timer for MPC update loop
        timer_ = this->create_wall_timer(20ms, std::bind(&MpcNode::update, this));

        RCLCPP_INFO(this->get_logger(), "MPC Node initialized. Waiting for odometry and goal...");
    }

    ~MpcNode()
    {
        // Clean up CasADi memory
        if (mem_ >= 0)
        {
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

        // ULTRADEBUGGING
        static size_t peak_rss = 0;

        // Get current RSS (Resident Set Size)
        std::ifstream stat_stream("/proc/self/status");
        std::string line;
        size_t current_rss = 0;

        while (std::getline(stat_stream, line))
        {
            if (line.find("VmRSS:") == 0)
            {
                std::istringstream iss(line);
                std::string key;
                iss >> key >> current_rss;
                break;
            }
        }

        if (current_rss > peak_rss)
        {
            peak_rss = current_rss;
            RCLCPP_WARN(this->get_logger(), "Memory increased to %zu KB", current_rss);
        }

        // Check if we have required data
        if (!odom_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for odometry...");
            return;
        }

        if (!goal_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for goal...");
            return;
        }

        // Solve MPC problem

        static int solve_count = 0;
        solve_count++;

        // Reset memory every 100 solves (every 2 seconds at 50 Hz)
        if (solve_count % 100 == 0)
        {
            if (mem_ >= 0)
            {
                opti_func_release(mem_);
            }
            mem_ = opti_func_checkout();
            RCLCPP_INFO(this->get_logger(), "Reset CasADi memory (solve #%d)", solve_count);
        }

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

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";

        for (int k = 0; k < K; k++)
        {
            // Unpack state X
            for (int i = 0; i < nx; i++)
            {
                X_[k][i] = res_buffer_[idx++];
            }

            // Populate the Path message
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = X_[k][0]; // x position
            pose_stamped.pose.position.y = X_[k][1]; // y position
            pose_stamped.pose.position.z = 0.0;

            // Optional: Set orientation (from state X_[k][2] - yaw)
            tf2::Quaternion q;
            q.setRPY(0, 0, X_[k][2]); // Assuming X_[k][2] is yaw
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
                    U_[k][i] = res_buffer_[idx++];
                }
            }
        }

        // Publish the Path message (NEW)
        path_pub_->publish(path_msg);

        // Apply first control (obtained from sol. state) action (MPC receding horizon)
        u_sol = X_[1][3];
        last_r_sol = r_sol;
        r_sol = X_[1][4];
        last_psi_sol = psi_sol;
        psi_sol = normalize_angle(integral_step * (r_sol + last_r_sol) / 2. + last_psi_sol);

        vel_setpoint_msg.data = u_sol;
        heading_setpoint_msg.data = psi_sol;

        heading_setpoint_pub_->publish(heading_setpoint_msg);
        vel_setpoint_pub_->publish(vel_setpoint_msg);
        // ang_vel_setpoint_pub_->publish(ang_vel_setpoint_msg);

        // Log info periodically
        double pos_error = std::sqrt(
            std::pow(X_[K - 1][0] - goal_state_[0], 2) +
            std::pow(X_[K - 1][1] - goal_state_[1], 2));
        RCLCPP_INFO(this->get_logger(), "MPC Solved (%.2f ms). Traj end pos error: %.2f",
                    (double)duration.count() / 1000.0, pos_error);
    }

    double normalize_angle(double x)
    {
        x = fmod(x + M_PI, M_PI * 2);
        if (x < 0)
            x += M_PI * 2;
        return x - M_PI;
    }

    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr time_horizon_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_vel_setpoint_pub_;

    std_msgs::msg::Float64 heading_setpoint_msg, vel_setpoint_msg, ang_vel_setpoint_msg;

    // State variables
    std::vector<double> start_state_{0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> goal_state_{1.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> time_horizon_{10.0};

    // Q_pos, Q_heading, R_u, R_r, R_tp, R_ts
    std::vector<double> weights_{100.0, 10.0, 0.1, 0.1, 0.1, 0.1};

    bool odom_received_ = false;
    bool goal_received_ = false;

    // CasADi work variables
    const double **arg_ = nullptr;
    double **res_ = nullptr;
    casadi_int *iw_ = nullptr;
    double *w_ = nullptr;
    int mem_ = -1;
    std::vector<double> res_buffer_;

    std::vector<std::vector<double>> X_;
    std::vector<std::vector<double>> U_;

    double last_r_sol{0.0};
    double u_sol{0.0};
    double r_sol{0.0};
    double last_psi_sol{0.0};
    double psi_sol{0.0};
    double integral_step{0.018};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcNode>());
    rclcpp::shutdown();
    return 0;
}