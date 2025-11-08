/*
 * ASV Spline Tracking - Closed-Loop Simulation in C
 * Replicates the Python closed-loop MPC simulation using generated ACADOS solvers
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

// ACADOS includes
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_solver_asv_dynamics.h"

// BLASFEO
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// ROS deps
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define NX ASV_DYNAMICS_NX
#define NU ASV_DYNAMICS_NU
#define NP ASV_DYNAMICS_NP
#define NBX0 ASV_DYNAMICS_NBX0
#define N_HORIZON ASV_DYNAMICS_N

// Simulation parameters
#define T_SIM 10.0          // Total simulation time [s]
#define TF 2.5              // MPC prediction horizon [s]
#define DT (TF / N_HORIZON) // Time step

using namespace std::chrono_literals;

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        using namespace std::placeholders;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/usv/state/odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                auto &q = msg->pose.pose.orientation;

                x0[0] = msg->pose.pose.position.x;
                x0[1] = msg->pose.pose.position.y;
                x0[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                x0[3] = msg->twist.twist.linear.x;
                x0[4] = msg->twist.twist.angular.z;
            });

        spline_t_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_t", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                x0[5] = msg.data;
            });

        spline_params_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mpc/spline_params", 10,
            [this](const std_msgs::msg::Float64MultiArray &msg)
            {
                for (int i = 0; i < 8; i++)
                {
                    spline_params[i] = msg.data[i];
                }
                // Update spline parameteres for all stages
                for (int i = 0; i <= N_HORIZON; i++)
                {
                    asv_dynamics_acados_update_params(ocp_capsule, i, spline_params, NP);
                }
            });

        sol_time_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/mpc/sol_time", 10);

        sol_path_pub_ =
            this->create_publisher<nav_msgs::msg::Path>("/mpc/sol_path", 10);

        heading_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_heading", 10);

        vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_velocity", 10);

        timer_ =
            this->create_wall_timer(50ms, std::bind(&MPCNode::update, this));

        sol_path_msg.header.frame_id = "world";
        sol_path_msg.poses.resize(N_HORIZON + 1);

        // === CREATE OCP SOLVER ===
        RCLCPP_INFO(this->get_logger(), "Creating OCP solver...");
        ocp_capsule = asv_dynamics_acados_create_capsule();
        status = asv_dynamics_acados_create_with_discretization(ocp_capsule, N_HORIZON, NULL);

        if (status)
        {
            RCLCPP_INFO(this->get_logger(), "OCP solver creation failed with status %d", status);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "OCP solver created successfully");
        }

        nlp_config = asv_dynamics_acados_get_nlp_config(ocp_capsule);
        nlp_dims = asv_dynamics_acados_get_nlp_dims(ocp_capsule);
        nlp_in = asv_dynamics_acados_get_nlp_in(ocp_capsule);
        nlp_out = asv_dynamics_acados_get_nlp_out(ocp_capsule);
        nlp_solver = asv_dynamics_acados_get_nlp_solver(ocp_capsule);

        // Set initial state
        memcpy(simX, x0, NX * sizeof(double));

        // Set spline parameteres for all stages
        for (int i = 0; i <= N_HORIZON; i++)
        {
            asv_dynamics_acados_update_params(ocp_capsule, i, spline_params, NP);
        }
    }

    ~MPCNode()
    {
        // === CLEANUP ===
        asv_dynamics_acados_free(ocp_capsule);
        asv_dynamics_acados_free_capsule(ocp_capsule);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sol_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        sol_time_pub_,
        vel_setpoint_pub_, heading_setpoint_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr spline_t_sub_;

    std_msgs::msg::Float64 sol_time_msg, vel_setpoint_msg, heading_setpoint_msg;
    nav_msgs::msg::Path sol_path_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    int status{0};
    double spline_params[NP];
    double x0[NX];

    asv_dynamics_solver_capsule *ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;

    double simX[NX];
    double simU[NU];

    double xtraj[NX * (N_HORIZON + 1)];

    void update()
    {
        // Update initial state
        memcpy(simX, x0, NX * sizeof(double));

        auto start_t = std::chrono::high_resolution_clock::now();

        // Set initial state constraint
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", simX);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", simX);

        // === RTI PHASE 1: PREPARATION ===
        int rti_phase = 1;
        ocp_nlp_solver_opts_set(nlp_config, ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
        status = asv_dynamics_acados_solve(ocp_capsule);

        if (status != 0 && status != 2 && status != 5)
        {
            RCLCPP_ERROR(this->get_logger(), "Warning: Preparation phase returned status %d\n", status);
        }

        // === RTI PHASE 2: FEEDBACK ===
        rti_phase = 2;
        ocp_nlp_solver_opts_set(nlp_config, ocp_capsule->nlp_opts, "rti_phase", &rti_phase);
        status = asv_dynamics_acados_solve(ocp_capsule);

        if (status != 0 && status != 2 && status != 5)
        {
            RCLCPP_ERROR(this->get_logger(), "Warning: Feedback phase returned status %d\n", status);
        }

        // Get optimal control
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", simU);

        auto end_t = std::chrono::high_resolution_clock::now();
        sol_time_msg.data = std::chrono::duration<double>(end_t - start_t).count();

        sol_path_msg.header.stamp = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped tmp_pose;
        for (int i = 0; i <= N_HORIZON; i++)
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * NX]);
            tmp_pose.pose.position.x = xtraj[i * NX];
            tmp_pose.pose.position.y = xtraj[i * NX + 1];
            sol_path_msg.poses[i] = tmp_pose;
        }
        vel_setpoint_msg.data = xtraj[10 * NX + 3];
        heading_setpoint_msg.data = xtraj[10 * NX + 2];

        int sqp_iter;
        ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
        RCLCPP_INFO(this->get_logger(), "SQP iters: %d", sqp_iter);

        sol_time_pub_->publish(sol_time_msg);
        sol_path_pub_->publish(sol_path_msg);
        vel_setpoint_pub_->publish(vel_setpoint_msg);
        heading_setpoint_pub_->publish(heading_setpoint_msg);
    }

    double point_distance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }

    // Evaluate spline at parameter t
    void evaluate_spline(double t, double spline_params[8], double result[2])
    {
        double t2 = t * t;
        double t3 = t2 * t;

        result[0] = spline_params[0] * t3 + spline_params[1] * t2 + spline_params[2] * t + spline_params[3];
        result[1] = spline_params[4] * t3 + spline_params[5] * t2 + spline_params[6] * t + spline_params[7];
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}