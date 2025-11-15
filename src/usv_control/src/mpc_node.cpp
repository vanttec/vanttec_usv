/*
 * ASV Spline Tracking - Closed-Loop Simulation in C
 * Replicates the Python closed-loop MPC simulation using generated ACADOS solvers
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <memory>

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
#define N_HORIZON ASV_DYNAMICS_N

// Simulation parameters
#define N_SP 8 // Spline params (4 x NDIMS)
#define N_WP 7 // Weight params

#define TF 2.5              // MPC prediction horizon [s]
#define DT (TF / N_HORIZON) // Time step

using namespace std::chrono_literals;

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        using namespace std::placeholders;

        // === PARAMETERS ===
        this->declare_parameter("mpc_tf", mpc_tf);
        mpc_tf = this->get_parameter("mpc_tf").as_double();

        this->declare_parameter("mpc_weights", mpc_weights);
        mpc_weights = this->get_parameter("mpc_weights").as_double_array();

        this->declare_parameter("mpc_enabled", mpc_enabled);
        mpc_enabled = this->get_parameter("mpc_enabled").as_bool();

        // === SUBSCRIBERS ===
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

                // Handle odom's angle wrapping discontinuity
                // if(last_theta - x0[2] > M_PI)
                //     x0[2] -= 2*M_PI;
                // else if(last_theta - x0[2] < -M_PI)
                //     x0[2] += 2*M_PI;
                // last_theta = x0[2];
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
                for (int i = 0; i < N_SP; i++)
                {
                    ocp_params[i] = msg.data[i];
                }
                // Update spline parameteres for all stages
                for (int i = 0; i <= N_HORIZON; i++)
                {
                    asv_dynamics_acados_update_params(ocp_capsule, i, ocp_params, NP);
                }
            });

        // === PARAMETER EVENT HANDLERS ===
        // For weight values
        weights_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto weights_param_cb = [this](const rclcpp::Parameter &p)
        {
            mpc_weights = p.as_double_array();
            for (int i = 0; i < N_WP; i++)
            {
                ocp_params[N_SP + i] = mpc_weights[i];
            }
        };
        weights_param_handle_ = weights_param_sub_->add_parameter_callback("mpc_weights", weights_param_cb);

        // For MPC toggle
        enabled_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto enabled_param_cb = [this](const rclcpp::Parameter &p)
        {
            mpc_enabled = p.as_bool();
        };
        enabled_param_handle_ = enabled_param_sub_->add_parameter_callback("mpc_enabled", enabled_param_cb);

        // For TF update
        tf_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto tf_param_cb = [this](const rclcpp::Parameter &p)
        {
            mpc_tf = p.as_double();
            double mpc_dt = mpc_tf / N_HORIZON;

            double new_time_steps[N_HORIZON];
            for (int i = 0; i < N_HORIZON; i++)
            {
                new_time_steps[i] = mpc_dt;
            }

            int status = asv_dynamics_acados_update_time_steps(ocp_capsule, N_HORIZON, new_time_steps);

            if (status != 0)
                RCLCPP_ERROR(this->get_logger(), "Failed to update time steps!");
            else
                RCLCPP_INFO(this->get_logger(),"Successfully updated MPC horizon: Tf=%.2fs, dt=%.4fs", mpc_tf, mpc_dt);
        };
        tf_param_handle_ = tf_param_sub_->add_parameter_callback("mpc_tf", tf_param_cb);

        // === PUBLISHERS ===
        sol_time_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/mpc/sol_time", 10);

        sol_path_pub_ =
            this->create_publisher<nav_msgs::msg::Path>("/mpc/sol_path", 10);

        heading_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_heading", 10);

        vel_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/guidance/desired_velocity", 10);

        right_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/usv/right_thruster", 10);
        left_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/usv/left_thruster", 10);

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
        // for (int i = 0; i <= N_HORIZON; i++)
        // {
        //     asv_dynamics_acados_update_params(ocp_capsule, i, spline_params, NP);
        // }
        for (int i = 0; i < N_WP; i++)
        {
            ocp_params[N_SP + i] = mpc_weights[i];
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
        vel_setpoint_pub_, heading_setpoint_pub_,
        left_thruster_pub_, right_thruster_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr spline_t_sub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> weights_param_sub_, enabled_param_sub_, tf_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> weights_param_handle_, enabled_param_handle_, tf_param_handle_;

    std_msgs::msg::Float64 sol_time_msg,
        vel_setpoint_msg, heading_setpoint_msg,
        left_thruster_msg, right_thruster_msg;
    nav_msgs::msg::Path sol_path_msg;

    // ROS2 parms global variables
    // w_along, w_cross, w_heading, w_input, w_slack, w_surge, w_yaw
    std::vector<double> mpc_weights{200.0, 5000.0, 100.0, 10.0, 100.0, 10.0, 10.0};
    double mpc_tf{2.5};
    bool mpc_enabled{true};

    rclcpp::TimerBase::SharedPtr timer_;

    int status{0};
    double ocp_params[NP];
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

    double last_theta{0.0};

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
        vel_setpoint_msg.data = xtraj[2 * NX + 3];
        heading_setpoint_msg.data = xtraj[2 * NX + 2];

        // RCLCPP_INFO(this->get_logger(), "Tp, Ts: %f, %f", simU[0], simU[1]);
        left_thruster_msg.data = simU[0];
        right_thruster_msg.data = simU[1];

        int sqp_iter;
        ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
        // FOR DEBUGGING
        // RCLCPP_INFO(this->get_logger(), "SQP iters: %d", sqp_iter);

        sol_time_pub_->publish(sol_time_msg);
        sol_path_pub_->publish(sol_path_msg);
        if (!mpc_enabled)
        {
            vel_setpoint_msg.data = 0.0;
            heading_setpoint_msg.data = 0.0;
            left_thruster_msg.data = 0.0;
            right_thruster_msg.data = 0.0;
        }
        vel_setpoint_pub_->publish(vel_setpoint_msg);
        heading_setpoint_pub_->publish(heading_setpoint_msg);
        left_thruster_pub_->publish(left_thruster_msg);
        right_thruster_pub_->publish(right_thruster_msg);
    }

    double normalize_angle(double x)
    {
        x = fmod(x + M_PI, M_PI * 2);
        if (x < 0)
            x += M_PI * 2;
        return x - M_PI;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}