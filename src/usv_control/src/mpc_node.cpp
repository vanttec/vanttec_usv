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
#include <eigen3/Eigen/Dense>

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

#define n_obs 3

// Simulation parameters
#define N_SP 8 // Spline params (4 x NDIMS)
#define N_WP 9 // Weight params
#define N_AP 1 // Additional params
#define N_OP n_obs*2 //Obstacle params (velocities)

#define TF 2.5              // MPC prediction horizon [s]
#define DT (TF / N_HORIZON) // Time step

struct WeightParams
{
    double
        min_t,
        max_t,
        min_w,
        max_w;
};

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

        this->declare_parameter("mpc_s_max_dt", mpc_s_max_dt);
        mpc_s_max_dt = this->get_parameter("mpc_s_max_dt").as_double();

        this->declare_parameter("mpc_weights", mpc_weights);
        mpc_weights = this->get_parameter("mpc_weights").as_double_array();
        update_weight_ps();

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

                // Feed continuous heading to MPC (remove angle wrapping)
                double new_psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                double delta = new_psi - last_theta;
                if (delta > M_PI)
                    x0[2] = x0[2] + (new_psi - 2.0 * M_PI) - last_theta;
                else if (delta < -M_PI)
                    x0[2] = x0[2] + (new_psi + 2.0 * M_PI) - last_theta;
                else
                    x0[2] = x0[2] + delta;
                last_theta = new_psi;

                x0[3] = msg->twist.twist.linear.x;
                x0[4] = msg->twist.twist.angular.z;
            });

        spline_t_la_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_t_la", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                // Previously t_la would be state[5] (t)
                // x0[5] = msg.data;
                ocp_params[N_SP + N_WP] = msg.data;
            });

        spline_t_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_t", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                s_t = msg.data;
                x0[5] = msg.data;
                along_e = (1 - msg.data) * s_length;
                cross_e = get_crosstrack_e();
                // Test relation, might try for some weights
                double ca = cross_e / std::clamp(along_e, 0.01, 1e10);

                // if (s_t <= 0.05 || s_t >= 0.95)
                if (s_t >= 0.95)
                {
                    along_e = min_ae;
                    cross_e = max_ce;
                }

                // Variable weights dependant on crosstrack or alongtrack errors.
                for (int i = 0; i < N_WP; i++)
                {
                    if (i < 3)
                        ocp_params[N_SP + i] = var_w_at(weight_ps[i], cross_e);
                    else
                        ocp_params[N_SP + i] = var_w_at(weight_ps[i], along_e);
                }
            });

        spline_length_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_l", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                s_length = msg.data;
            });

        spline_params_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mpc/spline_params", 10,
            [this](const std_msgs::msg::Float64MultiArray &msg)
            {
                bool new_spline = false;

                for (int i = 0; i < N_SP; i++)
                {
                    if (std::abs(ocp_params[i] - msg.data[i]) > 1e-6)
                    {
                        ocp_params[i] = msg.data[i];
                        new_spline = true;
                    }
                }
                if (new_spline)
                {
                    RCLCPP_WARN(this->get_logger(), "Resetting trajectory guess");
                    asv_dynamics_acados_reset(ocp_capsule, 1); // 1 = reset trajectory guess
                }
            });

        // === PARAMETER EVENT HANDLERS ===
        // For weight values
        weights_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto weights_param_cb = [this](const rclcpp::Parameter &p)
        {
            mpc_weights = p.as_double_array();
            update_weight_ps();
            // for (int i = 0; i < N_WP; i++)
            // {
            //     ocp_params[N_SP + i] = mpc_weights[i];
            // }
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
                RCLCPP_INFO(this->get_logger(), "Successfully updated MPC horizon: Tf=%.2fs, dt=%.4fs", mpc_tf, mpc_dt);
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
        debug_ce_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/mpc/debug/c_e", 10);
        debug_he_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/mpc/debug/h_e", 10);
        debug_residuals_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/mpc/debug/res", 10);
        debug_weights_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mpc/debug/w", 10);

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

        debug_weights_msg.data.resize(N_WP);

        x0[6] = 5.0;
        x0[7] = 0.0;
        x0[8] = 10.0;
        x0[9] = 0.0;
        x0[10] = 15.0;
        x0[11] = 0.0;
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
        left_thruster_pub_, right_thruster_pub_,
        debug_ce_pub_, debug_he_pub_, debug_residuals_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_weights_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr spline_t_sub_, spline_t_la_sub_, spline_length_sub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> weights_param_sub_, enabled_param_sub_,
        tf_param_sub_, s_max_dt_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> weights_param_handle_, enabled_param_handle_,
        tf_param_handle_, s_max_dt_param_handle_;

    std_msgs::msg::Float64 sol_time_msg,
        vel_setpoint_msg, heading_setpoint_msg,
        left_thruster_msg, right_thruster_msg,
        debug_ce_msg, debug_he_msg, debug_residuals_msg;
    nav_msgs::msg::Path sol_path_msg;
    std_msgs::msg::Float64MultiArray debug_weights_msg;

    double along_e, cross_e, ocp_cost;

    // w_along, w_cross, w_heading, w_input, w_slack, w_surge, w_yaw, w_terminal
    std::vector<double> mpc_weights{5.0, 15.0, 20.0, 0.05, 1000.0, 0.01, 0.01, 100.0};

    // map input [min,max] to output [min,max]
    double min_ae{0.1}, max_ae{0.80}, min_ce{0.05}, max_ce{0.2};
    double max_err_weights_mult[N_WP]{
        0.1, 10.0, 5.0,         // along,cross,heading
        0.1, 0.1, 0.1, 0.1, 0.5, // input,slack,surge,yaw,terminal
        1.0 // avoidance
    };
    WeightParams weight_ps[N_WP]{
        // These first weights depend on separation (cross_err)
        {min_ce, max_ce, mpc_weights[0], mpc_weights[0] * 0.1},  // along
        {min_ce, max_ce, mpc_weights[1], mpc_weights[1] * 10.0}, // cross
        {min_ce, max_ce, mpc_weights[2], mpc_weights[2] * 5.0},  // heading

        // These last weights depend on remaining dist. (along_err)
        {min_ae, max_ae, mpc_weights[3], mpc_weights[3] * 0.1}, // input
        {min_ae, max_ae, mpc_weights[4], mpc_weights[4] * 0.1}, // slack
        {min_ae, max_ae, mpc_weights[5], mpc_weights[5] * 0.1}, // surge
        {min_ae, max_ae, mpc_weights[6], mpc_weights[6] * 0.1}, // yaw
        {min_ae, max_ae, mpc_weights[7], mpc_weights[7] * 0.5}, // terminal

        {min_ae, max_ae, mpc_weights[8], mpc_weights[8] * 1.0}, // terminal
    };
    int sol_idx{20};
    WeightParams sol_idx_weight{0.1, 0.8, 10.0, 20.0};

    double mpc_tf{2.5}, mpc_s_max_dt{0.1}, s_length{0.001}, s_t{0.};
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

    void update_all_params()
    {
        for (int i = 0; i <= N_HORIZON; i++)
        {
            asv_dynamics_acados_update_params(ocp_capsule, i, ocp_params, NP);
        }
    }

    void update_weight_ps()
    {
        for (int i = 0; i < N_WP; i++)
        {
            weight_ps[i].min_w = mpc_weights[i];
            weight_ps[i].max_w = mpc_weights[i] * max_err_weights_mult[i];
        }
    }

    void update()
    {
        // Params may always be changing
        update_all_params();

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

        // Store previous cost
        static double prev_cost = 0.0;

        // Get current cost
        ocp_nlp_eval_cost(nlp_solver, nlp_in, nlp_out);
        ocp_nlp_get(nlp_solver, "cost_value", &ocp_cost);
        debug_residuals_msg.data = ocp_cost;

        if (status == 4)
        {
            double residuals[4];
            ocp_nlp_get(nlp_solver, "res_stat", &residuals[0]);
            ocp_nlp_get(nlp_solver, "res_eq", &residuals[1]);
            ocp_nlp_get(nlp_solver, "res_ineq", &residuals[2]);
            ocp_nlp_get(nlp_solver, "res_comp", &residuals[3]);
            RCLCPP_INFO(this->get_logger(),
                        "Residuals: [%.2e, %.2e, %.2e, %.2e]",
                        residuals[0], residuals[1], residuals[2], residuals[3]);
    
            double max_res = std::max({std::abs(residuals[0]), std::abs(residuals[1]),
                                       std::abs(residuals[2]), std::abs(residuals[3])});
            if (max_res > 1e-6)
            {
                RCLCPP_WARN(this->get_logger(),
                            "QP failed: res=[%.2e, %.2e, %.2e, %.2e]",
                            residuals[0], residuals[1], residuals[2], residuals[3]);
            }
        }

        // Get optimal control
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", simU);

        auto end_t = std::chrono::high_resolution_clock::now();
        sol_time_msg.data = std::chrono::duration<double>(end_t - start_t).count();

        sol_path_msg.header.stamp = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped tmp_pose;

        double sol_length = 0.0;
        for (int i = 0; i <= N_HORIZON; i++)
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * NX]);
            tmp_pose.pose.position.x = xtraj[i * NX];
            tmp_pose.pose.position.y = xtraj[i * NX + 1];
            sol_path_msg.poses[i] = tmp_pose;

            sol_length += std::fabs(xtraj[i * NX + 3]) * mpc_tf / N_HORIZON;
        }

        sol_idx = int(var_w_at(sol_idx_weight, sol_length));
        vel_setpoint_msg.data = xtraj[sol_idx * NX + 3];
        heading_setpoint_msg.data = xtraj[sol_idx * NX + 2];

        // RCLCPP_INFO(this->get_logger(), "Tp, Ts: %f, %f", simU[0], simU[1]);
        left_thruster_msg.data = simU[0];
        right_thruster_msg.data = simU[1];

        debug_ce_msg.data = get_crosstrack_e();
        debug_he_msg.data = get_heading_e();
        for (int i = 0; i < N_WP; i++)
        {
            debug_weights_msg.data[i] = ocp_params[N_SP + i];
        }

        int sqp_iter;
        ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);

        sol_time_pub_->publish(sol_time_msg);
        sol_path_pub_->publish(sol_path_msg);
        if (!mpc_enabled || ocp_cost > 10000.0 || status == 4)
        {
            RCLCPP_ERROR(this->get_logger(), "MPC IS DISABLED");
            vel_setpoint_msg.data = 0.0;
            heading_setpoint_msg.data = x0[2];
            left_thruster_msg.data = 0.0;
            right_thruster_msg.data = 0.0;
        }
        vel_setpoint_pub_->publish(vel_setpoint_msg);
        heading_setpoint_pub_->publish(heading_setpoint_msg);
        // left_thruster_pub_->publish(left_thruster_msg);
        // right_thruster_pub_->publish(right_thruster_msg);
        debug_ce_pub_->publish(debug_ce_msg);
        debug_he_pub_->publish(debug_he_msg);
        debug_residuals_pub_->publish(debug_residuals_msg);
        debug_weights_pub_->publish(debug_weights_msg);

        // MPC Debugging
        // RCLCPP_INFO(this->get_logger(),
        //             "OCP PARAMS\nSpline {%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f}\nWeights {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f}\nT LA {%.2f}",
        //             ocp_params[0], ocp_params[1], ocp_params[2], ocp_params[3], ocp_params[4], ocp_params[5], ocp_params[6],
        //             ocp_params[7], ocp_params[8], ocp_params[9], ocp_params[10], ocp_params[11], ocp_params[12], ocp_params[13],
        //             ocp_params[14], ocp_params[15], ocp_params[16]);
        // RCLCPP_INFO(this->get_logger(),
        //             "SOLUTION IDX: %.2d, Sol. length: %.2f", sol_idx, sol_length);
        // RCLCPP_INFO(this->get_logger(), "ERRORS {a_e: %.2f, c_e: %.2f}", along_e, cross_e);
    }

    double normalize_angle(double x)
    {
        x = fmod(x + M_PI, M_PI * 2);
        if (x < 0)
            x += M_PI * 2;
        return x - M_PI;
    }

    // Get a linear variable weight depending on t and its restrictions
    double var_w_at(WeightParams p, double t)
    {
        double w_m = (p.max_w - p.min_w) / (p.max_t - p.min_t);
        double w_b = p.min_w - w_m * p.min_t;
        if (p.min_w < p.max_w)
            return std::clamp(w_m * t + w_b, p.min_w, p.max_w);
        // In some cases, slope is negative, and sol. shouldn't depend on argument order...
        return std::clamp(w_m * t + w_b, p.max_w, p.min_w);
    }

    double get_crosstrack_e()
    {
        Eigen::Vector2d spline_pos, asv_pos, pos_diff;
        asv_pos << x0[0], x0[1];
        spline_pos = get_spline(s_t);
        pos_diff = spline_pos - asv_pos;

        return pos_diff.norm();
    }

    double get_heading_e()
    {
        Eigen::Vector2d s_dot;
        s_dot = get_spline_dot(s_t);
        double psi_ref = std::atan2(s_dot.y(), s_dot.x());
        double he_sqrt = std::sin((x0[2] - psi_ref) / 2.0);
        return he_sqrt * he_sqrt;
    }

    Eigen::Vector2d get_spline(double t)
    {
        return Eigen::Vector2d{ocp_params[0] * s_t * s_t * s_t + ocp_params[1] * s_t * s_t + ocp_params[2] * s_t + ocp_params[3],
                               ocp_params[4] * s_t * s_t * s_t + ocp_params[5] * s_t * s_t + ocp_params[6] * s_t + ocp_params[7]};
    }

    Eigen::Vector2d get_spline_dot(double t)
    {
        return Eigen::Vector2d{3 * ocp_params[0] * s_t * s_t + 2 * ocp_params[1] * s_t + ocp_params[2],
                               3 * ocp_params[4] * s_t * s_t + 2 * ocp_params[5] * s_t + ocp_params[6]};
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}