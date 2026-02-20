#include <algorithm>
#include <cmath>
#include <array>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

// Evaluate cubic spline at local t in [0,1]
// coeffs layout (matches /mpc/spline_params):
//   [0..3] = x: a,b,c,d   [4..7] = y: a,b,c,d
static std::pair<double,double> eval_spline(const double* c, double t)
{
    return {
        c[0]*t*t*t + c[1]*t*t + c[2]*t + c[3],
        c[4]*t*t*t + c[5]*t*t + c[6]*t + c[7]
    };
}

class LOSNode : public rclcpp::Node {
public:
    LOSNode() : Node("los_node")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/usv/state/odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                auto & q = msg->pose.pose.orientation;
                x_   = msg->pose.pose.position.x;
                y_   = msg->pose.pose.position.y;
                psi_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                  1.0 - 2.0*(q.y*q.y + q.z*q.z));
            });

        // 16 doubles: current spline [0..7], next spline [8..15]
        spline_params_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mpc/spline_params", 10,
            [this](const std_msgs::msg::Float64MultiArray & msg) {
                if (msg.data.size() >= 16) {
                    std::copy(msg.data.begin(), msg.data.begin()+16, spline_params_.begin());
                    ready_ = true;
                }
            });

        // absolute: floor = spline index, frac = local t  (e.g. 2.73 = spline #2, t=0.73)
        spline_t_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_t", 10,
            [this](const std_msgs::msg::Float64 & msg) { spline_t_ = msg.data; });

        // same format for lookahead — at most spline_t + 2.0
        spline_t_la_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mpc/spline_t_la", 10,
            [this](const std_msgs::msg::Float64 & msg) { spline_t_la_ = msg.data; });

        heading_pub_     = create_publisher<std_msgs::msg::Float64>("/guidance/desired_heading",  10);
        velocity_pub_    = create_publisher<std_msgs::msg::Float64>("/guidance/desired_velocity", 10);
        current_ref_pub_ = create_publisher<nav_msgs::msg::Path>("/usv/current_path_ref", 10);

        current_ref_.header.frame_id = "world";
        geometry_msgs::msg::PoseStamped dummy;
        dummy.header.frame_id = "world";
        current_ref_.poses = {dummy, dummy};

        timer_ = create_wall_timer(100ms, std::bind(&LOSNode::update, this));
        RCLCPP_INFO(get_logger(), "los_node ready | max_vel=%.2f  k_cte=%.2f", max_vel_, k_cte_);
    }

private:
    void update()
    {
        if (!ready_) {
            std_msgs::msg::Float64 zero;
            zero.data = 0.0;
            velocity_pub_->publish(zero);
            return;
        }

        // ── local t values relative to current spline ─────────────────────────
        double t    = spline_t_    - std::floor(spline_t_);  // [0,1] on current spline
        double t_la = spline_t_la_ - std::floor(spline_t_);  // relative, up to 2.0

        // ── current point on spline ───────────────────────────────────────────
        auto [cx, cy] = eval_spline(spline_params_.data(), t);

        // ── lookahead point: next spline coeffs if t_la > 1.0 ────────────────
        double        t_la_local = t_la;
        const double* la_coeffs  = spline_params_.data();   // current spline [0..7]

        if (t_la_local > 1.0) {
            t_la_local -= 1.0;
            la_coeffs   = spline_params_.data() + 8;        // next spline [8..15]
        }
        t_la_local = std::clamp(t_la_local, 0.0, 1.0);

        auto [lax, lay] = eval_spline(la_coeffs, t_la_local);

        // ── crosstrack error (signed, perpendicular to path segment) ─────────
        double path_dx  = lax - cx;
        double path_dy  = lay - cy;
        double path_len = std::sqrt(path_dx*path_dx + path_dy*path_dy);

        double cte = 0.0;
        if (path_len > 1e-3)
            cte = ((y_ - cy) * path_dx - (x_ - cx) * path_dy) / path_len;

        // ── ILOS heading: LOS angle corrected by crosstrack ──────────────────
        double psi_los = std::atan2(lay - cy, lax - cx);
        double psi_d   = psi_los - std::atan(k_cte_ * cte);

        // ── stop condition ────────────────────────────────────────────────────
        // current == next coeffs means no more splines ahead
        bool in_last = true;
        for (int i = 0; i < 8; i++) {
            if (std::fabs(spline_params_[i] - spline_params_[8+i]) > 1e-4) {
                in_last = false;
                break;
            }
        }

        bool done = in_last && (t_la >= 1.0) && (t >= 0.8);

        // ── velocity ──────────────────────────────────────────────────────────
        double vel = max_vel_;

        if (done) {
            vel   = 0.0;
            psi_d = psi_;   // freeze heading, avoid spinning in place
        } else if (in_last && t > 0.9) {
            vel = std::clamp((1.0 - t_la) / 0.1 * max_vel_, 0.0, max_vel_);
        }

        // ── publish ───────────────────────────────────────────────────────────
        std_msgs::msg::Float64 h_msg, v_msg;
        h_msg.data = psi_d;
        v_msg.data = vel;
        heading_pub_->publish(h_msg);
        velocity_pub_->publish(v_msg);

        // debug: LOS segment from current spline point to lookahead
        current_ref_.header.stamp = this->now();
        current_ref_.poses[0].pose.position.x = cx;
        current_ref_.poses[0].pose.position.y = cy;
        current_ref_.poses[1].pose.position.x = lax;
        current_ref_.poses[1].pose.position.y = lay;
        current_ref_pub_->publish(current_ref_);
    }

    // ── members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           spline_t_sub_, spline_t_la_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              heading_pub_, velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 current_ref_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    nav_msgs::msg::Path current_ref_;

    // [0..7]  = current spline (x: a,b,c,d | y: a,b,c,d)
    // [8..15] = next spline    (same layout)
    std::array<double, 16> spline_params_{};

    double spline_t_{0.0};    // absolute: floor = spline index, frac = local t
    double spline_t_la_{0.0};

    double x_{0.0}, y_{0.0}, psi_{0.0};

    const double max_vel_{0.5};
    const double k_cte_{0.5};  // crosstrack gain: 0.3 gentle, 1.0 aggressive

    bool ready_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LOSNode>());
    rclcpp::shutdown();
    return 0;
}