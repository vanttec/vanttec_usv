#include <eigen3/Eigen/Dense>
#include <vector>
#include <limits>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "utils/CatmulRom.cpp"

using namespace std::chrono_literals;

class SplinePublisherNode : public rclcpp::Node
{
public:
    SplinePublisherNode() : Node("spline_publisher_node")
    {
        using namespace std::placeholders;

        spline_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/usv/path_ref", 10);
        s_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spline_marker", 10);
        la_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);
        spline_params_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/spline_params", 10);
        spline_t_pub_ = this->create_publisher<std_msgs::msg::Float64>("/mpc/spline_t", 10);
        spline_t_la_pub_ = this->create_publisher<std_msgs::msg::Float64>("/mpc/spline_t_la", 10);
        spline_length_pub_ = this->create_publisher<std_msgs::msg::Float64>("/mpc/spline_l", 10);
        
        // Goal from mission_handler node
        mission_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/usv/goals/pose_array", 1,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
            {
                if (msg->poses.empty()) return;

                // Only replan if the final destination actually changed
                auto & last = msg->poses.back();
                if (!ref.empty()) {
                    double dx = last.position.x - last_goal_x_;
                    double dy = last.position.y - last_goal_y_;
                    if (std::sqrt(dx*dx + dy*dy) < 0.1) {
                        // same destination — just append new intermediate points
                        // without resetting closest_idx
                        ref.clear();
                        ref.push_back(trans(asv, -dist));
                        ref.push_back(trans(asv,  0.0));
                        for (int i = 0; i < msg->poses.size(); i++)
                            ref.push_back(trans(p_to_v(msg->poses[i]), 0.0));
                        ref.push_back(trans(p_to_v(msg->poses.back()), dist));
                        // don't reset closest_idx here
                        update_spline_params();
                        return;
                    }
                }

                // Final destination changed — full replan
                last_goal_x_ = last.position.x;
                last_goal_y_ = last.position.y;
                ref.clear();
                ref.push_back(trans(asv, -dist));
                ref.push_back(trans(asv,  0.0));
                for (int i = 0; i < msg->poses.size(); i++)
                    ref.push_back(trans(p_to_v(msg->poses[i]), 0.0));
                ref.push_back(trans(p_to_v(msg->poses.back()), dist));
                closest_idx = 0;
                last_idx    = -1;
                update_spline_params();
            });

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/usv/state/odom", 1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                auto &q = msg->pose.pose.orientation;

                asv.x() = msg->pose.pose.position.x;
                asv.y() = msg->pose.pose.position.y;
                asv.z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            });

        // Goal as a PoseStamped msg (for RViz)
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                ref.clear();
                ref.push_back(trans(asv, -dist));
                ref.push_back(trans(asv, 0.0));

                auto &q = msg->pose.orientation;
                tmp.x() = msg->pose.position.x;
                tmp.y() = msg->pose.position.y;
                tmp.z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                ref.push_back(trans(tmp, 0.0));
                ref.push_back(trans(tmp, dist));

                update_spline_params();
            });

        goal_pose_to_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_to", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                auto &q = msg->pose.orientation;
                tmp.x() = msg->pose.position.x;
                tmp.y() = msg->pose.position.y;
                tmp.z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                ref[ref.size()-1] = trans(tmp, 0.0);
                ref.push_back(trans(tmp, dist));
                
                update_spline_params();
            });

        goal_pose_from_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_from", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                ref.clear();
                auto &q = msg->pose.orientation;
                tmp.x() = msg->pose.position.x;
                tmp.y() = msg->pose.position.y;
                tmp.z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                ref.push_back(trans(tmp, -dist));
                ref.push_back(trans(tmp, 0.0));
                ref.push_back(trans(tmp, dist));
                
                update_spline_params();
            });

            timer_ = this->create_wall_timer(
            100ms, std::bind(&SplinePublisherNode::update, this));

        path_msg.header.frame_id = "world";

        // Setup spline marker
        s_marker_msg.id = 0;
        s_marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        s_marker_msg.action = 0;
        s_marker_msg.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.1).z(0.1);
        s_marker_msg.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(1).a(1);

        // Setup lookahead marker
        la_marker_msg.id = 0;
        la_marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        la_marker_msg.action = 0;
        la_marker_msg.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.1).z(0.1);
        la_marker_msg.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1).g(0).b(0).a(1);

        // Current and next spline's params
        spline_params_msg.data.resize(16);
        update_spline_params();
    }

protected:
    void update()
    {
        path_msg.poses.clear();
        path_msg.header.stamp = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped tmp_pose;
        tmp_pose.header = path_msg.header;
        s_marker_msg.header = path_msg.header;
        la_marker_msg.header = path_msg.header;

        if(s_.size() > 0){
            if (closest_idx < 0) closest_idx = 0;
            Eigen::Vector2d tmp_v;
            Eigen::Vector2d closest_p_tmp, closest_p;
            double closest_t, closest_t_tmp;
            double closest_dist = std::numeric_limits<double>::max();

            for(int i = 0 ; i < s_.size() ; i++){

                for (double t = 0; t <= 1; t += 1.0 / (n_ - 1))
                {
                    tmp_v = s_[i].get_s(t);
                    tmp_pose.pose.position.x = tmp_v.x();
                    tmp_pose.pose.position.y = tmp_v.y();
                    // tmp_pose.pose.position.z = i + t;
                    path_msg.poses.push_back(tmp_pose);
                }
                
                closest_t_tmp = s_[i].closest_t(asv);
                closest_p_tmp = s_[i].get_s(closest_t_tmp);

                if(distance(asv, closest_p_tmp) < closest_dist && 
                    fabs(i-closest_idx) <= 1){
                    closest_t = closest_t_tmp;
                    closest_dist = distance(asv, closest_p_tmp);
                    closest_idx = i;
                }
            }

            closest_p = s_[closest_idx].get_s(closest_t);

            double lookahead = 1.5;
            // For length L, we want to find a t+dt such that s(t+dt) is at [dist] from s(t)
            // To map L to dist: L is to 1, what dist is to dt -> dt = dist/L
            L_ = s_[closest_idx].L_;
            double la_t = s_[closest_idx].get_la(closest_t, lookahead);
            Eigen::Vector2d la_p = s_[closest_idx].get_s(la_t);
            if(la_t == 1.0 && closest_idx+1 < s_.size()){
                // la_t is most likely saturated and there still are splines left to cover
                double rem_dist = lookahead - s_[closest_idx].get_arc_length(closest_t, la_t);
                la_t = 1 + s_[closest_idx+1].get_la(0.0, rem_dist);
                la_p = s_[closest_idx+1].get_s(la_t - 1);
            }

            s_marker_msg.pose.position.x = closest_p.x();
            s_marker_msg.pose.position.y = closest_p.y();

            la_marker_msg.pose.position.x = la_p.x();
            la_marker_msg.pose.position.y = la_p.y();

            spline_t_msg.data = closest_idx + closest_t;
            spline_t_la_msg.data = closest_idx + la_t;

            if(last_idx != closest_idx){
                for (int i = 0; i < 2; i++)
                {
                    spline_params_msg.data[4 * i + 0] = s_[closest_idx].s_.a[i];
                    spline_params_msg.data[4 * i + 1] = s_[closest_idx].s_.b[i];
                    spline_params_msg.data[4 * i + 2] = s_[closest_idx].s_.c[i];
                    spline_params_msg.data[4 * i + 3] = s_[closest_idx].s_.d[i];
                }

                // If there is a 'next spline', update it!
                if(closest_idx+1 < s_.size()){
                    for (int i = 0; i < 2; i++)
                    {
                        spline_params_msg.data[4 * i + 8]  = s_[closest_idx+1].s_.a[i];
                        spline_params_msg.data[4 * i + 9]  = s_[closest_idx+1].s_.b[i];
                        spline_params_msg.data[4 * i + 10] = s_[closest_idx+1].s_.c[i];
                        spline_params_msg.data[4 * i + 11] = s_[closest_idx+1].s_.d[i];
                    }
                }
            }

            last_idx = closest_idx;

            spline_length_msg.data = L_;
        }

        spline_path_pub_->publish(path_msg);
        s_marker_pub_->publish(s_marker_msg);
        la_marker_pub_->publish(la_marker_msg);
        spline_params_pub_->publish(spline_params_msg);
        spline_t_pub_->publish(spline_t_msg);
        spline_t_la_pub_->publish(spline_t_la_msg);
        spline_length_pub_->publish(spline_length_msg);
    }

    void update_spline_params()
    {
        s_.clear();
        if(ref.size() < 4)
            return;
        s_.resize(ref.size() - 3);
        for(int i = 0 ; i < s_.size() ; i++){
            s_[i].update(
                ref[i  ],
                ref[i+1],
                ref[i+2],
                ref[i+3]
            );
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr s_marker_pub_, la_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spline_t_pub_, spline_t_la_pub_, spline_length_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_from_sub_, goal_pose_to_sub_, goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mission_goal_sub_;

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::Marker s_marker_msg, la_marker_msg;
    std_msgs::msg::Float64MultiArray spline_params_msg;
    std_msgs::msg::Float64 spline_t_msg, spline_t_la_msg, spline_length_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<CatmulRom> s_;
    double L_{0.0};
    int n_{20};
    double dist{0.1};
    int closest_idx{-1};
    int last_idx{-1};

    // std::vector<Eigen::Vector2d> ref{{0,0},{1,0},{7,3},{13,-3},{19,3},{25,-3}};
    // std::vector<Eigen::Vector2d> ref{{0,0},{0.3,0},{3,5},{7,0},{6,-5},{7,-8},{1,-5},{0.5,-1},{0,0}};
    std::vector<Eigen::Vector2d> ref{};
    Eigen::Vector3d asv, tmp;

    double last_goal_x_{std::numeric_limits<double>::quiet_NaN()};
    double last_goal_y_{std::numeric_limits<double>::quiet_NaN()};

    Eigen::Vector3d p_to_v(geometry_msgs::msg::Pose p){
        Eigen::Vector3d v;
        auto &q = p.orientation;
        v.x() = p.position.x;
        v.y() = p.position.y;
        v.z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return v;
    }

    Eigen::Vector2d trans(Eigen::Vector3d v, double dist)
    {
        Eigen::Vector2d w, p;
        w << v(0), v(1);
        p << std::cos(v(2)), std::sin(v(2));
        return w + dist * p;
    }

    double distance(Eigen::Vector3d a, Eigen::Vector2d b)
    {
        Eigen::Vector2d c{a.x(), a.y()};
        return (b-c).norm();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}