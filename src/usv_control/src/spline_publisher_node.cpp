#include <eigen3/Eigen/Dense>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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
                ref[0] = asv;

                auto &q = msg->pose.orientation;
                ref[1].x() = msg->pose.position.x;
                ref[1].y() = msg->pose.position.y;
                ref[1].z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                
                update_spline_params();
            });

        goal_pose_to_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_to", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                auto &q = msg->pose.orientation;
                ref[1].x() = msg->pose.position.x;
                ref[1].y() = msg->pose.position.y;
                ref[1].z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                
                update_spline_params();
            });

        goal_pose_from_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_from", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                auto &q = msg->pose.orientation;
                ref[0].x() = msg->pose.position.x;
                ref[0].y() = msg->pose.position.y;
                ref[0].z() = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                
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

        // k+2 control points needed and at least 4 cps for catmul spline.
        cps.resize(4);
        spline_params_msg.data.resize(8);
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

        Eigen::Vector2d tmp_v;
        Eigen::Vector2d closest_p;
        double closest_t{-1};

        for (double t = 0; t <= 1; t += 1.0 / (n_ - 1))
        {
            tmp_v = s_.get_s(t);
            tmp_pose.pose.position.x = tmp_v.x();
            tmp_pose.pose.position.y = tmp_v.y();
            // tmp_pose.pose.position.z = i + t;
            path_msg.poses.push_back(tmp_pose);
        }

        closest_t = s_.closest_t(asv);
        closest_p = s_.get_s(closest_t);

        double lookahead = 1.0;
        // For length L, we want to find a t+dt such that s(t+dt) is at [dist] from s(t)
        // To map L to dist: L is to 1, what dist is to dt -> dt = dist/L
        double la_t = std::clamp(closest_t+lookahead/L_, 0.0, 1.0);
        Eigen::Vector2d la_p = s_.get_s(la_t);

        s_marker_msg.pose.position.x = closest_p.x();
        s_marker_msg.pose.position.y = closest_p.y();

        la_marker_msg.pose.position.x = la_p.x();
        la_marker_msg.pose.position.y = la_p.y();

        spline_t_msg.data = closest_t;
        spline_t_la_msg.data = la_t;

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
        s_.update(
            translate(ref[0],-dist),
            translate(ref[0],0.0),
            translate(ref[1],0.0),
            translate(ref[1],dist)
        );
        
        for (int i = 0; i < 2; i++)
        {
            spline_params_msg.data[4 * i + 0] = s_.s_.a[i];
            spline_params_msg.data[4 * i + 1] = s_.s_.b[i];
            spline_params_msg.data[4 * i + 2] = s_.s_.c[i];
            spline_params_msg.data[4 * i + 3] = s_.s_.d[i];
        }

        L_ = s_.arc_length();
        spline_length_msg.data = L_;
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr s_marker_pub_, la_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr spline_params_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spline_t_pub_, spline_t_la_pub_, spline_length_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_from_sub_, goal_pose_to_sub_, goal_pose_sub_;

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::Marker s_marker_msg, la_marker_msg;
    std_msgs::msg::Float64MultiArray spline_params_msg;
    std_msgs::msg::Float64 spline_t_msg, spline_t_la_msg, spline_length_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Eigen::Vector2d> cps;
    CatmulRom s_;
    double L_{0.0};
    int n_{100};
    double dist{0.1};

    Eigen::Vector3d ref[2]{{0,0,0},{1,0,0}};
    Eigen::Vector3d asv;

    Eigen::Vector2d translate(Eigen::Vector3d v, double dist)
    {
        Eigen::Vector2d w, p;
        w << v(0), v(1);
        p << std::cos(v(2)), std::sin(v(2));
        return w + dist * p;
    }

    double distance(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}