#include <eigen3/Eigen/Dense>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "utils/CatmulRom.cpp"

using namespace std::chrono_literals;

class SplinePublisherNode : public rclcpp::Node
{
public:
    SplinePublisherNode() : Node("spline_publisher_node")
    {
        using namespace std::placeholders;

        spline_pub_ = this->create_publisher<nav_msgs::msg::Path>("/usv/path_ref", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spline_marker", 10);

        // PoseStamped msg from RViz
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                asv.x() = msg->pose.position.x;
                asv.y() = msg->pose.position.y;
            });


        timer_ = this->create_wall_timer(
            100ms, std::bind(&SplinePublisherNode::update, this));

        path_msg.header.frame_id = "world";

        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = 0;
        marker_msg.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.1).z(0.1);
        marker_msg.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(1).a(1);

        Eigen::Vector3d mother_cps[2]{Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(4.0, 2.0, 0.0)};
        // k+2 control points needed and at least 4 cps for catmul spline.
        std::vector<Eigen::Vector2d> cps;
        double transforms[]{-dist, 0, dist};
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cps.push_back(translate(mother_cps[i], transforms[j]));
            }
        }

        for (int i = 0; i < 3; i++)
        {
            s_[i].update(cps[i], cps[i + 1], cps[i + 2], cps[i + 3]);
        }
    }

protected:
    void update()
    {
        path_msg.poses.clear();
        path_msg.header.stamp = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped tmp_pose;
        tmp_pose.header = path_msg.header;
        marker_msg.header = path_msg.header;

        Eigen::Vector2d tmp_v;

        double closest_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_p;

        for (int i = 0; i < 3; i++)
        {
            for (double t = 0; t < 1; t += 1.0 / n_)
            {
                tmp_v = s_[i].get_s(t);
                tmp_pose.pose.position.x = tmp_v.x();
                tmp_pose.pose.position.y = tmp_v.y();
                // tmp_pose.pose.position.z = i + t;

                path_msg.poses.push_back(tmp_pose);
            }

            double local_closest_t = s_[i].closest_t(asv);
            Eigen::Vector2d local_closest_p = s_[i].get_s(local_closest_t);
            double local_closest_dist = CatmulRom::distance(asv, local_closest_p);

            if(local_closest_dist < closest_dist){
                closest_p = local_closest_p;
                closest_dist = local_closest_dist;
            }
        }

        marker_msg.pose.position.x = closest_p.x();
        marker_msg.pose.position.y = closest_p.y();

        spline_pub_->publish(path_msg);
        marker_pub_->publish(marker_msg);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::Marker marker_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    CatmulRom s_[3];
    int n_{100};
    double dist{0.1};

    Eigen::Vector2d asv{2,0};

    Eigen::Vector2d translate(Eigen::Vector3d v, double dist)
    {
        Eigen::Vector2d w, p;
        w << v(0), v(1);
        p << std::cos(v(2)), std::sin(v(2));
        return w + dist * p;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}