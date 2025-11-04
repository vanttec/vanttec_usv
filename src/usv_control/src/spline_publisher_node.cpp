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
        s_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spline_marker", 10);
        la_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 10);

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
        s_marker_msg.header = path_msg.header;
        la_marker_msg.header = path_msg.header;

        Eigen::Vector2d tmp_v;

        double closest_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_p;
        double closest_t{-1};
        int closest_i{0};

        for (int i = 0; i < 3; i++)
        {
            for (double t = 0; t <= 1; t += 1.0 / (n_-1))
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
                closest_t = local_closest_t;
                closest_i = i;
            }
        }

        double lookahead = 0.3;
        // Fix t so that it's in terms of the actual dt used
        // If n = 3, t can only be an element of {0,0.5,1}
        double fit_t = int(round(closest_t*(n_-1))) * 1.0 / (n_-1);
        // Find the index in path_msg that corresponds to the closest point
        int closest_path_idx = closest_i*n_+int(fit_t * (n_-1));
        int idx_ = closest_path_idx;

        // Find furthest point along the spline (in path_msg) inside lookahead region
        bool la_passed{false}; // if lookahead has been surpassed
        while(idx_ < path_msg.poses.size() - 1 && !la_passed){
            double new_dist = distance(
                path_msg.poses[closest_path_idx].pose.position, 
                path_msg.poses[idx_].pose.position
            );
            if(new_dist > lookahead){
                la_passed = true;
            } else {
                idx_++;
            }
        }

        s_marker_msg.pose.position.x = closest_p.x();
        s_marker_msg.pose.position.y = closest_p.y();

        la_marker_msg.pose.position = path_msg.poses[idx_].pose.position;

        spline_pub_->publish(path_msg);
        s_marker_pub_->publish(s_marker_msg);
        la_marker_pub_->publish(la_marker_msg);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr s_marker_pub_, la_marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::Marker s_marker_msg, la_marker_msg;

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

    double distance(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b){
        return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}