#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>
#include <map>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleNearestPublisherNode : public rclcpp::Node {
    public:
        ObstacleNearestPublisherNode(): Node("obstacle_publisher_node") {

            object_n_nearest_list_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/obj_n_nearest_list", 10);

            obstacle_list_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
                "/obj_list_global", 10,
                [this](const usv_interfaces::msg::ObjectList &msg){
                    std::vector<std::pair<double, int>> obj_dist_v;
                    for(int i = 0 ; i < msg.obj_list.size() ; i++){
                        obj_dist_v.push_back(
                            std::pair<double, int>{obj_dist(msg.obj_list[i], pose), i});
                    }
                    std::sort(obj_dist_v.begin(),obj_dist_v.end());

                    out.obj_list.clear();
                    for(int i = 0 ; i < obj_dist_v.size() ; i++){
                        if(out.obj_list.size() < 5){
                            out.obj_list.push_back(msg.obj_list[obj_dist_v[i].second]);
                        }
                    }
                    while(out.obj_list.size() < 5){
                        out.obj_list.push_back(usv_interfaces::build<usv_interfaces::msg::Object>().x(0.).y(0.).color(5).type("NaN"));
                    }
                });

            pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10,
                [this](const geometry_msgs::msg::Pose2D &msg){  pose = msg; });

            timer_ = this->create_wall_timer(
            100ms, std::bind(&ObstacleNearestPublisherNode::timer_callback, this));

        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr object_n_nearest_list_pub_;

        rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr obstacle_list_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;

        usv_interfaces::msg::ObjectList out;

        geometry_msgs::msg::Pose2D pose;

                
        void timer_callback() {
            object_n_nearest_list_pub_->publish(out);
        }

        double obj_dist(usv_interfaces::msg::Object obj, geometry_msgs::msg::Pose2D p){
            return sqrt((obj.x-p.x)*(obj.x-p.x) + (obj.y-p.y)*(obj.y-p.y));
        }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleNearestPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
