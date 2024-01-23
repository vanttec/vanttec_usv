#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/object.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstaclePublisherNode : public rclcpp::Node {
    public:
        ObstaclePublisherNode(): Node("obstacle_publisher_node") {            
            object_list_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/objects", 10);

            poseSub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10,
                [this](const geometry_msgs::msg::Pose2D &msg)
                {
                    this->pose.x = msg.x;
                    this->pose.y = msg.y;
                    this->pose.theta = msg.theta;
                });

            timer_ = this->create_wall_timer(
            500ms, std::bind(&ObstaclePublisherNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr object_list_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub_;
        usv_interfaces::msg::Object obj;
        geometry_msgs::msg::Pose2D pose;

        std::vector<std::vector<float>> obsAbsPose {{1,0}, {2,0}};
        std::vector<std::vector<float>> obsRelPose;
        float obsColors[4] = {0, 1};
        float x_t{0.0}, y_t{0.0}, d_t{0.0};

        void timer_callback() {
            usv_interfaces::msg::ObjectList obj_list;
            obj_list = fill_obj_list();
            object_list_pub_->publish(obj_list);
        }        

        usv_interfaces::msg::ObjectList fill_obj_list(){
            usv_interfaces::msg::ObjectList o_l;
            obsRelPose = obsAbsPose;

            for(int i = 0 ; i < obsAbsPose.size() ; i++){
                this->x_t = obsAbsPose[i][0] - this->pose.x;
                this->y_t = obsAbsPose[i][1] - this->pose.y;
                this->d_t = std::sqrt(std::pow(this->x_t,2) + std::pow(this->y_t,2));
                std::cout << this->d_t << std::endl;

                // if(this->x_t > 0 && this->d_t < 6){
                    obsRelPose[i][0] = std::cos(pose.theta)*this->x_t - std::sin(pose.theta)*this->y_t;
                    obsRelPose[i][1] = std::sin(pose.theta)*this->x_t + std::cos(pose.theta)*this->y_t;
                    obj.x = obsRelPose[i][0];
                    obj.y = obsRelPose[i][1];
                    obj.type = "round";
                    obj.color = obsColors[i];
                    o_l.obj_list.push_back(obj);
                // }
            }

            return o_l;
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisherNode>());
  rclcpp::shutdown();
  return 0;
}