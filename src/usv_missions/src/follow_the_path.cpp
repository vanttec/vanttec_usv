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
#include "usv_interfaces/msg/waypoint.hpp"
#include "usv.h"
#include "usv.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class FollowThePathNode : public rclcpp::Node {
    public:
        FollowThePathNode(): Node("follow_the_path") {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10, std::bind(&FollowThePathNode::pose_callback, this, _1)
            );

            object_list_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
                "/objects", 10, std::bind(&FollowThePathNode::obj_list_callback, this, _1)
            );
            
            mission_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv_mission/state", 10);
            mission_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv_mission/status", 10);
            wp_pub_ = this->create_publisher<usv_interfaces::msg::Waypoint>("/usv/waypoint", 10);
        
            this->pose.x = 0.0;
            this->pose.y = 0.0;
            this->pose.theta = 0.0;
            this->vtec_s3 = USV(this->pose);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&FollowThePathNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr object_list_sub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_;
        rclcpp::Publisher<usv_interfaces::msg::Waypoint>::SharedPtr wp_pub_;

        USVOutput feedback;
        USVPose pose;
        USV vtec_s3;
        USVUpdate update_params;

        bool goal_reached{false};
        float goal_dist{0.0};
        float x_diff{0.0}, y_diff{0.0};
        int yellow_found{0}, black_found{0};


        usv_interfaces::msg::Waypoint goal;
        usv_interfaces::msg::Object obj;
        std_msgs::msg::Int8 state, status;
        std::vector<Obstacle> obs_v;

        void timer_callback() {   
            this->x_diff = std::fabs(this->goal.x - this->pose.x);
            this->y_diff = std::fabs(this->goal.y - this->pose.y);
            this->goal_dist = std::sqrt(std::pow(this->x_diff,2) + std::pow(this->y_diff,2));
            this->goal_reached = false;
            if(this->goal_dist < 0.2)
                this->goal_reached = true;
            this->update_params.goal_reached = this->goal_reached;
            this->update_params.obs_list = this->obs_v;

            this->feedback = this->vtec_s3.update(this->pose, this->update_params);
            this->state.data = this->feedback.state;
            this->status.data = this->feedback.status;
            this->goal.x = this->feedback.goal.x;
            this->goal.y = this->feedback.goal.y;
            this->yellow_found = this->feedback.yellow_found;
            this->black_found = this->feedback.black_found;

            RCLCPP_INFO(get_logger(), "Goal: %f, %f", this->goal.x, this->goal.y);

            mission_state_pub_->publish(this->state);         
            mission_status_pub_->publish(this->status);  
            wp_pub_->publish(this->goal); 
        }

        void pose_callback(const geometry_msgs::msg::Pose2D & msg) {
            this->pose.x = msg.x;
            this->pose.y = msg.y;
            this->pose.theta = msg.theta;
        }

        void obj_list_callback(const usv_interfaces::msg::ObjectList & msg) {
            Obstacle obs_t;
            this->obs_v.clear();
            for(int i = 0 ; i < msg.obj_list.size() ; i++){
                if(msg.obj_list[i].color != 5){
                    obs_t.color = (int)(msg.obj_list[i].color);
                    obs_t.type = msg.obj_list[i].type.c_str();
                    obs_t.x = (float)(msg.obj_list[i].x);
                    obs_t.y = (float)(msg.obj_list[i].y);
                    this->obs_v.push_back(obs_t);
                }
                //msg.obj_list[i].color.data;
                // RCLCPP_INFO(get_logger(), "color: %d", msg.obj_list[i].color);
            }
        }
        
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowThePathNode>());
  rclcpp::shutdown();
  return 0;
}