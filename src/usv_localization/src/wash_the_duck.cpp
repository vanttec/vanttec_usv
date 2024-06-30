#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/waypoint.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wash.h"
#include "wash.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WashTheDuckNode : public rclcpp::Node {
    public:
        WashTheDuckNode(): Node("wash_the_duck") {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10, std::bind(&WashTheDuckNode::pose_callback, this, _1)
            );

            autoSub = this->create_subscription<std_msgs::msg::UInt16>(
                "/usv/op_mode", 1,
                [this](const std_msgs::msg::UInt16 &msg) { 
                    this->just_in = false;
                    if(this->auto_mode.data == 1 && msg.data == 0)
                        this->just_in = true;
                    this->auto_mode.data = msg.data; 
                    });

            arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/usv/waypoint/arrived", 10, std::bind(&WashTheDuckNode::arrived_callback, this, _1)
            );

            object_list_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
                "/objects_docking", 10, std::bind(&WashTheDuckNode::obj_list_callback, this, _1)
            );
            
            mission_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/m5/state", 10);
            mission_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/m5/status", 10);
            wp_pub_ = this->create_publisher<usv_interfaces::msg::Waypoint>("/usv/waypoint", 10);
            desired_pivot_pub_ = this->create_publisher<std_msgs::msg::Bool>("/usv/waypoint/pivot", 10);

            // this->pose.x = 0.0;
            // this->pose.y = 0.0;
            // this->pose.theta = 5.0;
            this->vtec_s3 = WASH(this->pose);
            this->pivot.data = false;
            timer_ = this->create_wall_timer(
            500ms, std::bind(&WashTheDuckNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr autoSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
        rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr object_list_sub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr desired_pivot_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_;
        rclcpp::Publisher<usv_interfaces::msg::Waypoint>::SharedPtr wp_pub_;

        USVOutput feedback;
        USVPose pose;
        WASH vtec_s3;
        USVUpdate update_params;

        bool goal_reached{false}, moving{false}, just_in{false};
        float goal_dist{0.0};
        float x_diff{0.0}, y_diff{0.0};
        int yellow_found{0}, black_found{0}, pivots_to_do{-1}, pivots_done{0};
        float pivot_goal{-4};
        
        float h_acum{0.0}, last_h{0.0};
        
        std::vector<usv_interfaces::msg::Waypoint> goals;

        usv_interfaces::msg::Waypoint goal;
        usv_interfaces::msg::Object obj;
        std_msgs::msg::Int8 state, status;
        std_msgs::msg::Bool pivot, arrived;
        std_msgs::msg::UInt16 auto_mode;
        std::vector<Obstacle> obs_v;

        void timer_callback() {
            if(this->auto_mode.data == 0){
                this->x_diff = std::fabs(this->goal.x - this->pose.x);
                this->y_diff = std::fabs(this->goal.y - this->pose.y);
                this->goal_dist = std::sqrt(std::pow(this->x_diff,2) + std::pow(this->y_diff,2));
                std::cout << this->goal_dist << std::endl;
                this->goal_reached = false;
                if(this->goal_dist < 1.7)
                    this->goal_reached = true;
                this->update_params.goal_reached = this->goal_reached;
                this->update_params.obs_list = this->obs_v;
                this->update_params.pivots_done = this->pivots_done;

                if(this->goal_reached || this->just_in){
                    this->feedback = this->vtec_s3.update(this->pose, this->update_params);
                    this->state.data = this->feedback.state;
                    this->status.data = this->feedback.status;
                    this->pivots_to_do = this->feedback.pivots_to_do;
                    this->goals.clear();
                    for(int i = 0 ; i < this->feedback.goals.size() ; i++){
                        this->goal.x = this->feedback.goals[i].x;
                        this->goal.y = this->feedback.goals[i].y;
                        this->goals.push_back(this->goal);
                        std::cout << "GOALS_: " << goal.x << ", " << goal.y << std::endl;

                    }
                }

                // std::cout << "size " << this->goals.size() << " XXXXXXXXX:" << this->feedback.goals[0].x << ", YYYYYYYYYY:" << this->goal.y << std::endl;
                // RCLCPP_INFO(get_logger(), "Goal: %f, %f", this->goal.x, this->goal.y);

                mission_state_pub_->publish(this->state);         
                mission_status_pub_->publish(this->status); 

                // if(this->pivots_to_do == -1){
                if(this->goals.size() > 0){
                    this->goal.x = this->goals[0].x;
                    this->goal.y = this->goals[0].y;
                }
                wp_pub_->publish(this->goal); 
                this->pivot.data = false;
                // }
                // else{
                //     std::cout << "TO DO: " << this->pivots_to_do << std::endl;
                //     if(this->pivot_goal == -4){
                //         this->pivot_goal = this->pose.theta + this->pivots_to_do * 2 * M_PI;
                //         this->h_acum = 0; 
                //         if(this->pose.theta < 0)
                //             this->h_acum = -2*M_PI;
                //         this->pivot.data = true;
                //     }
                //     if(!this->pivots_done){
                //         if(this->pose.theta > 0 && this->last_h < 0)
                //             this->h_acum+=2*M_PI;
                //         std::cout << "Accumulated ang: " << this->h_acum + this->pose.theta << ", goal: " << this->pivot_goal << std::endl;
                //         if(this->h_acum + this->pose.theta >= this->pivot_goal - 0.2)
                //             this->pivots_done = 1;
                //     } else{
                //         this->pivot.data = false;
                //         // this->pivots_done = 1;
                //     }
                // }

                desired_pivot_pub_->publish(this->pivot);
                this->last_h = this->pose.theta; 
            } else if(this->auto_mode.data == 1){
                this->vtec_s3 = WASH(this->pose);
                this->goal.x = this->pose.x;
                this->goal.y = this->pose.y;
                this->goals.clear();
            }
        }

        void pose_callback(const geometry_msgs::msg::Pose2D & msg) {
            this->pose.x = msg.x;
            this->pose.y = msg.y;
            this->pose.theta = msg.theta;
        }

        void arrived_callback(const std_msgs::msg::Bool & msg) {
            this->arrived.data = msg.data;
        }


        void obj_list_callback(const usv_interfaces::msg::ObjectList & msg) {
            Obstacle obs_t;
	        bool fake_news = false;
            this->obs_v.clear();
            for(int i = 0 ; i < msg.obj_list.size() ; i++){
                fake_news = false;
                if(msg.obj_list[i].color != 5 && msg.obj_list[i].color != -1 && (msg.obj_list[i].x != 0 || msg.obj_list[i].y != 0)){
                    for(int j = 0 ; j < this->obs_v.size() ; j++){
                        if((int)(msg.obj_list[i].color) == (int)(this->obs_v[j].color)){
                            fake_news = true;
                            std::cout << msg.obj_list[i].color << "==" << this->obs_v[j].color << std::endl;
                        }
                    }
                    if(!fake_news){
                        // std::cout << "Adding " << msg.obj_list[i].color << std::endl;
                        obs_t.color = (int)(msg.obj_list[i].color);
                        obs_t.type = msg.obj_list[i].type.c_str();
                        obs_t.x = (float)(msg.obj_list[i].x);
                        obs_t.y = (float)(msg.obj_list[i].y);
                        this->obs_v.push_back(obs_t);
                    }
                }
            }
	        std::sort(this->obs_v.begin(),this->obs_v.end(), [](Obstacle &a, Obstacle &b){ return a.y < b.y; });
            for(int i = 0 ; i < this->obs_v.size() ; i++){
                std::cout << " " << this->obs_v[i].color;
            }
            std::cout << std::endl;
        }
        
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WashTheDuckNode>());
  rclcpp::shutdown();
  return 0;
}
