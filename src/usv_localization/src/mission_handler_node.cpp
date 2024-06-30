#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/waypoint.hpp"
#include "usv_interfaces/msg/waypoint_list.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mission_classes/mission.cpp"
#include "mission_classes/m0.cpp"
#include "mission_classes/m1.cpp"
#include "mission_classes/m2.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MissionHandlerNode : public rclcpp::Node {
    public:
        MissionHandlerNode(): Node("mission_handler_node"){
            pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10, 
                [this](const geometry_msgs::msg::Pose2D &msg) { 
                    pose << msg.x, msg.y, msg.theta;
            });

            auto_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
                "/usv/op_mode", 1,
                [this](const std_msgs::msg::UInt16 &msg) { auto_mode.data = msg.data; });

            object_list_sub_ = this->create_subscription<usv_interfaces::msg::ObjectList>(
                "/obj_list", 10, std::bind(&MissionHandlerNode::obj_list_callback, this, _1)
            );

            mission_command_sub_ = this->create_subscription<std_msgs::msg::Int8>(
                "/usv/mission_command", 10, 
                [this](const std_msgs::msg::Int8 &msg) { 
                    if(mission_command != msg.data){
                        mission_command = msg.data;
                        switch(mission_command){
                            case 1:
                                vtec = std::make_shared<M1>();
                                break;
                            case 2:
                                vtec = std::make_shared<M2>();
                                break;
                        }

                    }
            });

            vtec = std::make_shared<M2>();

            mission_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/state", 10);
            mission_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/status", 10);
            wp_pub_ = this->create_publisher<usv_interfaces::msg::WaypointList>("/usv/goals", 10);

            timer_ = this->create_wall_timer(
            100ms, std::bind(&MissionHandlerNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr auto_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
        rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr object_list_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_command_sub_;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr desired_pivot_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_status_pub_;
        rclcpp::Publisher<usv_interfaces::msg::WaypointList>::SharedPtr wp_pub_;

        usv_interfaces::msg::Object obj;
        usv_interfaces::msg::WaypointList wp_list;
        std_msgs::msg::Int8 state, status;
        std_msgs::msg::Bool pivot, arrived;
        std_msgs::msg::UInt16 auto_mode;
        std::vector<Obstacle> obs_v;

        USVOutput feedback;
        Eigen::Vector3f pose;
        USVUpdate update_params;
        
        std::shared_ptr<Mission> vtec;

        bool goal_reached{false}, moving{false};
        double goal_dist{0.0};
        double x_diff{0.0}, y_diff{0.0};
        int yellow_found{0}, black_found{0}, pivots_to_do{-1}, pivots_done{0};
        double pivot_goal{-4};
        double h_acum{0.0}, last_h{0.0};
        int mission_command{0};
        
        std::vector<usv_interfaces::msg::Waypoint> goals;

        void timer_callback() {
            feedback = vtec->update(pose, update_params);
            
            if(feedback.goals.size() > 0){
                set_goals(feedback.goals);
            }

            state.data = feedback.state;
            status.data = feedback.status;

            wp_pub_->publish(wp_list);
            mission_state_pub_->publish(state);
            mission_status_pub_->publish(status);
        }

        void obj_list_callback(const usv_interfaces::msg::ObjectList & msg) {
            Obstacle obs_t;
            obs_v.clear();
            for(int i = 0 ; i < msg.obj_list.size() ; i++){
                if(msg.obj_list[i].color != 5){
                    obs_t.color = (int)(msg.obj_list[i].color);
                    obs_t.type = msg.obj_list[i].type.c_str();
                    obs_t.x = (double)(msg.obj_list[i].x);
                    obs_t.y = (double)(msg.obj_list[i].y);
                    obs_v.push_back(obs_t);
                }
            }
            update_params.obs_list = obs_v;
        }

        void set_goals(std::vector<Eigen::Vector3f> vec){
            wp_list.waypoint_list.clear();

            usv_interfaces::msg::Waypoint wp;
            for(int i = 0 ; i < vec.size() ; i++) { 
                wp.x = vec[i](0);
                wp.y = vec[i](1);
                wp_list.waypoint_list.push_back(wp);
            }
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionHandlerNode>());
  rclcpp::shutdown();
  return 0;
}
