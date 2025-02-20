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
#include "mission_classes/m3.cpp"
#include "mission_classes/m4.cpp"
#include "mission_classes/m5.cpp"
#include "mission_classes/m6.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MissionHandlerNode : public rclcpp::Node {
    public:
        MissionHandlerNode(): Node("mission_handler_node"){
            if(!this->has_parameter("task_schedule")){
                this->declare_parameter("task_schedule", rclcpp::PARAMETER_INTEGER_ARRAY);
            }
            std::vector<int64_t> task_schedule_og = this->get_parameter("task_schedule").as_integer_array();
            if(task_schedule_og.size() > 0){
                task_schedule.clear();
                for(int i = 0 ; i < task_schedule_og.size() ; i++){
                    std::cout << task_schedule_og[i] << std::endl;
                    task_schedule.push_back(int(task_schedule_og[i]));
                }
            }
            
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

            wp_arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/usv/wp_arrived", 1,
                [this](const std_msgs::msg::Bool &msg) {
                    update_params.wp_arrived = msg.data;
                    update_params.docking_color_choice = 1;
                });

            vessel_sub = this->create_subscription<std_msgs::msg::Bool>(
                "/usv/vessel_detected", 1,
                [this](const std_msgs::msg::Bool &msg) {
                    vessel_detected = msg.data;
                });

            racquetball_sub = this->create_subscription<std_msgs::msg::Bool>(
                "/usv/shoot_racquetball", 1,
                [this](const std_msgs::msg::Bool &msg) {
                    shoot_racquetball();
                });

            id.data = 0;
            vtec = std::make_shared<M0>();
            vtec->set_status(1);    // Initially reached wp, to enable new mission assignment

            mission_id_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/mission/id", 10);
            mission_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/mission/state", 10);
            mission_status_pub_ = this->create_publisher<std_msgs::msg::Int8>("/usv/mission/status", 10);
            wp_pub_ = this->create_publisher<usv_interfaces::msg::WaypointList>("/usv/goals", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&MissionHandlerNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr auto_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrived_sub_;
        rclcpp::Subscription<usv_interfaces::msg::ObjectList>::SharedPtr object_list_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wp_arrived_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vessel_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr racquetball_sub;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_state_pub_, mission_status_pub_, mission_id_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr desired_pivot_pub_;
        rclcpp::Publisher<usv_interfaces::msg::WaypointList>::SharedPtr wp_pub_;

        usv_interfaces::msg::Object obj;
        usv_interfaces::msg::WaypointList wp_list;
        std_msgs::msg::Int8 id, state, status;
        std_msgs::msg::Bool pivot, arrived;
        std_msgs::msg::UInt16 auto_mode;
        std::vector<Obstacle> obs_v;

        USVOutput feedback;
        Eigen::Vector3f pose;
        USVUpdate update_params;
        
        std::shared_ptr<Mission> vtec;

        bool wp_arrived{false}, moving{false};
        double goal_dist{0.0};
        double x_diff{0.0}, y_diff{0.0};
        int yellow_found{0}, black_found{0}, pivots_to_do{-1}, pivots_done{0};
        double pivot_goal{-4};
        double h_acum{0.0}, last_h{0.0};
        int mission_command{0};
        
        std::vector<usv_interfaces::msg::Waypoint> goals;
        std::vector<int> task_schedule{2,1,3};

        bool vessel_detected{false};

        // Check for all tasks if their starting point has been found, get first in schedule
        int get_next_known_id(){
            for(int i = 0 ; i < task_schedule.size() ; i++){
                std::string str_id = "m" + std::to_string(task_schedule[i]);
                if(!this->has_parameter(str_id + ".pose")){
                    this->declare_parameter(str_id + ".pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
                }
                std::vector<double> tmp_task_pose = this->get_parameter(str_id + ".pose").as_double_array();
                if(tmp_task_pose.size() == 3){
                    return task_schedule[i];
                }
            }
            return 0;
        }

        // Check if task id should be updated.
        void check_mission_jump(){
            // Check if vessel is detected
            if (vessel_detected == true){
                // Jumpt to mission 5
                update_mission_id(5);
                return;
            }

            if(task_schedule.size() > 0 && status.data){    // If task is completed
                int suitable_id = get_next_known_id();  // Find next reachable task from schedule
                if(!suitable_id){
                    std::cout << "EXPLORE" << std::endl;
                    // TODO: No mission init pose found, explore to finish it.
                    // TODO: id.data = 0; // ID 0 means 'travelling' ...
                    return;
                }
                if(id.data == 0){   // If just arrived to task
                    update_mission_id(suitable_id); // Update state machine
                    task_schedule.erase(
                        std::remove(task_schedule.begin(), task_schedule.end(), suitable_id),
                        task_schedule.end());   // Remove task from schedule
                } else {
                    // Travel to the beginning of next task
                    std::vector<double> travel_arr;
                    std::string str_id = "m" + std::to_string(suitable_id);
                    travel_arr = this->get_parameter(str_id + ".pose").as_double_array();
                    Eigen::Vector3f travel_wp{travel_arr[0], travel_arr[1], travel_arr[2]};
                    std::vector<Eigen::Vector3f> travel_wp_vec{travel_wp};
                    set_goals(travel_wp_vec);
                    wp_pub_->publish(wp_list);
                    update_mission_id(0);   // Change mission id to 'travelling'
                }
            }
        }

        void update_mission_id(int mission_command){

            switch(mission_command){
                case 1:
                    vtec = std::make_shared<M1>();
                    break;
                case 2:
                    vtec = std::make_shared<M2>();
                    break;
                case 3:
                    vtec = std::make_shared<M3>();
                    break;
                case 4:
                    vtec = std::make_shared<M4>();
                    break;
                case 5:
                    vtec = std::make_shared<M5>();
                    break;

                case 6:
                    vtec = std::make_shared<M6>();
                    break;
                default:
                    vtec = std::make_shared<M0>();
                    break;
            }
            
            vtec->re_init();

        }

        void timer_callback() {
            check_mission_jump();

            feedback = vtec->update(pose, update_params);
            
            if(feedback.goals.size() > 0){
                set_goals(feedback.goals);
                wp_pub_->publish(wp_list);
            }

            state.data = feedback.state;
            status.data = feedback.status;
            id.data = vtec->get_id();

            // RCLCPP_INFO(get_logger(), "WP LIST SIZE: %d", wp_list.waypoint_list.size());
            
            mission_id_pub_->publish(id);
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
                wp.theta = vec[i](2);
                wp_list.waypoint_list.push_back(wp);
            }
        }

        void shoot_racquetball(){
            // TODO: Implement racquetball shooting
            std::cout << "Shooting racquetball" << std::endl;
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionHandlerNode>());
  rclcpp::shutdown();
  return 0;
}
