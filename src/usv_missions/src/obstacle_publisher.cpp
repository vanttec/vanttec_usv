#include <chrono>
#include <functional>
#include <memory>
#include <string>
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
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

struct MarkerProps {
  int type;
  double x, y, z, z_trans;
};

class ObstaclePublisherNode : public rclcpp::Node {
    public:
        ObstaclePublisherNode(): Node("obstacle_publisher_node") {

            this->declare_parameter("dynamic_obstacles", rclcpp::PARAMETER_BOOL);
            dynamic_obs = this->get_parameter("dynamic_obstacles").as_bool();

            this->declare_parameter("global_tasks", rclcpp::PARAMETER_STRING_ARRAY);
            update_params();

            // Append dynamic obstacles
            if(dynamic_obs){
                for(int i = 0 ; i < dyn_obs_n; i++){
                    usv_interfaces::msg::Object obj;
                    visualization_msgs::msg::Marker marker;

                    double x = dyn_obs[i][0];
                    double y = dyn_obs[i][1];
                    double v_x = dyn_obs[i][2];
                    double v_y = dyn_obs[i][3];
                    int64_t color = 3;
                    std::string type = "marker";

                    obj = usv_interfaces::build<usv_interfaces::msg::Object>().x(x).y(y).v_x(v_x).v_y(v_y).color(color).type(type);
                    dyn_obs_id[i] = object_list_global.obj_list.size();
                    object_list_global.obj_list.push_back(obj);

                    int r{color_list[color][0]},
                        g{color_list[color][1]},
                        b{color_list[color][2]},
                        a{color_list[color][3]};

                    marker.header.frame_id = "world";
                    marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
                    marker.action = 0;
                    marker.id = marker_arr.markers.size();
                    marker.type = marker_type[type].type;
                    marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().
                                    x(marker_type[type].x).y(marker_type[type].y).z(marker_type[type].z); 
                    marker.pose.position.x = x;
                    marker.pose.position.y = y;
                    marker.pose.position.z = marker_type[type].z_trans;
                    marker_arr.markers.push_back(marker);
                }
            }

            object_list_global_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/obj_list_global", 10);
            object_list_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/obj_list", 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);
            on_watch_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/on_watch", 10);

            pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/usv/state/pose", 10,
                [this](const geometry_msgs::msg::Pose2D &msg){  pose = msg; });

            docking_rotate_srv_ = this->create_service<std_srvs::srv::Empty>("/usv/obstacle_publisher/docking/rotate",
                std::bind(&ObstaclePublisherNode::docking_rotate_srv, this, _1, _2));

            timer_ = this->create_wall_timer(
            10ms, std::bind(&ObstaclePublisherNode::timer_callback, this));

        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr object_list_pub_, object_list_global_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_, on_watch_marker_pub_;

        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr docking_rotate_srv_;

        usv_interfaces::msg::ObjectList object_list, object_list_global;
        visualization_msgs::msg::MarkerArray marker_arr, on_watch_arr;
        geometry_msgs::msg::Pose2D pose;
        bool dynamic_obs;

        static const int dyn_obs_n{11};
        double dyn_obs[dyn_obs_n][4]{
            {20., 1., -0.125, 0.1},
            {0., -3., 0.135, 0.2},
            {4., 2., -0.125, 0.15},

            {9., 0., 0.15, 0.4},
            {8., 3., -0.35, 0.},
            {10., -5., 0, 0.123},
            {1., -5., 0, 0.23},

            {7., 2., 0, 0.28},
            {10., 0., 0, -0.28},
            {14., 0., 0, -0.28},
            // {17., 3., 0, -0.1323},
            {20., 0., 0, -0.1},
        };
        int dyn_obs_id[dyn_obs_n];

        int color_list[6][4]{
            {1,0,0,1},
            {0,1,0,1},
            {0,0,1,1},
            {1,1,0,1},
            {0,0,0,1},
            {0,0,0,0}
        };

        std::map<std::string, MarkerProps> marker_type = {
            {"round", MarkerProps{2, 0.5, 0.5, 0.5, 0}},
            {"boat", MarkerProps{2, 1.0, 1.0, 1.0, 0}},
            {"marker", MarkerProps{3, 0.5, 0.5, 1, 0.25}},
            {"picture", MarkerProps{1, 0.5, 0.5, 0.5, 0.25}},
        };
                
        void timer_callback() {
            fill_local_list();
            if(dynamic_obs){
                update_dyn();
            }
    
            object_list_global_pub_->publish(object_list_global);
            object_list_pub_->publish(object_list);
            marker_pub_->publish(marker_arr);
            on_watch_marker_pub_->publish(on_watch_arr);
        }

        void docking_rotate_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

            // this->declare_parameter("mission_choice", rclcpp::PARAMETER_STRING);
            std::string mission_prefix = this->get_parameter("m3.name").as_string();
            
            int docking_versions = 3;

            // int new_docking_version = std::stoi(mission_prefix.back())%(docking_versions) + 1;
            int new_docking_version = std::stoi(std::string(1, mission_prefix.back())) % docking_versions + 1;

            std::string new_prefix = "m3_v"+std::to_string(new_docking_version);
            std::cout << "new prefix: " << new_prefix << std::endl;
            
            // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("mission_choice", new_prefix)};
            // this->set_parameters(all_new_parameters);
            rclcpp::Parameter updated_param("m3.name", new_prefix);
            this->set_parameter(updated_param);
            update_params();
        }

        void update_dyn(){
            double dt = 0.01;
            for(int i = 0 ; i < dyn_obs_n ; i++){
                dyn_obs[i][0] += dyn_obs[i][2]*dt;
                dyn_obs[i][1] += dyn_obs[i][3]*dt;

                if(dyn_obs[i][0] < 0){
                    dyn_obs[i][2] = fabs(dyn_obs[i][2]);
                } else if(dyn_obs[i][0] > 25){
                    dyn_obs[i][2] = -fabs(dyn_obs[i][2]);
                }
                if(dyn_obs[i][1] < -5){
                    dyn_obs[i][3] = fabs(dyn_obs[i][3]);
                } else if(dyn_obs[i][1] > 5){
                    dyn_obs[i][3] = -fabs(dyn_obs[i][3]);
                }

                object_list_global.obj_list[dyn_obs_id[i]].x = dyn_obs[i][0];
                object_list_global.obj_list[dyn_obs_id[i]].y = dyn_obs[i][1];
                object_list_global.obj_list[dyn_obs_id[i]].v_x = dyn_obs[i][2];
                object_list_global.obj_list[dyn_obs_id[i]].v_y = dyn_obs[i][3];
                marker_arr.markers[dyn_obs_id[i]].pose.position.x = dyn_obs[i][0];
                marker_arr.markers[dyn_obs_id[i]].pose.position.y = dyn_obs[i][1];
            }
        }

        void fill_local_list(){
            Eigen::Matrix3f rotM;
            rotM << std::cos(-pose.theta), - std::sin(-pose.theta), 0, 
                    std::sin(-pose.theta), std::cos(-pose.theta), 0, 
                    0, 0, 1;

            Eigen::Vector3f po, poseR;
            double x_tmp, y_tmp, d_tmp;

            usv_interfaces::msg::Object obj;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";

            object_list.obj_list.clear();
            on_watch_arr.markers.clear();

            for(int i = 0 ; i < object_list_global.obj_list.size() ; i++){
                x_tmp = object_list_global.obj_list[i].x - pose.x;
                y_tmp = object_list_global.obj_list[i].y - pose.y;
                d_tmp = std::sqrt(std::pow(x_tmp,2) + std::pow(y_tmp,2));
                po << x_tmp, y_tmp, 0;
                poseR = rotM*po;
                
                if(poseR(0) > 0 && d_tmp < 10 && std::fabs(std::atan2(poseR(1),poseR(0))) < 1){
                    obj.x = poseR(0);
                    obj.y = poseR(1);
                    obj.type = object_list_global.obj_list[i].type;
                    obj.color = object_list_global.obj_list[i].color;
                    object_list.obj_list.push_back(obj);

                    int r{color_list[obj.color][0]},
                        g{color_list[obj.color][1]},
                        b{color_list[obj.color][2]};
                    marker.action = 0;
                    marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(0.8);
                    marker.color.a = 0.5;
                    marker.type = marker_type[obj.type].type;
                    marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().
                                    x(marker_type[obj.type].x * 2.0).
                                    y(marker_type[obj.type].y * 2.0).
                                    z(marker_type[obj.type].z * 2.0); 
                    marker.pose.position.x = object_list_global.obj_list[i].x;
                    marker.pose.position.y = object_list_global.obj_list[i].y;
                    marker.pose.position.z = marker_type[obj.type].z_trans;
                } else
                    marker.action = 2;

                marker.id = i;
                on_watch_arr.markers.push_back(marker);
            }
        }

        void update_params(){
            std::vector<std::string> global_tasks = this->get_parameter("global_tasks").as_string_array();
            object_list_global.obj_list.clear();
            marker_arr.markers.clear();
            for(std::string &mission_id : global_tasks) {
                if(!this->has_parameter(mission_id + ".name")){
                    this->declare_parameter(mission_id + ".name", rclcpp::PARAMETER_STRING);
                    this->declare_parameter(mission_id + ".pose", rclcpp::PARAMETER_DOUBLE_ARRAY);
                }
                std::string mission_prefix = this->get_parameter(mission_id + ".name").as_string();
                std::vector<double> mission_pose = this->get_parameter(mission_id + ".pose").as_double_array();
                Eigen::Vector3f mission_base_pose{mission_pose[0],mission_pose[1],mission_pose[2]};

                if(!this->has_parameter(mission_prefix + ".obj_names")){
                this->declare_parameter(mission_prefix + ".obj_names", rclcpp::PARAMETER_STRING_ARRAY);
                }
                std::vector<std::string> n = this->get_parameter(mission_prefix + ".obj_names").as_string_array();

                for(std::string &name : n) {
                    usv_interfaces::msg::Object obj;
                    visualization_msgs::msg::Marker marker;

                    if(!this->has_parameter(mission_prefix + ".obj_info." + name + ".x")){
                    this->declare_parameter(mission_prefix + ".obj_info." + name + ".x", rclcpp::PARAMETER_DOUBLE);
                    this->declare_parameter(mission_prefix + ".obj_info." + name + ".y", rclcpp::PARAMETER_DOUBLE);
                    this->declare_parameter(mission_prefix + ".obj_info." + name + ".color", rclcpp::PARAMETER_INTEGER);
                    this->declare_parameter(mission_prefix + ".obj_info." + name + ".type", rclcpp::PARAMETER_STRING);
                    }

                    double x = this->get_parameter(mission_prefix + ".obj_info." + name + ".x").as_double();
                    double y = this->get_parameter(mission_prefix + ".obj_info." + name + ".y").as_double();

                    Eigen::Vector3f tf_obs_pose = tf_body_to_world(mission_base_pose, Eigen::Vector3f{x, y, 0.0});
                    x = tf_obs_pose(0);
                    y = tf_obs_pose(1);

                    int64_t color = this->get_parameter(mission_prefix + ".obj_info." + name + ".color").as_int();
                    std::string type = this->get_parameter(mission_prefix + ".obj_info." + name + ".type").as_string();

                    obj = usv_interfaces::build<usv_interfaces::msg::Object>().x(x).y(y).v_x(0.).v_y(0.).color(color).type(type);
                    object_list_global.obj_list.push_back(obj);

                    int r{color_list[color][0]},
                        g{color_list[color][1]},
                        b{color_list[color][2]},
                        a{color_list[color][3]};

                    marker.header.frame_id = "world";
                    marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
                    marker.action = 0;
                    marker.id = marker_arr.markers.size();
                    marker.type = marker_type[type].type;
                    marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().
                                    x(marker_type[type].x).y(marker_type[type].y).z(marker_type[type].z); 
                    marker.pose.position.x = x;
                    marker.pose.position.y = y;
                    marker.pose.position.z = marker_type[type].z_trans;
                    marker_arr.markers.push_back(marker);
                }
            }
        }

        Eigen::Vector3f tf_body_to_world(Eigen::Vector3f pose_, Eigen::Vector3f relative){
        Eigen::Matrix3f rotM;
        rotM << std::cos(pose_(2)), - std::sin(pose_(2)), 0, 
                std::sin(pose_(2)), std::cos(pose_(2)), 0, 
                0, 0, 1;
        return pose_ + rotM * relative;
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
