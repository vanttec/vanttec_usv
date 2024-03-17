#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstaclePublisherNode : public rclcpp::Node {
    public:
        ObstaclePublisherNode(): Node("obstacle_publisher_node") {            
            // object_list_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/objects_docking", 10);
            object_list_pub_ = this->create_publisher<usv_interfaces::msg::ObjectList>("/objects", 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);

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
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub_;
        usv_interfaces::msg::Object obj;
        geometry_msgs::msg::Pose2D pose;
        visualization_msgs::msg::Marker marker;

        // /*
        // =========================
        // ||  MANDATORY MISSION  ||
        // =========================
        // */
        // std::vector<std::vector<float>> obsAbsPose {
        // //Red buoys
        // {-3.5,-3.0}, {-19.0, -4.1},
        // //Green buoys
        // {-3.7, 0.9}, {-17.0, -1.0},
        // };
        // std::vector<int> obsColors = {0,0,1,1};        
        
        // /*
        // =======================
        // ||  FOLLOW THE PATH  ||
        // =======================
        // */
        // // TEST #1: CHECKED
        // std::vector<std::vector<float>> obsAbsPose {
        // //Red buoys
        // {3.5,-3.0}, {9.0, -4.1}, {13.0, -2.0},
        // //Green buoys
        // {3.7, 0.9}, {8.8, -1.0}, {14.0, 1.23},
        // //Yellow buoys
        // {6.7,-2.0},
        // //Black buoys
        // {3.2, -1.5}, {13.0,0.5}
        // };
        // std::vector<int> obsColors = {0,0,0,1,1,1,3,4,4};
        // std::vector<std::string> obsType = {"round","round","round","round","round","round","round","round","round"};

        // // TEST #2: TODO
        // std::vector<std::vector<float>> obsAbsPose {
        // //Red buoys
        // {-3.0,3.5}, {-4.1, 9.0}, {-2.0, 13.0},
        // //Green buoys
        // {0.9, 3.7}, {0.0, 8.8}, {1.23, 14.0},
        // //Yellow buoys
        // {-2.5, 7.5},
        // //Black buoys
        // {-1.5, 2.0}, {0.5, 13.0}
        // };
        // std::vector<int> obsColors = {0,0,0,1,1,1,3,4,4};

        // // TEST #2: "EASIER"
        // std::vector<std::vector<float>> obsAbsPose {
        // //Red buoys
        // {1,1},{4,3},{7,3},{10,0},{7,-2},{4,-2},
        // //Green buoys
        // {-1,3},{3,5},{8,5},{12,0},{8,-4},{4,-5}, 
        // };
        // std::vector<int> obsColors = {0,0,0,0,0,0,1,1,1,1,1,1};

        // // TEST #3: "temp, volver al 2 ahorita yaaa"
        // std::vector<std::vector<float>> obsAbsPose {
        // //Red buoys
        // {1,1},{3,5},
        // //Green buoys
        // {-1,3},{4,3},
        // };
        // std::vector<int> obsColors = {0,0,1,1};

        /*
        =======================
        ||  SPEED CHALLENGE  ||
        =======================
        */

    //    // Case 1
    //     std::vector<std::vector<float>> obsAbsPose {
    //     //Red buoys
    //     {-1.5,-3.0},
    //     //Green buoys
    //     {-1.5, 3.0},
    //     //Blue buoys
    //     {-13.0, 10.0},
    //     //Yellow buoys
    //     {-15, 0},
    //     };
    //     std::vector<int> obsColors = {0,1,2,3};    
    //     std::vector<std::string> obsType = {"round", "round", "round", "round"};

       // Case 2
        std::vector<std::vector<float>> obsAbsPose {
        //Red buoys
        {8,9},
        //Green buoys
        {7, 10.5},
        //Blue buoys
        {-13.0, 10.0},
        //Yellow buoys
        {-2, 0},
        };
        std::vector<int> obsColors = {0,1,2,3};    
        std::vector<std::string> obsType = {"round", "round", "round", "round"};


        // /*
        // ============================
        // ||  DOCKING CHALLENGE HUH ||
        // ============================
        // */
        // std::vector<std::vector<float>> obsAbsPose {
        // //BLUE
        // {9,19},
        // //GREEN
        // {8.5,14},
        // //DUCK
        // {9.5,24},
        // };
        // std::vector<int> obsColors = {3,2,1};    
        // std::vector<std::string> obsType = {"duck", "plus", "circle"}; // circle, duck, plus, triangle
        

        std::vector<std::vector<float>> obsRelPose;
        float x_t{0.0}, y_t{0.0}, d_t{0.0};

        void timer_callback() {
            usv_interfaces::msg::ObjectList obj_list;
            obj_list = fill_obj_list();
            object_list_pub_->publish(obj_list);

            visualization_msgs::msg::MarkerArray marker_arr;
            for(int i = 0 ; i < this->obsAbsPose.size() ; i++){
                switch(this->obsColors[i]){
                    case 0:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1).g(0).b(0).a(1); 
                        break;     
                    case 1:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(1).b(0).a(1); 
                        break;     
                    case 2:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(1).a(1); 
                        break;     
                    case 3:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1).g(1).b(0).a(1); 
                        break;     
                    case 4:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(0).a(1); 
                        break;     
                    default:
                        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0).g(0).b(0).a(0); 
                        break;     
                }
                marker.header.frame_id = "world";
                // marker.ns = i;
                marker.id = i;
                marker.type = 2;
                marker.action = 0;
                marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.5).y(0.5).z(0.5); 
                marker.pose.position.x = this->obsAbsPose[i][0];
                marker.pose.position.y = this->obsAbsPose[i][1];
                marker.pose.position.z = 0.0;
                marker_arr.markers.push_back(marker);
            }
            marker_pub_->publish(marker_arr);
        }        

        usv_interfaces::msg::ObjectList fill_obj_list(){
            usv_interfaces::msg::ObjectList o_l;
            obsRelPose = obsAbsPose;
            Eigen::Matrix3f rotM;
            Eigen::Vector3f po, poseR;
            rotM << std::cos(-this->pose.theta), - std::sin(-this->pose.theta), 0, 
                    std::sin(-this->pose.theta), std::cos(-this->pose.theta), 0, 
                    0, 0, 1;

            for(int i = 0 ; i < obsAbsPose.size() ; i++){
                this->x_t = obsAbsPose[i][0] - this->pose.x;
                this->y_t = obsAbsPose[i][1] - this->pose.y;
                this->d_t = std::sqrt(std::pow(this->x_t,2) + std::pow(this->y_t,2));
                // std::cout << this->d_t << "x_t: " << this->x_t << "y_t: " << this->y_t << std::endl;

                po << x_t, y_t, 0;
                poseR = rotM*po;

                obsRelPose[i][0] = poseR[0];
                obsRelPose[i][1] = poseR[1];

                if(this->obsRelPose[i][0] >= 0 && this->d_t < 20){
                    obj.x = obsRelPose[i][0];
                    obj.y = obsRelPose[i][1];
                    obj.type = obsType[i];
                    obj.color = obsColors[i];
                    o_l.obj_list.push_back(obj);
                }
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