#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <map>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleViewerNode : public rclcpp::Node {
    public:
        ObstacleViewerNode(): Node("obstacle_viewer_node") {

            obstacle_map_pub_ = this->create_publisher<nav_msgs::msg::GridCells>("/usv/obstacle_map", 10);
            obstacle_margins_map_pub_ = this->create_publisher<nav_msgs::msg::GridCells>("/usv/obstacle_margins_map", 10);
            
            marker_arr_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "/marker_array", 10,
                [this](const visualization_msgs::msg::MarkerArray &msg) {
                    bool found;
                    for(int i = 0 ; i < msg.markers.size() ; i++){
                        found = false;
                        for(int j = 0 ; j < obstacle_map_msg.cells.size() ; j++){
                            if(is_same_point(msg.markers[i].pose.position, obstacle_map_msg.cells[j])){
                                found = true;
                            }
                        }
                        if(!found){
                            obstacle_map_msg.cells.push_back(msg.markers[i].pose.position);
                            std::vector<geometry_msgs::msg::Point> margin_points = get_margin_points(obstacle_map_msg.cells[i], 1., obs_size);
                            obstacle_margins_map_msg.cells.insert(obstacle_margins_map_msg.cells.end(), margin_points.begin(), margin_points.end());
                        }

                    }
                });

            obstacle_map_msg.header.frame_id = "world";
            obstacle_map_msg.cell_height = obs_size;
            obstacle_map_msg.cell_width = obs_size;
            obstacle_margins_map_msg = obstacle_map_msg;

            timer_ = this->create_wall_timer(100ms, std::bind(&ObstacleViewerNode::timer_callback, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr obstacle_map_pub_, obstacle_margins_map_pub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arr_sub_;

        nav_msgs::msg::GridCells obstacle_map_msg, obstacle_margins_map_msg;

        double obs_size = 0.5;

        std::vector<geometry_msgs::msg::Point> get_margin_points(geometry_msgs::msg::Point p, double offset, double resolution){
            std::vector<geometry_msgs::msg::Point> out;
            for(int i = -1 ; i < 2 ; i++){
                for(int j = -1 ; j < 2 ; j++){
                    out.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().x((p.x+i*obs_size)).y((p.y+j*obs_size)).z(0.));
                        // std::cout << p.x+i << ", " << p.y + j << ", " << std::endl;
                        std::cout << "p.x " << p.x << ",i " << i << ", final " << p.x+i*obs_size << std::endl;
                        std::cout << "p.y " << p.y << ",j " << j << ", final " << p.y+j*obs_size << std::endl;
                }
            }

            // The same but drawing a circle with the offset for radius, more computational cost however
            // for(double i = -offset ; i < offset ; i++){
            //     for(double j = -offset ; j < offset ; j++){
            //         if(sqrt(i*i + j*j) <= offset){
            //             out.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().x(floor(p.x+i)).y(floor(p.y+j)).z(0.));
            //             std::cout << p.x+i << ", " << p.y + j << ", " << std::endl;
            //         }
            //     }
            // }
            return out;
        }

        bool is_same_point(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2){
            if(sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2)) < 1.)
                return true;
            return false;
        }

        void timer_callback() {
            obstacle_map_pub_->publish(obstacle_map_msg);
            obstacle_margins_map_pub_->publish(obstacle_margins_map_msg);
        }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleViewerNode>());
  rclcpp::shutdown();
  return 0;
}
