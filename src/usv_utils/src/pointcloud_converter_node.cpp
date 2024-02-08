#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <functional>
#include <chrono>
using namespace std::chrono_literals;


class PointcloudConverterNode : public rclcpp::Node {
public:
  PointcloudConverterNode() : Node("pointcloud_converter_node"){
    using namespace std::placeholders;

    newPCPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc", 10);

    pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/points", 10,
      std::bind(&PointcloudConverterNode::pc_callback, this, _1));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&PointcloudConverterNode::timer_callback, this));
  }

protected:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr newPCPub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcSub;

  sensor_msgs::msg::PointCloud2 new_pc;

  void timer_callback() {
    // RCLCPP_INFO(get_logger(), "");
  }        
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudConverterNode>());
  rclcpp::shutdown();
  return 0;
}