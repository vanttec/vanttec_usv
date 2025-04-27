#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gz/transport/Node.hh"
#include "gz/msgs/double.pb.h"
#include "gz/msgs/pointcloud_packed.pb.h"
#include "gz/msgs/header.pb.h"
#include "gz_ros_conversions.hpp"

class CustomBridge : public rclcpp::Node 
{
public:
    CustomBridge() : Node("custom_bridge") 
    {
        // Ros2 subscribers 
        left_ros_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/model/vtec_s3/joint/left_engine_propeller_joint/cmd_thrust",
            10,
            std::bind(&CustomBridge::left_callback, this, std::placeholders::_1));

        right_ros_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/model/vtec_s3/joint/right_engine_propeller_joint/cmd_thrust",
            10,
            std::bind(&CustomBridge::right_callback, this, std::placeholders::_1));

        lidar_ros_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/velodyne_points", 10);

        // Gazebo publishers 
        left_gz_pub = gz_node.Advertise<gz::msgs::Double>(
            "/model/vtec_s3/joint/left_engine_propeller_joint/cmd_thrust");

        if (!left_gz_pub) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create left Gazebo publisher");
            return;
        }

        right_gz_pub = gz_node.Advertise<gz::msgs::Double>(
          "/model/vtec_s3/joint/right_engine_propeller_joint/cmd_thrust");

        if (!right_gz_pub) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create right Gazebo publisher");
            return;
        }

        // Gazebo subscribers
        if (!gz_node.Subscribe("/lidar/points", &CustomBridge::gz_to_ros_lidar_points_callback, this)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create Gazebo subscription");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Custom bridge initialized");
    }

private:
    void left_callback(const std_msgs::msg::Float64::SharedPtr msg) 
    {
        gz::msgs::Double gz_msg;
        gz_msg.set_data(msg->data);
        
        if (!left_gz_pub.Publish(gz_msg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish to Gazebo");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Published to Gazebo: %f", msg->data);
        }
    }

    void right_callback(const std_msgs::msg::Float64::SharedPtr msg) 
    {
        gz::msgs::Double gz_msg;
        gz_msg.set_data(msg->data);
        
        if (!right_gz_pub.Publish(gz_msg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish to Gazebo");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Published to Gazebo: %f", msg->data);
        }
    }

    void gz_to_ros_lidar_points_callback(const gz::msgs::PointCloudPacked& gz_msg) 
    {
        sensor_msgs::msg::PointCloud2 ros_msg;
        
        gz_ros_conversions::convert_gz_to_ros(gz_msg, ros_msg);

        // Publish the message
        lidar_ros_pub->publish(std::move(ros_msg));
        // RCLCPP_INFO(this->get_logger(), "Published point cloud with %d points", gz_msg.width() * gz_msg.height());
    }


    gz::transport::Node gz_node;
  
    // Gazebo publishers
    gz::transport::Node::Publisher left_gz_pub;
    gz::transport::Node::Publisher right_gz_pub;
  
    // Ros2 publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_ros_pub;
    // Ros2 subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_ros_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_ros_sub;

};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
