#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class Bag2ImageNode : public rclcpp::Node
{
public:
  Bag2ImageNode()
  : Node("bag2image_node"), frame_count_(0)
  {
    // Create a parameter for output directory with default value
    this->declare_parameter("output_dir", "output_frames");
    output_dir_ = this->get_parameter("output_dir").as_string();

    // Create directory if it doesn't exist
    std::string command = "mkdir -p " + output_dir_;
    system(command.c_str());
    
    RCLCPP_INFO(this->get_logger(), "Saving frames to: %s", output_dir_.c_str());

    std::string topic_name = "/bebblebrox/video";

    // Create subscription with the image callback
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10, 
      std::bind(&Bag2ImageNode::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // Convert ROS Image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      
      // Generate filename with frame count
      std::string filename = output_dir_ + "/frame_" + 
                             std::to_string(frame_count_++) + ".png";
      
      // Save the image
      cv::imwrite(filename, cv_ptr->image);
      
      RCLCPP_INFO(this->get_logger(), "Saved frame %d", frame_count_ - 1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::string output_dir_;
  int frame_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Bag2ImageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}