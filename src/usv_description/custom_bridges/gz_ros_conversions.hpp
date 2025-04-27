#ifndef GZ_ROS_CONVERSIONS_HPP_
#define GZ_ROS_CONVERSIONS_HPP_

#include <string>

// ROS2 message includes
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"

// Gazebo message includes
#include "gz/msgs/pointcloud_packed.pb.h"
#include "gz/msgs/header.pb.h"
#include "gz/msgs/time.pb.h"

namespace gz_ros_conversions
{

void convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg);

void convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg);

void convert_gz_to_ros(
  const gz::msgs::Time & gz_msg,
  builtin_interfaces::msg::Time & ros_msg);

std::string replace_delimiter(
  const std::string & input,
  const std::string & old_delim,
  const std::string & new_delim);

std::string frame_id_gz_to_ros(const std::string & frame_id);

}  



#endif  // GZ_ROS_CONVERSIONS_HPP_
