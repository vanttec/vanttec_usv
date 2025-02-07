#include "gz_ros_conversions.hpp"

#include <cstring>

namespace gz_ros_conversions
{

void convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();
  ros_msg.is_bigendian = gz_msg.is_bigendian();
  ros_msg.point_step = gz_msg.point_step();
  ros_msg.row_step = gz_msg.row_step();
  ros_msg.is_dense = gz_msg.is_dense();
  ros_msg.data.resize(gz_msg.data().size());
  memcpy(ros_msg.data.data(), gz_msg.data().c_str(), gz_msg.data().size());

  ros_msg.fields.clear();  // Clear existing fields before adding new ones
  for (int i = 0; i < gz_msg.field_size(); ++i) {
    sensor_msgs::msg::PointField pf;
    pf.name = gz_msg.field(i).name();
    pf.count = gz_msg.field(i).count();
    pf.offset = gz_msg.field(i).offset();
    switch (gz_msg.field(i).datatype()) {
      case gz::msgs::PointCloudPacked::Field::INT8:
        pf.datatype = sensor_msgs::msg::PointField::INT8;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT8:
        pf.datatype = sensor_msgs::msg::PointField::UINT8;
        break;
      case gz::msgs::PointCloudPacked::Field::INT16:
        pf.datatype = sensor_msgs::msg::PointField::INT16;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT16:
        pf.datatype = sensor_msgs::msg::PointField::UINT16;
        break;
      case gz::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::msg::PointField::INT32;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::msg::PointField::UINT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT64;
        break;
      default:
        // Handle unknown type as INT8 (you might want to throw an exception instead)
        pf.datatype = sensor_msgs::msg::PointField::INT8;
        break;
    }
    ros_msg.fields.push_back(pf);
  }
}

void convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg)
{
  convert_gz_to_ros(gz_msg.stamp(), ros_msg.stamp);
  ros_msg.frame_id = "";  // Default empty frame_id
  
  for (auto i = 0; i < gz_msg.data_size(); ++i) {
    const auto & pair = gz_msg.data(i);
    if (pair.key() == "frame_id" && pair.value_size() > 0) {
      ros_msg.frame_id = frame_id_gz_to_ros(pair.value(0));
      break;  // Found frame_id, no need to continue
    }
  }
}

void convert_gz_to_ros(
  const gz::msgs::Time & gz_msg,
  builtin_interfaces::msg::Time & ros_msg)
{
  ros_msg.sec = gz_msg.sec();
  ros_msg.nanosec = gz_msg.nsec();
}

std::string replace_delimiter(
  const std::string & input,
  const std::string & old_delim,
  const std::string & new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size()) {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos) {
      output += new_delim;
      last_pos = pos + old_delim.size();
    } else {
      break;  // No more delimiters found
    }
  }

  // Add remaining part of the string if any
  if (last_pos < input.size()) {
    output += input.substr(last_pos);
  }

  return output;
}

std::string frame_id_gz_to_ros(const std::string & frame_id)
{
  return replace_delimiter(frame_id, "::", "/");
}


}  // namespace gz_ros_conversions
