#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("usv_tf2_frame_publisher")
  {
    // Declare and acquire `usv_name` parameter
    usv_name_ = this->declare_parameter<std::string>("usv_name", "usv");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a usv{1}{2}/pose topic and call handle_usv_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << usv_name_.c_str() << "/state/pose";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_usv_pose, this, std::placeholders::_1));

    pose_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "usv/pose_path", 10);
    pose_stamped_tmp_.header.frame_id = "world";
    pose_path.header.frame_id = "world";
    pose_path.header.stamp = FramePublisher::now();
    
  }

private:
  void handle_usv_pose(const std::shared_ptr<geometry_msgs::msg::Pose2D> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = usv_name_.c_str();

    // usv only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, usv can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    pose_stamped_tmp_.pose.position.x = msg->x;
    pose_stamped_tmp_.pose.position.y = msg->y;
    pose_path.poses.push_back(pose_stamped_tmp_);
    
    // Erase when path is too long
    if(pose_path.poses.size() > 5000){
      pose_path.poses.erase(pose_path.poses.begin(), pose_path.poses.begin()+1);
    }

    pose_path_pub_->publish(pose_path);

  }

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub_;
  geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
  nav_msgs::msg::Path pose_path;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string usv_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
