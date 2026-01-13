#include "usv_interfaces/rviz_tools/goal_to_tool.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/arrow.hpp>

namespace usv_interfaces
{
namespace rviz_tools
{

GoalToTool::GoalToTool()
{
  shortcut_key_ = 't';
  setName("GoalTo");
}

void GoalToTool::onPoseSet(double x, double y, double theta)
{
  arrow_->setColor(1.0, 0.0, 0.0, 1.0); // RGBA
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = context_->getClock()->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();
  
  auto pub = context_->getRosNodeAbstraction().lock()->get_raw_node()->
    template create_publisher<geometry_msgs::msg::PoseStamped>("/goal_to", 1);
  pub->publish(pose);
}

}  // namespace rviz_tools
}  // namespace usv_interfaces

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(usv_interfaces::rviz_tools::GoalToTool, rviz_common::Tool)