#include "usv_interfaces/rviz_tools/center_on_robot_tool.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp>
#include <QKeyEvent>

namespace usv_interfaces
{
namespace rviz_tools
{

CenterOnRobotTool::CenterOnRobotTool()
{
  shortcut_key_ = 'x';
  robot_frame_ = "usv";  // Your robot frame
}

void CenterOnRobotTool::onInitialize()
{
  auto ros_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  setName("Center on Robot");
  setIcon(QIcon());
}

void CenterOnRobotTool::activate()
{
  // Immediately center
  centerOnRobot();
  
  // Auto-deactivate - switch back to the default tool (Interact)
  auto tool_manager = context_->getToolManager();
  if (tool_manager)
  {
    // Get the first tool (usually Interact) and activate it
    tool_manager->setCurrentTool(tool_manager->getTool(0));
  }
}

void CenterOnRobotTool::deactivate()
{
}

int CenterOnRobotTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel * /*panel*/)
{
  // This processes key events when the tool is active
  if (event->key() == Qt::Key_X && event->type() == QEvent::KeyPress)
  {
    centerOnRobot();
    return 1;
  }
  return 0;
}

void CenterOnRobotTool::centerOnRobot()
{
  try
  {
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    
    RCLCPP_INFO(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                "Centering camera on robot. Fixed frame: %s, Robot frame: %s", 
                fixed_frame.c_str(), robot_frame_.c_str());
    
    // Get transform from fixed frame to robot frame
    geometry_msgs::msg::TransformStamped transform = 
      tf_buffer_->lookupTransform(fixed_frame, robot_frame_, tf2::TimePointZero);
    
    // Get the current view controller
    rviz_common::ViewController* view = context_->getViewManager()->getCurrent();
    
    if (view)
    {
      // Set the focal point to the robot's position
      Ogre::Vector3 focal_point(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
      );
      
      RCLCPP_INFO(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                  "Moving camera to position: x=%.2f, y=%.2f, z=%.2f", 
                  focal_point.x, focal_point.y, focal_point.z);
      
      view->lookAt(focal_point);
    }
    else
    {
      RCLCPP_WARN(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                  "No view controller available");
    }
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                "Could not get robot transform: %s", ex.what());
  }
}

}  // namespace rviz_tools
}  // namespace usv_interfaces

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(usv_interfaces::rviz_tools::CenterOnRobotTool, rviz_common::Tool)