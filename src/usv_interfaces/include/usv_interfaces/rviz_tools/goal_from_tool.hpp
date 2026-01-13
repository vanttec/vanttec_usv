#ifndef USV_INTERFACES__RVIZ_TOOLS__GOAL_FROM_TOOL_HPP_
#define USV_INTERFACES__RVIZ_TOOLS__GOAL_FROM_TOOL_HPP_

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace usv_interfaces
{
namespace rviz_tools
{

class GoalFromTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalFromTool();
  ~GoalFromTool() override = default;
  
protected:
  void onPoseSet(double x, double y, double theta) override;
};

}  // namespace rviz_tools
}  // namespace usv_interfaces

#endif  // USV_INTERFACES__RVIZ_TOOLS__GOAL_FROM_TOOL_HPP_