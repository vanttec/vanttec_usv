#include <cmath>
#include <cstdlib>
#include <memory>

#include <rclcpp/time.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "sbg_ros2_driver/msg/SbgGpsPos.msg"



using namespace std;




class Heartbeat : public rclcpp::Node
{
  public:
    Heartbeat()
    : Node("minimal_publisher")
    {
       
      subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&Heartbeat::timer_callback, this, std::placeholders::_1));
      

      

      
    }

  private:

    
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
      {
      }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}