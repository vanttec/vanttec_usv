#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <std_msgs/msg/detail/int8__struct.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int8.hpp"

class Heartbeat : public rclcpp::Node
{
  public:
    Heartbeat()
    : Node("robocommand_heartbeat")
    {
       subscription_gps = this->create_subscription<sbg_driver::msg::SbgGpsPos>(
            "/sbg/gps_pos", 
            10, 
            std::bind(&Heartbeat::sbgGps_callback, this, std::placeholders::_1)
        );

      subscription_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/imu/velocity", 
            10, 
            std::bind(&Heartbeat::imuVel_callback, this, std::placeholders::_1)
        );

      subscription_heading = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(&Heartbeat::imuHeading_callback,this, std::placeholders::_1)
      );

      subscription_mission_id = this->create_subscription<std_msgs::msg::Int8>(
        "/usv/mission/id", 
        10, 
        std::bind(&Heartbeat::missionId_callback,this,std::placeholders::_1)  
      );

      
    }

  private:

    struct heartbeatInfo{
      double latitude = 0;
      double longitude = 0;
      float spd_mps = 0;
      float heading_deg = 0;
      int robot_state = 0;
      int current_task = 0;
    };

    heartbeatInfo currentData;

    void sbgGps_callback(const sbg_driver::msg::SbgGpsPos::SharedPtr msg)
    {
        // Accessing latitude, longitude, and altitude
        RCLCPP_INFO(this->get_logger(), "Received GPS Pos -> Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                    msg->latitude, 
                    msg->longitude, 
                    msg->altitude);

        currentData.latitude = msg->latitude;
        currentData.longitude = msg->longitude;
    }

    void imuVel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {

        double velocity = sqrt(pow(msg->twist.linear.x,2) + pow(msg->twist.linear.y,2) + pow(msg->twist.linear.z,2));

        currentData.spd_mps = velocity;
    }

    void imuHeading_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        double heading = msg->orientation.w;

        // Conversion to degrees
        currentData.heading_deg = (heading*180)/M_PI;
    }

    void missionId_callback(const std_msgs::msg::Int8 msg)
    {
      currentData.current_task = msg.data;
    }


    rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_heading;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_mission_id;
    

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}