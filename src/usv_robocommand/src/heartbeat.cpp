#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <std_msgs/msg/detail/int8__struct.hpp>
#include <string>
#include <usv_interfaces/msg/detail/system_status__struct.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int8.hpp"
#include "usv_interfaces/msg/system_status.hpp"

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

      subscription_system_status = this->create_subscription<usv_interfaces::msg::SystemStatus>(
        "/usv/status", 
        10, 
        std::bind(&Heartbeat::systemStatus_callback,this,std::placeholders::_1)  
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

    void systemStatus_callback(const usv_interfaces::msg::SystemStatus msg)
    {
      // op_mode != robot_state
      // op_mode:
      // auto, tele, inactivo es 0,1 y 2 respectivamente

      // robot_state:
      //    STATE_UNKNOWN = 0; 
      //  STATE_KILLED = 1; 
      //  STATE_MANUAL = 2; 
      //  STATE_AUTO = 3; 

      int op_mode = msg.op_mode;

      if(op_mode == 0){ // robot is in auto
        currentData.robot_state = 3;
      } else if(op_mode == 1){ // robot is in tele
        currentData.robot_state = 2; // (manual)
      } else if (op_mode == 2){ // robot is inactive
        currentData.robot_state = 1; // (killed)
      } else {
        currentData.robot_state = 0; // unformated message recieved, taken as unknown state
      }

    }

    rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_heading;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_mission_id;
    rclcpp::Subscription<usv_interfaces::msg::SystemStatus>::SharedPtr subscription_system_status;
    

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}