#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <netinet/in.h>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>

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
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "report.pb.h"

using namespace std::chrono;

class Heartbeat : public rclcpp::Node

{
  public:
    Heartbeat()
    : Node("robocommand_heartbeat")
    {
      connectionActive = false;

       subscription_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/imu/nav_sat_fix", 
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

      // creating socket
      clientSocket = socket(AF_INET, SOCK_STREAM, 0);

      // specifying address
      sockaddr_in serverAddress;
      serverAddress.sin_family = AF_INET;
      serverAddress.sin_port = htons(50000);
      serverAddress.sin_addr.s_addr = INADDR_ANY;

      // sending connection request
      if(connect(clientSocket, (struct sockaddr*)&serverAddress,
              sizeof(serverAddress)) < 0){

                RCLCPP_WARN(this->get_logger(), "RoboCommand Connection Failed!");
      } else{
        connectionActive = true;
      }

      timer_ = this->create_wall_timer(900ms, 
        std::bind(&Heartbeat::timer_callback,this));

      
    }

  private:

    struct heartbeatInfo{
      // Non-zero placeholder values for proper testing
      double latitude = 10; // 0;
      double longitude = 7; // 0;
      float spd_mps = 5;    // 0;
      float heading_deg = 2; //0;
      int robot_state = 1;   //0;
      int current_task = 1; //0;
    };

    heartbeatInfo currentData;

    void sbgGps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Accessing latitude, longitude
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

    void timer_callback(){

        
      if(!connectionActive){
        // creating socket
        clientSocket = socket(AF_INET, SOCK_STREAM, 0);

        // specifying address
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(50000);
        serverAddress.sin_addr.s_addr = INADDR_ANY;

        // sending connection request
        if(connect(clientSocket, (struct sockaddr*)&serverAddress,
                sizeof(serverAddress)) < 0){

                  RCLCPP_WARN(this->get_logger(), "RoboCommand Connection Failed!");
                  return;
        } else{
          connectionActive = true;
        }
      }

      try{
        RCLCPP_INFO(this->get_logger(), "Timer triggered - Sending heartbeat...");
    

        // building report
        robocommand::roboboat::v1::Report report;

        report.set_team_id("TDMY");                           // Team id fill (Given by team handbook)
        report.set_vehicle_id("S1");                          // Vehicle id fill
        report.set_seq(sequence_counter_++);                            //

        // time stamp
        time_point now = system_clock::now();
        auto time_since_epoch = now.time_since_epoch();
        auto duration_s = duration_cast<seconds>(time_since_epoch);
        auto duration_ns = duration_cast<nanoseconds>(time_since_epoch-duration_s);
        google::protobuf::Timestamp* ts = report.mutable_sent_at();
        ts->set_seconds(duration_s.count());
        ts->set_nanos(duration_ns.count());

        // building heartbeat
        auto* heartbeat = report.mutable_heartbeat();
        heartbeat->set_spd_mps(currentData.spd_mps);
        heartbeat->set_heading_deg(currentData.heading_deg);
        robocommand::roboboat::v1::LatLng* latlng = heartbeat->mutable_position();
        latlng->set_latitude(currentData.latitude);
        latlng->set_longitude(currentData.longitude);

        robocommand::roboboat::v1::RobotState state = robocommand::roboboat::v1::STATE_UNKNOWN;
        switch (currentData.robot_state) {
          case 0:
            state = robocommand::roboboat::v1::STATE_UNKNOWN;
            break;
          case 1:
            state = robocommand::roboboat::v1::STATE_KILLED;
            break;
          case 2:
            state = robocommand::roboboat::v1::STATE_MANUAL;
            break;
          case 3:
            state = robocommand::roboboat::v1::STATE_AUTO;
            break;
        }
        heartbeat->set_state(state);

        robocommand::roboboat::v1::TaskType task = robocommand::roboboat::v1::TASK_UNKNOWN;
        switch (currentData.current_task){
          case 0:
            task = robocommand::roboboat::v1::TASK_UNKNOWN;
            break;
          case 1:
            task = robocommand::roboboat::v1::TASK_NONE;
            break;
          case 2:
            task = robocommand::roboboat::v1::TASK_ENTRY_EXIT;
            break;
          case 3:
            task = robocommand::roboboat::v1::TASK_NAV_CHANNEL;
            break;
          case 4:
            task = robocommand::roboboat::v1::TASK_SPEED_CHALLENGE;
            break;
          case 5:
            task = robocommand::roboboat::v1::TASK_OBJECT_DELIVERY;
            break;
          case 6:
            task = robocommand::roboboat::v1::TASK_DOCKING;
            break;
          case 7:
            task = robocommand::roboboat::v1::TASK_SOUND_SIGNAL;
            break;
        }
        heartbeat->set_current_task(task);

        // formatting message
        std::string msg = report.SerializeAsString();
        uint8_t msg_len = uint8_t(report.ByteSizeLong());

        // sending 2-byte header
        auto s1 = send(clientSocket, "$R", 2, MSG_NOSIGNAL);

        // sending 1-byte length
        auto s2 = send(clientSocket, &msg_len, sizeof(msg_len), MSG_NOSIGNAL);

        // sending serialized report message
        auto s3 = send(clientSocket, msg.c_str(), msg_len, MSG_NOSIGNAL);

        // sending 2-byte footer
        auto s4 = send(clientSocket, "!!", 2, MSG_NOSIGNAL);

        if (s1 < 0 || s2 < 0 || s3 < 0 || s4 < 0) {
          int err = errno;
          RCLCPP_ERROR(this->get_logger(), "Socket send failed: %s (errno: %d)", strerror(err), err);
          
          connectionActive = false;
          // Handle reconnection logic here if needed
          // close(clientSocket);
          // clientSocket = -1;
        }

      }catch (const std::exception& e){

        RCLCPP_ERROR(this->get_logger(), "Unexpected exception in heartbeat: %s", e.what());
        connectionActive = false;
      }

      

    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_heading;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_mission_id;
    rclcpp::Subscription<usv_interfaces::msg::SystemStatus>::SharedPtr subscription_system_status;
    rclcpp::TimerBase::SharedPtr timer_;

    bool connectionActive;
    
    int sequence_counter_ = 0;
    int clientSocket;

};





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}