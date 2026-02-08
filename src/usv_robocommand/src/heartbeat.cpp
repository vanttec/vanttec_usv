#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>

#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"



class Heartbeat : public rclcpp::Node
{
  public:
    Heartbeat()
    : Node("robocommand_heartbeat")
    {
       subscription_ = this->create_subscription<sbg_driver::msg::SbgGpsPos>(
            "/sbg/gps_pos", 
            10, 
            std::bind(&Heartbeat::sbgGps_callback, this, std::placeholders::_1)
        );

      

      
    }

  private:


    void sbgGps_callback(const sbg_driver::msg::SbgGpsPos::SharedPtr msg) const
    {
        // Accessing latitude, longitude, and altitude
        RCLCPP_INFO(this->get_logger(), "Received GPS Pos -> Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                    msg->latitude, 
                    msg->longitude, 
                    msg->altitude);
        
    }


    rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}