#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "usv_interfaces/msg/usv_loc.hpp"

class Localization : public rclcpp::Node {
public:
  Localization() : Node("localization_node") {
    subscription_quat = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
      "/sbg/ekf_quat", 10, std::bind(&Localization::quat_callback, this, std::placeholders::_1));

    subscription_nav = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
      "/sbg/ekf_nav", 10, std::bind(&Localization::nav_callback, this, std::placeholders::_1));

    publisher = this->create_publisher<usv_interfaces::msg::UsvLoc>("/usv_loc", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo de Localization iniciado.");
  }

private:
  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr subscription_quat;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr subscription_nav;
  rclcpp::Publisher<usv_interfaces::msg::UsvLoc>::SharedPtr publisher;

  bool origin_set = false;
  bool angles_valid = false;
  bool pose_valid = false;
  double lat0 = 0.0;
  double lon0 = 0.0;
  double cos_lat0 = 0.0;
  double roll = 0.0; 
  double pitch = 0.0; 
  double yaw = 0.0;

  struct NedPosition{
    double x;
    double y;
  };

  NedPosition latLonToNED(double lat, double lon){
    constexpr double R_TIERRA = 6371000.0; // radio medio de la Tierra en metros

    double delta_lat_rad = (lat - lat0) * M_PI / 180.0;
    double delta_lon_rad = (lon - lon0) * M_PI / 180.0;
    NedPosition pos;
    pos.x = delta_lat_rad * R_TIERRA;
    pos.y = delta_lon_rad * cos_lat0 * R_TIERRA;

    return pos;
  }

  void quat_callback(const sbg_driver::msg::SbgEkfQuat::SharedPtr msg) {

    angles_valid = (msg->status.solution_mode == 4) && (msg->status.heading_valid) && (msg->status.attitude_valid);

    tf2::Quaternion q(
      msg->quaternion.x,
      msg->quaternion.y,
      msg->quaternion.z,
      msg->quaternion.w
    );

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (!angles_valid) {
     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "DATOS NO CONFIABLES - Modo: %d | Attitude: %s | Heading: %s |Mag: %s",
        msg->status.solution_mode,
        msg->status.attitude_valid ? "VALID" : "INVALID",
        msg->status.heading_valid ? "VALID" : "INVALID",
        msg->status.mag_ref_used ? "USED" : "NOT USED");
    }
  }



  void nav_callback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg){
    if (!origin_set){
        if(msg->status.solution_mode == 4 && msg->status.position_valid){
            lat0 = msg->latitude;
            lon0 = msg->longitude;
            cos_lat0 = std::cos(lat0 * M_PI / 180.0);
            origin_set=true;
        }
        return;
    }

    pose_valid = (msg->status.solution_mode == 4) && (msg->status.position_valid);

    if (!pose_valid) {
     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "DATOS NO CONFIABLES - Mode: %d | Position: %s",
        msg->status.solution_mode,
        msg->status.position_valid ? "VALID" : "INVALID");
    }

    NedPosition pos = latLonToNED(msg->latitude,msg->longitude);

    usv_interfaces::msg::UsvLoc msg_out;
    msg_out.status = msg->status.solution_mode;
    msg_out.latitud = msg->latitude;
    msg_out.longitud = msg->longitude;
    msg_out.position_valid = msg->status.position_valid;
    msg_out.x = pos.x;
    msg_out.y = pos.y;


    msg_out.attitude_valid= msg->status.attitude_valid;
    msg_out.heading_valid = msg->status.heading_valid;
    msg_out.roll = roll;
    msg_out.pitch = pitch;
    msg_out.yaw = yaw;

    publisher->publish(msg_out);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localization>());
  rclcpp::shutdown();
  return 0;
}