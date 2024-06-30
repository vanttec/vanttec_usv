#ifndef ZED_USV
#define ZED_USV
#include "sl/Camera.hpp"
#include "rclcpp/rclcpp.hpp"

class ZED_usv {
public:
    sl::Camera cam;

    ZED_usv(rclcpp::Logger);
    ~ZED_usv();

private:
    sl::InitParameters init_params;
    sl::ObjectDetectionParameters detection_params;

    rclcpp::Logger logger;
};

ZED_usv::ZED_usv(rclcpp::Logger logger_param) : logger(logger)
{   
    this->logger = logger_param;

    init_params.sdk_verbose = true; // TODO false ?
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_maximum_distance = 20;

    detection_params.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;

    auto returned_state = cam.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->logger, "COULD NOT OPEN CAMERA");
        return;
    }

    cam.enablePositionalTracking();

    returned_state = cam.enableObjectDetection(detection_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->logger, "COULD NOT INIT OBJECT DETECTION%d", returned_state);
        cam.close();
        return;
    }

}

ZED_usv::~ZED_usv()
{
    RCLCPP_DEBUG(this->logger, "ZED CLOSED");
    cam.close();
}

#endif
