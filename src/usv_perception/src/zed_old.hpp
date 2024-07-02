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
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_maximum_distance = 20;

    auto returned_state = cam.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->logger, "no se pudo abrir la camara :(");
        return;
    }

    cam.enablePositionalTracking();
    
    detection_params.enable_tracking = false; // should it be true?
    detection_params.enable_segmentation = false;
    detection_params.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;

    returned_state = cam.enableObjectDetection(detection_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->logger, "no se pudo empezar la deteccion de objetos en el zed-sdk %d", returned_state);
        cam.close();
        return;
    }

}

ZED_usv::~ZED_usv()
{
    RCLCPP_DEBUG(this->logger, "se ha cerrado la camara");
    cam.close();
}

#endif
