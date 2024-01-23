
#define GPU

// std lib
#include "chrono"

// ros
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

//#include "visualization_msgs/msg/marker.hpp"
//#include "visualization_msgs/msg/marker_array.hpp"
//#include "std_msgs/msg/color_rgba.hpp"
//#include "std_msgs/msg/header.hpp"
//#include "geometry_msgs/msg/vector3.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/point.hpp"
//#include "geometry_msgs/msg/quaternion.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/object_list.hpp"

// zed sdk
#include "sl/Camera.hpp"

// opencv
#include "opencv2/opencv.hpp"

#include "tensorrt.hpp"
#include "zed.hpp"

using std::placeholders::_1;

// TODO engine file path as parameter
// TODO output topic as paramter
// TODO launch file
// TODO detect if file does not exist before hand

template <typename E>
class DetectorInterface: public rclcpp::Node {

private:
	cv::Mat			res, image;
	cv::Size		size        = cv::Size{640, 640};
	int			num_labels  = 80;
	int			topk        = 100;
	float			score_thres = 0.25f;
	float			iou_thres   = 0.65f;

	std::vector<sl::CustomBoxObjectData>	in_objs;
	sl::Objects				out_objs;
	sl::ObjectDetectionRuntimeParameters	object_tracker_parameters_rt;

	std::string		engine_path;
	std::string		classes_path;
	std::string		output_topic;

	E*			detector_engine;

	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr objects_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
	rclcpp::TimerBase::SharedPtr timer;

	sl::Mat left_sl, point_cloud;
	cv::Mat left_cv;

	ZED_usv zed_interface;

usv_interfaces::msg::ObjectList objs2markers(sl::Objects objs) {

	usv_interfaces::msg::ObjectList ma;

	for (int i = 0; i < 5; i++) {
		try {
			auto& obj = objs.object_list.at(i);

			usv_interfaces::msg::Object o;

			//o.id = obj.id;
			
			int color = 0;

			switch (obj.raw_label) {
				case 0:	// black
					color = 4;
					break;
				case 1:	// blue
					color = 2;
					break;
				case 3: // green
					color = 1;
					break;
				case 5: // red
					color = 0;
					break;
				case 7: // yellow
					color = 3;
					break;
				default: // TODO
					color = -1;
					break;
			}

			o.color = color;
			
			o.x = obj.position[0] / 1000.0;
			o.y = obj.position[1] / 1000.0;

			o.type = "buoy";

			ma.obj_list.push_back(o);
		}
		catch (const std::out_of_range& oor) {
			usv_interfaces::msg::Object o;

			o.color = 5;
			o.x = 0;
			o.y = 0;
			o.type = "ignore";
			ma.obj_list.push_back(o);
		}

	}

	// for (auto& obj : objs.object_list) {
	// 	usv_interfaces::msg::Object o;
	//
	//
	// 	//o.id = obj.id;
	// 	
	// 	int color = 0;
	//
	// 	switch (obj.raw_label) {
	// 		case 0:	// black
	// 			color = 4;
	// 			break;
	// 		case 1:	// blue
	// 			color = 2;
	// 			break;
	// 		case 3: // green
	// 			color = 1;
	// 			break;
	// 		case 5: // red
	// 			color = 0;
	// 			break;
	// 		case 7: // yellow
	// 			color = 3;
	// 			break;
	// 		default: // TODO
	// 			color = -1;
	// 			break;
	// 	}
	//
	// 	o.color = color;
	// 	
	// 	o.x = obj.position[0];
	// 	o.y = obj.position[1];
	//
	// 	o.type = "buoy";
	//
	// 	ma.obj_list.push_back(o);
	// }
	// 
	return ma;
}

void frame()
{
	if (zed_interface.cam.grab() == sl::ERROR_CODE::SUCCESS) {

		zed_interface.cam.retrieveImage(left_sl, sl::VIEW::LEFT);

		cv::Mat img = slMat2cvMat(left_sl);
		cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

		detector_engine->copy_from_Mat(img, size);

		auto start = std::chrono::system_clock::now();
		
		detector_engine->infer();
		detector_engine->postprocess(in_objs);

		zed_interface.cam.ingestCustomBoxObjects(in_objs);
		
		zed_interface.cam.retrieveObjects(out_objs, object_tracker_parameters_rt);

		auto end = std::chrono::system_clock::now();
		auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
	
		usv_interfaces::msg::ObjectList detections = objs2markers(out_objs);

		this->objects_pub->publish(detections);

		//sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

		//this->img_pub->publish(*msg.get());

		RCLCPP_INFO(this->get_logger(), "inference done: %2.4lf ms [%d]", tc, detections.obj_list.size());
	}
}

public:
DetectorInterface() : Node("bebblebrox_vision"), zed_interface(this->get_logger()) {
	get_parameter("engine_path", engine_path);
        get_parameter("classes_path", classes_path);
        get_parameter("output_topic", output_topic);

	// engine_path = "/home/vanttec/vanttec_usv/RB2024.engine";

	engine_path = ament_index_cpp::get_package_share_directory("usv_perception") + "/data/";
	engine_path += "vtec_v2.engine";


	size = cv::Size{640, 640};

	detector_engine = new YOLOv8(engine_path);
	
	detector_engine->make_pipe(true);

	this->objects_pub = this->create_publisher<usv_interfaces::msg::ObjectList>("/objects", 10);
	
	//this->img_pub = this->create_publisher<sensor_msgs::msg::Image>("/zed_rgba", 10);

	timer = this->create_wall_timer(
			std::chrono::milliseconds(70),
			std::bind(&DetectorInterface::frame, this)
		);
	}
};

int main(int argc, char** argv) {

	cudaSetDevice(0);

	rclcpp::init(argc, argv);

	auto node = std::make_shared< DetectorInterface<YOLOv8> >();

	rclcpp::spin(node);

	rclcpp::shutdown();
	
	return 0;
}
