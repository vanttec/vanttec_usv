// yolo implementation based on: github.com/triple-Mu/YOLOv8-TensorRT

#define GPU

// std lib
#include "chrono"

// ros
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/image.hpp"

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

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
	rclcpp::TimerBase::SharedPtr timer;

	sl::Mat left_sl, point_cloud;
	cv::Mat left_cv;

	ZED_usv zed_interface;

visualization_msgs::msg::MarkerArray objs2markers(sl::Objects objs) {

	visualization_msgs::msg::MarkerArray ma;

	for (auto& obj : objs.object_list) {
		visualization_msgs::msg::Marker m;
		std_msgs::msg::Header h;

		h.frame_id = "map";
		h.stamp = this->now();

		m.header = h;
		m.type = 2; // sphere
		m.id = obj.id;

		geometry_msgs::msg::Vector3 s;
		s.x = 10.0;
		s.y = 10.0;
		s.z = 10.0;

		m.scale = s;

		std_msgs::msg::ColorRGBA c;

		switch (obj.raw_label) {
			case 0:	
				c.r = 0.0;
				c.g = 0.0;
				c.b = 0.0;
				c.a = 1.0;
				break;
			case 1:	
				c.r = 0.0;
				c.g = 0.0;
				c.b = 1.0;
				c.a = 1.0;
				break;
			case 3:
				c.r = 0.0;
				c.g = 1.0;
				c.b = 0.0;
				c.a = 1.0;
				break;
			case 5:
				c.r = 1.0;
				c.g = 0.0;
				c.b = 0.0;
				c.a = 1.0;
				break;
			case 7:
				c.r = 1.0;
				c.g = 1.0;
				c.b = 0.0;
				c.a = 1.0;
				break;
			default:
				c.r = 1.0;
				c.g = 1.0;
				c.b = 1.0;
				c.a = 1.0;
				break;
		}

		m.color = c;
		
		geometry_msgs::msg::Point pp;
		pp.x = obj.position[0]/10.0;
		pp.y = obj.position[1]/10.0;
		pp.z = obj.position[2]/10.0;

		geometry_msgs::msg::Quaternion q;
		q.x = 0.0;
		q.y = 0.0;
		q.z = 0.0;
		q.w = 1.0;

		geometry_msgs::msg::Pose p;
		p.position = pp;
		p.orientation = q;
		
		m.pose = p;

		ma.markers.push_back(m);
	}
	
	return ma;
}

void frame()
{
	if (zed_interface.cam.grab() == sl::ERROR_CODE::SUCCESS) {

		// delete past detections in rviz
		visualization_msgs::msg::MarkerArray mda;
		visualization_msgs::msg::Marker md;
		md.header.frame_id = "map";    
		md.action = visualization_msgs::msg::Marker::DELETEALL;
		mda.markers.push_back(md);
		this->marker_pub->publish(mda);

		zed_interface.cam.retrieveImage(left_sl, sl::VIEW::LEFT);

		cv::Mat img = slMat2cvMat(left_sl);
		cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

		detector_engine->copy_from_Mat(img, size);
		detector_engine->infer();
		detector_engine->postprocess(in_objs);

		zed_interface.cam.ingestCustomBoxObjects(in_objs);
		
		zed_interface.cam.retrieveObjects(out_objs, object_tracker_parameters_rt);
	
		visualization_msgs::msg::MarkerArray detections = objs2markers(out_objs);

		this->marker_pub->publish(detections);

		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

		this->img_pub->publish(*msg.get());

		RCLCPP_INFO(this->get_logger(), "size! %d", detections.markers.size());
	}

	// auto start = std::chrono::system_clock::now();
	// auto end = std::chrono::system_clock::now();
	// auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
	// RCLCPP_INFO(this->get_logger(), "inference done! %2.4lf ms", tc);
}

public:
DetectorInterface() : Node("bebblebrox_vision"), zed_interface(this->get_logger()) {
	get_parameter("engine_path", engine_path);
        get_parameter("classes_path", classes_path);
        get_parameter("output_topic", output_topic);

	// engine_path = "/home/vanttec/vanttec_usv/RB2024.engine";

	engine_path = ament_index_cpp::get_package_share_directory("usv_perception") + "/data/";
	engine_path += "vtec.engine";


	size = cv::Size{640, 640};

	detector_engine = new YOLOv8(engine_path);
	
	detector_engine->make_pipe(true);

	this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);
	
	this->img_pub = this->create_publisher<sensor_msgs::msg::Image>("/zed_rgba", 10);

	timer = this->create_wall_timer(
			std::chrono::milliseconds(40),
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
