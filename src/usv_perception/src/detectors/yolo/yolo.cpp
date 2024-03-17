#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

#include "usv_interfaces/msg/zbbox_array.hpp"
#include "usv_interfaces/msg/zbbox.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "tensorrt.hpp"

using std::placeholders::_1;

template <typename E>
class YoloDetector: public rclcpp::Node {

private:
	cv::Size		size        = cv::Size{640, 640};

	std::string		engine_path;
	std::string		video_topic;
	std::string		output_topic;
	double			threshold;

	E*			detector_engine;

	std::shared_ptr<image_transport::ImageTransport> it;
	std::shared_ptr<image_transport::Subscriber> is;
	std::shared_ptr<rclcpp::Publisher<usv_interfaces::msg::ZbboxArray>> dets;


	// rclcpp::Publisher<usv_interfaces::msg::ZbboxArray>::SharedPtr dets;



void frame(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
	auto img = cv_bridge::toCvCopy(msg, "bgr8")->image;

	detector_engine->copy_from_Mat(img, size);
	detector_engine->infer();
	
	usv_interfaces::msg::ZbboxArray objs;
	objs = detector_engine->postprocess();

	objs.header.stamp = this->now();

	this->dets->publish( objs );

	RCLCPP_INFO(this->get_logger(), "--> inference done [%ld]", objs.boxes.size());
}

public:
	YoloDetector() : Node("yolo") {

		this->declare_parameter("engine_path", "/home/vanttec/vanttec_usv/SARASOTA.engine");
		engine_path = this->get_parameter("engine_path").as_string();

		this->declare_parameter("video_topic", "/beeblebrox/video");
		video_topic = this->get_parameter("video_topic").as_string();

		this->declare_parameter("output_topic", "/yolo/detections");
		output_topic = this->get_parameter("output_topic").as_string();

		this->declare_parameter("threshold", 0.6);
		threshold = this->get_parameter("threshold").as_double();

		size = cv::Size{640, 640};

		detector_engine = new YOLOv8(engine_path, threshold, this->get_logger());
		
		detector_engine->make_pipe(true);

		this->dets = this->create_publisher<usv_interfaces::msg::ZbboxArray>(output_topic, 10);
	}

	void init() {
		this->it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

		this->is = std::make_shared<image_transport::Subscriber>(
			it->subscribe(
				video_topic,
				10,
				std::bind(&YoloDetector::frame, this, std::placeholders::_1)
			)
		);

		RCLCPP_INFO(this->get_logger(), "-> yolo ready");
	}

};


int main(int argc, char **argv)
{
	cudaSetDevice(0);

	rclcpp::init(argc, argv);

	auto node = std::make_shared< YoloDetector<YOLOv8> >();
	node->init();

	rclcpp::spin(node);

	rclcpp::shutdown();
	
	return 0;
}
