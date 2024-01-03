// yolo implementation based on: github.com/triple-Mu/YOLOv8-TensorRT
#include "chrono"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

#define GPU

#ifdef GPU
#include "tensorrt.hpp"
#else
#include "onnx.hpp"
#endif

using std::placeholders::_1;

// TODO engine file path as parameter
// TODO input topic as parameter
// TODO output topic as paramter
// TODO launch file
// TODO detect if file does not exist before hand

template <typename E>
class DetectorInterface: public rclcpp::Node {
private:
    cv::Mat  res, image;
    cv::Size size        = cv::Size{640, 640};
    int      num_labels  = 80;
    int      topk        = 100;
    float    score_thres = 0.25f;
    float    iou_thres   = 0.65f;
	std::vector<Object> objs;

	std::string engine_path;
	std::string classes_path;
	std::string input_topic;
	std::string output_topic;

	E*			detector_engine;

	std::shared_ptr<image_transport::ImageTransport> it;
	std::shared_ptr<image_transport::Subscriber> is;

	void frame_detect(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {

		auto img = cv_bridge::toCvShare(msg, "bgr8")->image;

		detector_engine->copy_from_Mat(img, size);
		detector_engine->infer();
		detector_engine->postprocess(objs, score_thres, iou_thres, topk, num_labels);

		RCLCPP_INFO(this->get_logger(), "inference done!");
    }
public:
    DetectorInterface() : Node("yolov8_node") {
        get_parameter("engine_path", engine_path);
        get_parameter("classes_path", classes_path);
        get_parameter("input_topic", input_topic);
        get_parameter("output_topic", output_topic);

		input_topic = "/video";

		size = cv::Size{640, 640};

		#ifdef GPU
		detector_engine = new YOLOv8(engine_path);
		detector_engine->make_pipe(true);
		#else
		this->detector_engine = new ONNX(engine_path);
		#endif
    }

	void init() {
		it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

		is = std::make_shared<image_transport::Subscriber>(
			it->subscribe(
				input_topic,
				10,
				std::bind(&DetectorInterface::frame_detect, this, std::placeholders::_1) )
		);
	}

};

int main(int argc, char** argv)
{
	cudaSetDevice(0);

	rclcpp::init(argc, argv);

	#ifdef GPU
	auto node = std::make_shared< DetectorInterface< YOLOv8 > >();
	#else
	auto node = std::make_shared< DetectorInterface< ONNX > >();
	#endif
	
	node->init();

	rclcpp::spin(node);

	rclcpp::shutdown();
    return 0;
}
