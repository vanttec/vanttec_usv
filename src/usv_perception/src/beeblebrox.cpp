
#define GPU

// ros
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
;
// msgs
#include "sensor_msgs/msg/image.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/zbbox.hpp"
#include "usv_interfaces/msg/zbbox_array.hpp"

// zed sdk
#include "sl/Camera.hpp"

// opencv
#include "opencv2/opencv.hpp"

// local headers
#include "zed.hpp"

using std::placeholders::_1;

inline cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4;
            break;
        default: break;
    }

    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

/***
 * Class for interfacing with the multiple detectors and syncronizing their results
***/
class DetectorInterface: public rclcpp::Node {

private:
	cv::Mat			res, image;
	cv::Size		size        = cv::Size{640, 640};
	int			num_labels  = 80;
	int			topk        = 100;
	float			score_thres = 0.25f;
	float			iou_thres   = 0.65f;

	sl::Objects				out_objs;
	sl::ObjectDetectionRuntimeParameters	object_tracker_parameters_rt;

	std::string		engine_path;
	std::string		classes_path;
	std::string		output_topic;

	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr objects_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;

	message_filters::Subscriber<usv_interfaces::msg::ZbboxArray> yolo_sub;
	message_filters::Subscriber<usv_interfaces::msg::ZbboxArray> shapes_sub;

	rclcpp::TimerBase::SharedPtr timer;

	sl::Mat left_sl, point_cloud;
	cv::Mat left_cv;

	ZED_usv zed_interface;

	std::shared_ptr<rclcpp::Subscription<usv_interfaces::msg::ZbboxArray>> sub_;

/***
 * Send frame to ros
***/
void frame_send()
{
	if (zed_interface.cam.grab() == sl::ERROR_CODE::SUCCESS) {
		
		// retrieve image from zed
		zed_interface.cam.retrieveImage(left_sl, sl::VIEW::LEFT);

		// convert to usable format
		cv::Mat img = slMat2cvMat(left_sl);
		cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

		// publish
		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
		this->img_pub->publish(*msg.get());
	}

	RCLCPP_INFO(this->get_logger(), "--> image sent");
}

void frame_test(const usv_interfaces::msg::ZbboxArray::SharedPtr dets) {

		std::vector<sl::CustomBoxObjectData> total_dets_sl;
	
		for (auto& det : dets->boxes) {
			sl::CustomBoxObjectData sl_det;
			sl_det.label = det.label;
			sl_det.probability = det.prob;
			sl_det.unique_object_id = sl::String(det.uuid.data());

			std::vector<sl::uint2> bbox(4);
			bbox[0].x = det.x0;
			bbox[0].y = det.y0;
			bbox[1].x = det.x1;
			bbox[1].y = det.y1;

			sl_det.bounding_box_2d = bbox;
			sl_det.is_grounded = false;

			total_dets_sl.push_back(sl_det);
		}

		RCLCPP_INFO(this->get_logger(), "real size: [%d]", total_dets_sl.size());

		sl::Objects t_objs;

		auto start = std::chrono::system_clock::now();

		// send to zed sdk
		zed_interface.cam.ingestCustomBoxObjects(total_dets_sl);
		zed_interface.cam.retrieveObjects(t_objs, object_tracker_parameters_rt);

		auto end = std::chrono::system_clock::now();
		auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
		
		// publish detections in pointcloud
		usv_interfaces::msg::ObjectList detections = objs2markers(t_objs);
		this->objects_pub->publish(detections);

		RCLCPP_INFO(this->get_logger(), "pointcloud estimation done (test): %2.4lf ms [%d]", tc, detections.obj_list.size());
}

/***
 * Callback for detections
 * @param yolo_dets: yolo detections
 * @param shape_dets: shape detections
***/
void receive(
		const usv_interfaces::msg::ZbboxArray::SharedPtr yolo_dets,
		const usv_interfaces::msg::ZbboxArray::SharedPtr shape_dets
	) {
	
		std::vector<sl::CustomBoxObjectData> total_dets_sl;
		
		// convert from ros message to global format
		to_sl(total_dets_sl, yolo_dets);
		to_sl(total_dets_sl, shape_dets);

		auto start = std::chrono::system_clock::now();

		// send to zed sdk
		zed_interface.cam.ingestCustomBoxObjects(total_dets_sl);
		zed_interface.cam.retrieveObjects(out_objs, object_tracker_parameters_rt);

		auto end = std::chrono::system_clock::now();
		auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
		
		// publish detections in pointcloud
		usv_interfaces::msg::ObjectList detections = objs2markers(out_objs);
		this->objects_pub->publish(detections);

		RCLCPP_INFO(this->get_logger(), "pointcloud estimation done: %2.4lf ms [%d]", tc, detections.obj_list.size());
}

/***
 * Convert from ros message to global format
 * @param sl_dets: global format
 * @param dets: ros message
***/
void to_sl(std::vector<sl::CustomBoxObjectData>& sl_dets, const usv_interfaces::msg::ZbboxArray::SharedPtr dets) {
	for (auto& det : dets->boxes) {
		sl::CustomBoxObjectData sl_det;
		sl_det.label = det.label;
		sl_det.probability = det.prob;
		sl_det.unique_object_id = sl::String(det.uuid.data());

		std::vector<sl::uint2> bbox(4);
		bbox[0].x = det.x0;
		bbox[0].y = det.y0;
		bbox[1].x = det.x1;
		bbox[1].y = det.y1;

		sl_det.bounding_box_2d = bbox;
		sl_det.is_grounded = false;

		sl_dets.push_back(sl_det);
	}
}

/***
 * Convert from global format to message understandable by usv_control
 * @param objs: global format
 * @return ros message
***/
usv_interfaces::msg::ObjectList objs2markers(sl::Objects objs) {

	usv_interfaces::msg::ObjectList ma;

	for (int i = 0; i < 5; i++) {
		try {
			auto& obj = objs.object_list.at(i);

			usv_interfaces::msg::Object o;

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

public:
DetectorInterface()
	:	Node("bebblebrox_vision"), zed_interface(this->get_logger()) {

	this->declare_parameter("objects_topic", "/beeblebrox/objects");
	std::string objects_topic = this->get_parameter("objects_topic").as_string();

	this->declare_parameter("video_topic", "/beeblebrox/video");
	std::string video_topic = this->get_parameter("video_topic").as_string();

	
	this->declare_parameter("yolo_sub_topic", "/yolo/detections");
	std::string yolo_sub_topic = this->get_parameter("yolo_sub_topic").as_string();

	this->declare_parameter("shapes_sub_topic", "/shapes/detections");
	std::string shapes_sub_topic = this->get_parameter("shapes_sub_topic").as_string();

	this->declare_parameter("frame_interval", 100);
	int frame_interval = this->get_parameter("frame_interval").as_int();

	this->objects_pub = this->create_publisher<usv_interfaces::msg::ObjectList>(objects_topic, 10);
	this->img_pub = this->create_publisher<sensor_msgs::msg::Image>(video_topic, 10);

	this->yolo_sub.subscribe(this, yolo_sub_topic);
	this->shapes_sub.subscribe(this, shapes_sub_topic);

	typedef message_filters::sync_policies::ApproximateTime
		<usv_interfaces::msg::ZbboxArray, usv_interfaces::msg::ZbboxArray> approximate_policy;

	message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(10), yolo_sub, shapes_sub);

	sub_ = this->create_subscription<usv_interfaces::msg::ZbboxArray>(yolo_sub_topic, 10, 
			std::bind(&DetectorInterface::frame_test, this, _1)
		);

	syncApproximate.registerCallback(&DetectorInterface::receive, this);

	timer = this->create_wall_timer(
			std::chrono::milliseconds(frame_interval),
			std::bind(&DetectorInterface::frame_send, this)
		);
	}
};

int main(int argc, char** argv) {

	cudaSetDevice(0);

	rclcpp::init(argc, argv);

	auto node = std::make_shared< DetectorInterface >();

	rclcpp::spin(node);

	rclcpp::shutdown();
	
	return 0;
}
