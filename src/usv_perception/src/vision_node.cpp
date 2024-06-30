#define GPU

// ros
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"

// msgs
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/object_list.hpp"
#include "usv_interfaces/msg/zbbox.hpp"
#include "usv_interfaces/msg/zbbox_array.hpp"

// zed sdk
#include "sl/Camera.hpp"

// opencv
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// local headers
#include "zed.hpp"

using std::placeholders::_1;

// Class for interfacing with the multiple detectors and syncronizing their results
class DetectorInterface: public rclcpp::Node {

private:
	cv::Mat	res, image, img;
	cv::Size size = cv::Size{640, 640};
	int num_labels = 80;
	int topk = 100;
	float score_thres = 0.25f;
	float iou_thres   = 0.65f;

	int deviceID = 0;
    int apiID = cv::CAP_ANY;

	cv::VideoCapture cap;

	sl::ObjectDetectionRuntimeParameters object_tracker_parameters_rt;

	std::string		engine_path;
	std::string		classes_path;
	std::string		output_topic;

	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr yolo_pub;
	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr shapes_pub;
	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr shapes_pub;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;

	rclcpp::Subscription<usv_interfaces::msg::ZbboxArray>::SharedPtr yolo_sub;
	rclcpp::Subscription<usv_interfaces::msg::ZbboxArray>::SharedPtr shapes_sub;

	rclcpp::TimerBase::SharedPtr timer;

	cv::Mat left_cv;

// Send frame to ros
void frame_send() {
    cap.read(img);
    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Blank frame grabbed!!!");
    }else{
		// convert to ros2 message
		std_msgs::msg::Header hdr;
		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
		msg->header.stamp = this->get_clock()->now();

		// TODO: TERMINR DE LIMPIAR, ME QUEDE AQUI JAAAA

		// publish
		this->ip.publish(msg);
		
		sensor_msgs::msg::CameraInfo leftCamInfoMsg;

		/* publish camera info parameters */

		sl::CalibrationParameters zedParam;
		zedParam = zed_interface.cam.getCameraInformation().camera_configuration.calibration_parameters;
		
		sl::Resolution res;
		res = zed_interface.cam.getCameraInformation().camera_configuration.resolution;

		// only for ZED2, ZED2i, ZEDX and ZEDXm
		leftCamInfoMsg.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

		// yes
		leftCamInfoMsg.d.resize(8);
		leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];    // k1
		leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];    // k2
		leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];    // p1
		leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];    // p2
		leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];    // k3
		leftCamInfoMsg.d[5] = zedParam.left_cam.disto[5];    // k4
		leftCamInfoMsg.d[6] = zedParam.left_cam.disto[6];    // k5
		leftCamInfoMsg.d[7] = zedParam.left_cam.disto[7];    // k6

		// yes
		leftCamInfoMsg.k.fill(0.0);
		leftCamInfoMsg.k[0] = static_cast<double>(zedParam.left_cam.fx);
		leftCamInfoMsg.k[2] = static_cast<double>(zedParam.left_cam.cx);
		leftCamInfoMsg.k[4] = static_cast<double>(zedParam.left_cam.fy);
		leftCamInfoMsg.k[5] = static_cast<double>(zedParam.left_cam.cy);
		leftCamInfoMsg.k[8] = 1.0;

		// yes
		leftCamInfoMsg.r.fill(0.0);
		for (size_t i = 0; i < 3; i++) {
			// identity
			leftCamInfoMsg.r[i + i * 3] = 1;
		}

		// yes
		leftCamInfoMsg.p.fill(0.0);
		leftCamInfoMsg.p[0] = static_cast<double>(zedParam.left_cam.fx);
		leftCamInfoMsg.p[2] = static_cast<double>(zedParam.left_cam.cx);
		leftCamInfoMsg.p[5] = static_cast<double>(zedParam.left_cam.fy);
		leftCamInfoMsg.p[6] = static_cast<double>(zedParam.left_cam.cy);
		leftCamInfoMsg.p[10] = 1.0;

		// yes
		leftCamInfoMsg.width = static_cast<uint32_t>(res.width);
		leftCamInfoMsg.height = static_cast<uint32_t>(res.height);
		leftCamInfoMsg.header.frame_id = "zed2i_left_camera_optical_frame";
		
		this->cam_info_pub->publish(leftCamInfoMsg);
	}
}

/***
 * Callback for detections
 * @param dets: yolo detections
***/
void receive_yolo(const usv_interfaces::msg::ZbboxArray::SharedPtr dets) {

		RCLCPP_DEBUG(this->get_logger(), "[yolo] received detections: %ld", dets->boxes.size());
	
		std::vector<sl::CustomBoxObjectData> dets_sl;
		
		// convert from ros message to global format
		to_sl(dets_sl, dets);

		auto start = std::chrono::system_clock::now();

		// send to zed sdk
		sl::Objects out_objs;
		zed_interface.cam.ingestCustomBoxObjects(dets_sl);
		zed_interface.cam.retrieveObjects(out_objs, object_tracker_parameters_rt);

		auto end = std::chrono::system_clock::now();
		auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
		
		// publish detections in pointcloud
		usv_interfaces::msg::ObjectList detections = objs2markers(out_objs);
		this->yolo_pub->publish(detections);

		RCLCPP_DEBUG(this->get_logger(), "[yolo] pointcloud estimation done: %2.4lf ms [%ld]", tc, detections.obj_list.size());
}

/***
 * Callback for detections
 * @param dets: yolo detections
***/
void receive_shapes(const usv_interfaces::msg::ZbboxArray::SharedPtr dets) {

		RCLCPP_DEBUG(this->get_logger(), "[shapes] received detections: %ld", dets->boxes.size());
	
		std::vector<sl::CustomBoxObjectData> dets_sl;
		
		// convert from ros message to global format
		to_sl(dets_sl, dets);

		auto start = std::chrono::system_clock::now();

		// send to zed sdk
		sl::Objects out_objs;
		zed_interface.cam.ingestCustomBoxObjects(dets_sl);
		zed_interface.cam.retrieveObjects(out_objs, object_tracker_parameters_rt);

		auto end = std::chrono::system_clock::now();
		auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
		
		// publish detections in pointcloud
		usv_interfaces::msg::ObjectList detections = objs2markers_shapes(out_objs);
		this->shapes_pub->publish(detections);

		RCLCPP_DEBUG(this->get_logger(), "[shapes] pointcloud estimation done: %2.4lf ms [%ld]", tc, detections.obj_list.size());
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
		bbox[0] = sl::uint2(det.x0, det.y0);
		bbox[1] = sl::uint2(det.x1, det.y0);
		bbox[2] = sl::uint2(det.x1, det.y1);
		bbox[3] = sl::uint2(det.x0, det.y1);

		sl_det.bounding_box_2d = bbox;
		sl_det.is_grounded = false; // si es `true`, la zed no rastreara este objeto

		sl_dets.push_back(sl_det);
	}
}

/***
 * Convert from global format to message understandable by usv_control
 * @param objs: global format
 * @return ros message
***/
usv_interfaces::msg::ObjectList objs2markers_shapes(sl::Objects objs) {

	usv_interfaces::msg::ObjectList ma;

	for (int i = 0; i < 10; i++) {
		try {
			auto& obj = objs.object_list.at(i);

			usv_interfaces::msg::Object o;

			int color = 0;

			switch (obj.raw_label) {
				case 0:		// blue circle
					o.color = 2;
					o.type = "circle";
					break;
				case 1:		// blue plus
					o.color = 2;
					o.type = "plus";
					break;
				case 2:		// blue square
					o.color = 2;
					o.type = "square";
					break;
				case 3:		// blue triangle
					o.color = 2;
					o.type = "triangle";
					break;
				case 4:		// duck
					o.color = 3;
					o.type = "duck";
					break;
				case 5:		// green circle
					o.color = 1;
					o.type = "circle";
					break;
				case 6:		// green plus
					o.color = 1;
					o.type = "plus";
					break;
				case 7:		// green square
					o.color = 1;
					o.type = "square";
					break;
				case 8:		// green triangle
					o.color = 1;
					o.type = "triangle";
					break;
				case 9:		// red circle
					o.color = 0;
					o.type = "circle";
					break;
				case 10:	// red plus
					o.color = 0;
					o.type = "plus";
					break;
				case 11:	// red square
					o.color = 0;
					o.type = "square";
					break;
				case 12:	// red triangle
					o.color = 0;
					o.type = "triangle";
					break;
				default: // TODO
					color = -1;
					o.type = "ignore";
					break;
			}

			o.x = obj.position[0];
			if ( std::isnan(o.x) ) {
				o.x = 0.0;
			}

			o.y = obj.position[1];
			if ( std::isnan(o.y) ) {
				o.y = 0.0;
			}

			ma.obj_list.push_back(o);
		}
		catch (const std::out_of_range& oor) {
			usv_interfaces::msg::Object o;

			o.color = -1;
			o.x = 0;
			o.y = 0;
			o.type = "ignore";
			ma.obj_list.push_back(o);
		}

	}

	return ma;
}

/***
 * Convert from global format to message understandable by usv_control
 * @param objs: global format
 * @return ros message
***/
usv_interfaces::msg::ObjectList objs2markers(sl::Objects objs) {

	usv_interfaces::msg::ObjectList ma;

	for (int i = 0; i < 10; i++) {
		try {
			auto& obj = objs.object_list.at(i);

			usv_interfaces::msg::Object o;

			int color = 0;

			switch (obj.raw_label) {
				case 0:	// black
					o.color = 4;
					o.type = "round";
					break;
				case 1:	// blue
					o.color = 2;
					o.type = "round";
					break;
				case 2: // course marker
					o.color = 4;
					o.type = "marker";
					break;
				case 3: // green
					o.color = 1;
					o.type = "round";
					break;
				case 4: // red marker
					o.color = 0;
					o.type = "marker";
					break;
				case 5: // red
					o.color = 0;
					o.type = "round";
					break;
				case 6: // green marker
					o.color = 1;
					o.type = "marker";
					break;
				case 7: // yellow
					o.color = 3;
					o.type = "round";
					break;
				default: // TODO
					color = -1;
					o.type = "ignore";
					break;
			}

			o.x = obj.position[0];
			if ( std::isnan(o.x) ) {
				o.x = 0.0;
			}

			o.y = obj.position[1];
			if ( std::isnan(o.y) ) {
				o.y = 0.0;
			}

			ma.obj_list.push_back(o);
		}
		catch (const std::out_of_range& oor) {
			usv_interfaces::msg::Object o;

			o.color = -1;
			o.x = 0;
			o.y = 0;
			o.type = "ignore";
			ma.obj_list.push_back(o);
		}

	}

	return ma;
}

public:
DetectorInterface() : Node("bebblebrox_vision"), zed_interface(this->get_logger()) {
	/* PARAMETERS */
	
	// final detections' topic
	this->declare_parameter("objects_yolo_topic", "/beeblebrox/objects/yolo");
	std::string objects_yolo_topic = this->get_parameter("objects_yolo_topic").as_string();

	this->declare_parameter("objects_shapes_topic", "/beeblebrox/objects/shapes");
	std::string objects_shapes_topic = this->get_parameter("objects_shapes_topic").as_string();
	
	// publisher's video topic
	this->declare_parameter("video_topic", "/beeblebrox/video");
	std::string video_topic = this->get_parameter("video_topic").as_string();
	
	// subcriptions to detections' topic
	this->declare_parameter("yolo_sub_topic", "/yolo/detections");
	std::string yolo_sub_topic = this->get_parameter("yolo_sub_topic").as_string();

	this->declare_parameter("shapes_sub_topic", "/shapes/detections");
	std::string shapes_sub_topic = this->get_parameter("shapes_sub_topic").as_string();

	// frame interval (ms)
	this->declare_parameter("frame_interval", 100);
	int frame_interval = this->get_parameter("frame_interval").as_int();

	/* PUBLISHERS */
	
	// yolo's final detections publisher
	this->yolo_pub = this->create_publisher<usv_interfaces::msg::ObjectList>( objects_yolo_topic, 10 );

	// shapes' final detections publisher
	this->shapes_pub = this->create_publisher<usv_interfaces::msg::ObjectList>( objects_shapes_topic, 10 );

	// video publisher
	rclcpp::Node::SharedPtr nh(std::shared_ptr<DetectorInterface>(this, [](auto *) {}));
	image_transport::ImageTransport it(nh);
	this->ip = it.advertise(video_topic, 5);

	this->cam_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/beeblebrox/camera_info", 10);

	/* SUBSCRIBERS */
	
	rclcpp::CallbackGroup::SharedPtr r_group;
	r_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	rclcpp::SubscriptionOptions options;
	options.callback_group = r_group;

	this->yolo_sub = this->create_subscription<usv_interfaces::msg::ZbboxArray>(
			yolo_sub_topic,
			rclcpp::SystemDefaultsQoS(),
			std::bind(&DetectorInterface::receive_yolo, this, _1),
			options
		);
	
	this->shapes_sub = this->create_subscription<usv_interfaces::msg::ZbboxArray>(
			shapes_sub_topic,
			rclcpp::SystemDefaultsQoS(),
			std::bind(&DetectorInterface::receive_shapes, this, _1),
			options
		);

	timer = this->create_wall_timer(
			std::chrono::milliseconds(frame_interval),
			std::bind(&DetectorInterface::frame_send, this)
		);
	
	cap.open(apiID, deviceID);
	}
};

int main(int argc, char** argv) {
	cudaSetDevice(0);
	rclcpp::init(argc, argv);

	auto node = std::make_shared< DetectorInterface >();
	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();

	rclcpp::shutdown();
	return 0;
}