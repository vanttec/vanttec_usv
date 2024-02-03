#define GPU

// ros
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"
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

// local headers
#include "zed.hpp"

using std::placeholders::_1;
// using std::placeholders::_2;

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

	sl::ObjectDetectionRuntimeParameters	object_tracker_parameters_rt;

	std::string		engine_path;
	std::string		classes_path;
	std::string		output_topic;

	// final detections publishers
	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr yolo_pub;
	rclcpp::Publisher<usv_interfaces::msg::ObjectList>::SharedPtr shapes_pub;

	// image_transport publisher
	image_transport::Publisher ip;

	// publish camera info from zed
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;

	// receiver of boundingboxes
	rclcpp::Subscription<usv_interfaces::msg::ZbboxArray>::SharedPtr yolo_sub;
	rclcpp::Subscription<usv_interfaces::msg::ZbboxArray>::SharedPtr shapes_sub;

	// timer in which to execute everything
	rclcpp::TimerBase::SharedPtr timer;

	sl::Mat left_sl, point_cloud;
	cv::Mat left_cv;

	// instance of the zed camera
	ZED_usv zed_interface;

	// typedef message_filters::sync_policies::ExactTime
	// 	<usv_interfaces::msg::ZbboxArray, usv_interfaces::msg::ZbboxArray> exact_policy;
	//
	// typedef message_filters::Synchronizer<exact_policy> Synchronizer;
	//
	// std::unique_ptr<Synchronizer> syncExact;

/***
 * Send frame to ros
***/
void frame_send()
{
	if (zed_interface.cam.grab() == sl::ERROR_CODE::SUCCESS) {
		/* publish zed image */

		// retrieve image from zed
		zed_interface.cam.retrieveImage(left_sl, sl::VIEW::LEFT);

		// convert to opencv format
		cv::Mat img = slMat2cvMat(left_sl);
		cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

		// convert to ros2 message
		std_msgs::msg::Header hdr;
		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
		msg->header.stamp = this->get_clock()->now();

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

		leftCamInfoMsg.d.resize(8);
		leftCamInfoMsg.d[0] = zedParam.left_cam.disto[0];    // k1
		leftCamInfoMsg.d[1] = zedParam.left_cam.disto[1];    // k2
		leftCamInfoMsg.d[2] = zedParam.left_cam.disto[2];    // p1
		leftCamInfoMsg.d[3] = zedParam.left_cam.disto[3];    // p2
		leftCamInfoMsg.d[4] = zedParam.left_cam.disto[4];    // k3
		leftCamInfoMsg.d[5] = zedParam.left_cam.disto[5];    // k4
		leftCamInfoMsg.d[6] = zedParam.left_cam.disto[6];    // k5
		leftCamInfoMsg.d[7] = zedParam.left_cam.disto[7];    // k6

		leftCamInfoMsg.k.fill(0.0);
		leftCamInfoMsg.k[0] = static_cast<double>(zedParam.left_cam.fx);
		leftCamInfoMsg.k[2] = static_cast<double>(zedParam.left_cam.cx);
		leftCamInfoMsg.k[4] = static_cast<double>(zedParam.left_cam.fy);
		leftCamInfoMsg.k[5] = static_cast<double>(zedParam.left_cam.cy);
		leftCamInfoMsg.k[8] = 1.0;

		leftCamInfoMsg.r.fill(0.0);
		for (size_t i = 0; i < 3; i++) {
			// identity
			leftCamInfoMsg.r[i + i * 3] = 1;
		}

		leftCamInfoMsg.p.fill(0.0);
		leftCamInfoMsg.p[0] = static_cast<double>(zedParam.left_cam.fx);
		leftCamInfoMsg.p[2] = static_cast<double>(zedParam.left_cam.cx);
		leftCamInfoMsg.p[5] = static_cast<double>(zedParam.left_cam.fy);
		leftCamInfoMsg.p[6] = static_cast<double>(zedParam.left_cam.cy);
		leftCamInfoMsg.p[10] = 1.0;

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

		RCLCPP_INFO(this->get_logger(), "[yolo] pointcloud estimation done: %2.4lf ms [%ld]", tc, detections.obj_list.size());
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
		usv_interfaces::msg::ObjectList detections = objs2markers(out_objs);
		this->shapes_pub->publish(detections);

		RCLCPP_INFO(this->get_logger(), "[shapes] pointcloud estimation done: %2.4lf ms [%ld]", tc, detections.obj_list.size());
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

	// syncExact.reset(new message_filters::Synchronizer<exact_policy>(exact_policy(10), this->yolo_sub, this->shapes_sub));
	// syncExact->registerCallback(&DetectorInterface::receive, this);

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

	rclcpp::executors::MultiThreadedExecutor exec;
	exec.add_node(node);
	exec.spin();

	rclcpp::shutdown();
	
	return 0;
}


