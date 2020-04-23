/**
 * @file: lidar_pointcloud.h
 * @author: Ivana Collado
 * @date: March 27, 2020
 *
 * @brief: Defines a class that will recueve and manipulate lidar pointcloud
 */

// INCLUDES --------------------------------------------------------------------
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>

#include <usv_perception/obstacles_list.h>

// PCL
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_ros/transforms.h>
//#include <pcl_ros/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/PCLPointCloud2.h>


class Lidar {
public:
	/**
	 * Constructor. 
	 * @param obstacles_pub[in]: Publisher's topic name. 
	 * @param lidar_sub[in]: Subscriber's topic name. 
	 * @param queue_size[in]: Message queue size.
	 */
	Lidar(
		const std::string &lidar_sub = "/velodyne_pcl",
		const std::string &obstacles_pub = "/usv_perception/lidar_detector/obstacles",
		const std::string &pcl_pub = "/usv_perception/lidar_detector/filtered_pcl",
		const int &queue_size = 10
	);
	
	/** 
	 * Callback to process point cloud. 
	 * @param input[in]: Received pointcloud.  
	 */
	void LidarCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input);

	/** 
	 * Filters only ROI 
	 */
	void DetectObstacles();

private:

	/** 
	 * Filters only ROI 
	 */
	void PassThrough();

	// ROS Node 
	ros::NodeHandle lidar_node_;

	// Publishers to Lidar PC.
  ros::Subscriber lidar_sub_;
	// Publishers for obsacle list.
  ros::Publisher obstacles_pub_;
	ros::Publisher pcl_pub_;

	usv_perception::obstacles_list list_;
	geometry_msgs::Vector3 obstacle_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

};