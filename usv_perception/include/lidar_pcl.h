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
#include <pcl/point_types.h>
//#include <pcl_ros/transforms.h>
//#include <pcl_ros/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/PCLPointCloud2.h>

class Lidar {
public:
	/**
	 * Constructor. 
	 * @param obstacles_pub[in]: Publisher's topic name. 
	 * @param lidar_sub[in]:Subscriber's topic name. 
	 */
	Lidar(
		const std::string &obstacles_pub = "/usv_perception/lidar_detector/obstacles",
		const std::string &lidar_sub = "/velodyne_pcl"
	);
	
	/** 
	 * Callback to process point cloud. 
	 * @param input[in]: Received pointcloud.  
	 */
	//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	//void PClCallback(const sensor_msgs::PointCloud2 &pcl);
	void LidarCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input);

	/** 
	 * Filters only ROI 
	 */
	void PassThrough();



private:

	// ROS Node 
	ros::NodeHandle lidar_node_;

	// Publishers for obsacle list.
  ros::Publisher obstacles_pub_;
	// Publishers to Lidar PC.
  ros::Subscriber lidar_sub_;

	usv_perception::obstacles_list list_;
	geometry_msgs::Vector3 obstacle_;

};