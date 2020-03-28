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

class LidarPCl {
public:
	/**
	 * Constructor. 
	 * @param obstacles_pub_topic[in]: Publisher's topic name. 
	 * @param lidar_sub_topic[in]:Subscriber's topic name. 
	 */
	LidarPCl(
		const std::string &obstacles_pub_topic = "/usv_perception/lidar_detector/obstacles",
		const std::string &lidar_sub_topic = "/velodyne_points");
	
	/** 
	 * Callback to process point cloud. 
	 * @param pcl[in]: Received pointcloud.  
	 */
	void PClCallback(const sensor_msgs::PointCloud2 &pcl);

	/** 
	 * Filters only ROI 
	 */
	void PassThrough();



private:

	// ROS Node 
	ros::NodeHandle lidar_pcl_node_;

	// Publishers for obsacle list.
  ros::Publisher obstacles_pub_;
	// Publishers to Lidar PC.
  ros::Subscriber lidar_sub_;

	usv_perception::obstacles_list list_;
	geometry_msgs::Vector3 obstacle_;

};