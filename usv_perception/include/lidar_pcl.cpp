// ***********************************************************************
/*!
 *  \file    lidar_pointcloud.cpp
 *  \brief   Implementation of hooks to Velodyne data for mover detection, 
 *           and estimation and removal of ground elevation. Simple cannonical
 *           code for exploratory purposes...
 *  \date    March 27, 2020
 *  \author  Ivana Collado González
 *
 *  This node reads Velodyne data; converts it to a local frame; 
 *
 *  Ver. 1.0
 */
// ***********************************************************************

// INCLUDES --------------------------------------------------------------------
#include "lidar_pcl.h"


// CLASS ----------------------------------------------------------------------
Lidar::Lidar(
	const std::string &lidar_sub,
  const std::string &obstacles_pub,
  const std::string &pcl_pub, 
  const int &queue_size)
{
  lidar_sub_ = lidar_node_.subscribe(lidar_sub, queue_size, 
    &Lidar::LidarCallback, this);
	obstacles_pub_ = lidar_node_.advertise<usv_perception::obstacles_list>(
	  obstacles_pub, queue_size);
  pcl_pub_ = lidar_node_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
	  pcl_pub, queue_size);
  ROS_INFO("Lidar is ready");
}

// FUNCTIONS -------------------------------------------------------------------

void Lidar::LidarCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input) {
  *cloud = *input;
  //cloud.header = input.header
}

void Lidar::DetectObstacles(){
  PassThrough();
}

void Lidar::PassThrough(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*cloud_filtered);
  *cloud_filtered.header.frame_id = "/velodyne";
  //cloud_filtered.header = cloud.header
  pcl_pub_.publish(cloud_filtered);
}

// FORWARD DECLARATIONS --------------------------------------------------------

// TYPEDEFS AND DEFINES --------------------------------------------------------

// ENUMS -----------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------

// NON-CLASS FUNCTIONS ---------------------------------------------------------

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------

