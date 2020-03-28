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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
//#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
// CLASS ----------------------------------------------------------------------
LidarPCl::LidarPCl(
	const std::string &obstacles_pub_topic, 
	const std::string &lidar_sub_topic)
{
	obstacles_pub_ = lidar_pcl_node_.advertise<usv_perception::obstacles_list>(
		obstacles_pub_topic, 10);
  lidar_sub_ = lidar_pcl_node_.subscribe(lidar_sub_topic, 10, 
    &LidarPCl::PClCallback, this);

	ROS_INFO("Lidar is ready");
}

// FUNCTIONS -------------------------------------------------------------------

//PC2 subscriber callback
void LidarPCl::PClCallback(const sensor_msgs::PointCloud2 &input) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (input, cloud);
  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //pcl::PCLPointCloud2 cloud;
  //pcl_conversions::toPCL(*input, cloud);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::fromPCLPointCloud2(cloud,*temp_cloud);
    //do stuff with temp_cloud here
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempVelodyneRaw( 
    new pcl::PointCloud<pcl::PointXYZ> );
  pcl_conversions::toPCL( *lastMsg, pclVelodyneRaw );
  pcl::fromPCLPointCloud2( pclVelodyneRaw, *tempVelodyneRaw );
  pcl::PointCloud<pcl::PointXYZ> cloud ;*/
}

void LidarPCl::PassThrough(){
  //pcl::PassThrough<pcl::PointXYZ> pass;
}

// FORWARD DECLARATIONS --------------------------------------------------------

// TYPEDEFS AND DEFINES --------------------------------------------------------

// ENUMS -----------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------

// NON-CLASS FUNCTIONS ---------------------------------------------------------

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------

