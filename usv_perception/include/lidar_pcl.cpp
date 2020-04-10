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
#include <pcl/filters/passthrough.h>

// CLASS ----------------------------------------------------------------------
Lidar::Lidar(
	const std::string &obstacles_pub, 
	const std::string &lidar_sub)
{
	obstacles_pub_ = lidar_node_.advertise<usv_perception::obstacles_list>(
		obstacles_pub, 10);
  lidar_sub_ = lidar_node_.subscribe(lidar_sub, 10, &Lidar::LidarCallback, this);
	
  ROS_INFO("Lidar is ready");
}

// FUNCTIONS -------------------------------------------------------------------

//PC2 subscriber callback
//void Lidar::LidarCallback(const sensor_msgs::PointCloud2 &input) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::fromROSMsg (input, cloud);
  //ROS_INFO("Recieved pointcloud");
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
//}

void Lidar::LidarCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input) {
  pcl::PointCloud<pcl::PointXYZ> cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_ = input->points;
  //ROS_INFO("Recieved pointcloud");
}


void Lidar::PassThrough(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud(input_);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*cloud_filtered);
}

// FORWARD DECLARATIONS --------------------------------------------------------

// TYPEDEFS AND DEFINES --------------------------------------------------------

// ENUMS -----------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------

// NON-CLASS FUNCTIONS ---------------------------------------------------------

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------

