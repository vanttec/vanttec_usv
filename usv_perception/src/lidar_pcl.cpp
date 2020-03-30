/**
 * @file: lidar_pcl.cpp
 * @author: Ivana Collado
 * @date: March 27, 2020
 *
 * @brief: 
 */
#include "lidar_pcl.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_pcl");

	LidarPCl lidar_pcl;

	ros::spin();

	return 0;
}