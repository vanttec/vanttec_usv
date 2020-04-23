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
	ros::init(argc, argv, "lidar_detector");
	Lidar lidar_pcl;
	ros::Rate loop_rate(100);
  while (ros::ok())
  { 
    lidar_pcl.DetectObstacles();
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}