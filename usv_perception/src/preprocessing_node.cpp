/** ----------------------------------------------------------------------------
 * @file:     preprocessing_node.cpp
 * @date:     Octubre 8, 2020
 * @coauthor: Ivana Collado
 * @email:    a00569475@itesm.mx
 * @coauthor: Rodolfo Cuan
 * @email:    a01233155@itesm.mx
 * 
 * @brief: Use case of the VantTec Point Cloud (VTPC) library. 
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include "VTPC/VTPC.h"
#include <pcl_conversions/pcl_conversions.h>

// NAMESPACES ------------------------------------------------------------------

// GLOBAL PARAMETERS -----------------------------------------------------------
/**
  * VTPC library object
  * */
VTPC<pcl::PointXYZI> vtpc_;
/**
 * Callback Global Cloud
 * */
pcl::PointCloud<pcl::PointXYZI> cloud_;
/**
  * Size of buffer queue 
  * */
int queue_size_ = 10;
/**
  * Subscribers
  * */
ros::Subscriber cloud_sub_;
/**
  * Publishers
  * */
ros::Publisher cloud_pub_;
ros::Publisher obj_detected_pub_;



// LAUNCH PARAMETERS ---------------------------------------------------------
/**
  * Topics
  * */
const std::string topic_cloud_sub_ = "/velodyne_points";
const std::string topic_cloud_pub_ = "/output_pointcloud";
const std::string topic_objdet_pub_ = "/obj_detected_lidar";
/** 
  * Parameters
  * */
const std::string param_queue_size_ = "queue_size";

// FUNCTIONS -------------------------------------------------------------------
  /**
  * Initialize processing node
  * @return void.
  * */
void initialize(ros::NodeHandle &node);
  /**
  * Callback to obtain point cloud. 
  * @param msg[in]: Received Point cloud.
  * @return void.
  */
void on_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr &msg);

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "preprocessing");
  ros::NodeHandle preprocessing_node("preprocessing");
  ros::Rate loop_rate(100);
  initialize(preprocessing_node);

  while (ros::ok())
  { 
    
    vtpc_.SetCloud(cloud_);

    //Testing Functions
    vtpc_.PassThrough("z", -0.4, 2.0);
    vtpc_.RadiusFilter(2.5);
    if (vtpc_.GetCloud().size() != 0){
      vtpc_.CreateGrid(0.2);
      vtpc_.ClusterGrid(true);
      vtpc_.ObjectDetectionPublish(obj_detected_pub_);
      //vtpc_.viewPointCloud();
      cloud_pub_.publish(vtpc_.GetCloudMsg());
    }


    //Publish PointCloud
    
    
    //ros::Duration(2.0).sleep();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void initialize(ros::NodeHandle &node){
  // Subscribers
  cloud_sub_ = node.subscribe(topic_cloud_sub_, queue_size_, 
    &on_cloud_msg);
  // Publishers
  cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>(
    topic_cloud_pub_, queue_size_);

  obj_detected_pub_ = node.advertise<usv_perception::obj_detected_list>(
    topic_objdet_pub_, queue_size_);
  //Success
  ROS_INFO("Velocity obstacle node is Ready!");
}

void on_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr &msg){

  ROS_INFO("RECIEVED POINT CLOUD");
  
  pcl::PCLPointCloud2 pcl_pc2;
  //From sensor_msgs::PointCloud2 to pcl::PCLPointCloud2 
  pcl_conversions::toPCL(*msg,pcl_pc2);
  //From pcl::PCLPointCloud2 to pcl::PointCloud<PointT>
  pcl::fromPCLPointCloud2(pcl_pc2,cloud_);  

}

