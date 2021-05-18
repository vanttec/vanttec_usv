/** ----------------------------------------------------------------------------
 * @file:     obj_detector_node.cpp
 * @date:     February 18, 2021
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
/**
 * Services
 */
ros::ServiceClient dock_corners_client_;



// LAUNCH PARAMETERS ---------------------------------------------------------
/**
  * Topics
  * */
std::string topic_cloud_sub_ = "";
std::string topic_cloud_pub_ = "";
std::string topic_objdet_pub_ = "";
/** 
  * Parameters
  * */
const std::string param_queue_size_ = "queue_size";
bool viewPointCloudParam_ = false;

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
  ros::init(argc, argv, "obj_detector_node");
  ros::NodeHandle obj_detector_node("~");
  ros::Rate loop_rate(20);
  initialize(obj_detector_node);

  while (ros::ok())
  { 
    

    vtpc_.SetCloud(cloud_);

    //Testing Functions
    //vtpc_.PassThrough("z", -0.4, 2.0);
    vtpc_.RadiusFilter(200);

    
    if (vtpc_.GetCloud().size() != 0){
      vtpc_.CreateGrid(0.2);
      vtpc_.ClusterGrid(false);
      vtpc_.ObjectDetectionPublish(obj_detected_pub_);
      if(viewPointCloudParam_){
        vtpc_.viewPointCloud();
      }
      
    }
    
    
    cloud_pub_.publish(vtpc_.GetCloudMsg());

    
    //Publish PointCloud
    
    
    //ros::Duration(2.0).sleep();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void initialize(ros::NodeHandle &node){

  node.getParam("input_cloud", topic_cloud_sub_);
  node.getParam("output_cloud", topic_cloud_pub_);
  node.getParam("obj_detected_list", topic_objdet_pub_);
  node.getParam("view_pointcloud", viewPointCloudParam_);

  // Subscribers
  cloud_sub_ = node.subscribe(topic_cloud_sub_.c_str(), queue_size_, 
    &on_cloud_msg);
  // Publishers
  cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>(
    topic_cloud_pub_, queue_size_);

  obj_detected_pub_ = node.advertise<usv_perception::obj_detected_list>(
    topic_objdet_pub_, queue_size_);


  

  dock_corners_client_ =  node.serviceClient<usv_perception::dock_corners>("/get_dock_corners");
  vtpc_.setDockService(dock_corners_client_);
  
  //Success
  ROS_INFO("Velocity obstacle node is Ready!");
}

void on_cloud_msg(const sensor_msgs::PointCloud2::ConstPtr &msg){
  
  pcl::PCLPointCloud2 pcl_pc2;
  //From sensor_msgs::PointCloud2 to pcl::PCLPointCloud2 
  pcl_conversions::toPCL(*msg,pcl_pc2);
  //From pcl::PCLPointCloud2 to pcl::PointCloud<PointT>
  pcl::fromPCLPointCloud2(pcl_pc2,cloud_);  

}

