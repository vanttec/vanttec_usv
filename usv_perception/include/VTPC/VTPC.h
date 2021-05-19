/** ----------------------------------------------------------------------------
 * @file:     VTPC.h
 * @date:     Octubre 8, 2020
 * @coauthor: Ivana Collado
 * @email:    a00569475@itesm.mx
 * @coauthor: Rodolfo Cuan
 * @email:    a01233155@itesm.mx
 * 
 * @brief: Contains de definition of the VantTec Point Cloud (VTPC) class.
 * ---------------------------------------------------------------------------*/

// IFNDEF ----------------------------------------------------------------------
#ifndef VTPC_H_
#define VTPC_H_

// INCLUDES --------------------------------------------------------------------
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>


//Custom Msgs
#include <usv_perception/obj_detected.h>
#include <usv_perception/obj_detected_list.h>
#include <usv_perception/dock_corners.h>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>

// STD
#include <vector> 
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <boost/random.hpp> 

//OpenCV

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//Other Classes
#include "VTPC/ObjectDetectionVTPC.h"

// NAMESPACES ------------------------------------------------------------------

using namespace std; 
using namespace cv;

// STRUCTS ---------------------------------------------------------------------

// CLASS DECLARATION -----------------------------------------------------------
template<class PointType>
class VTPC {
public:
  // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------------
  /**
    * Constructor. 
    */
  VTPC();
  /**
   * Destructor
   * */
  ~VTPC();  


  // FUNCTIONS -----------------------------------------------------------------
  /** 
   * Cloud Setter
   * @param in_cloud[in]: PointCloud to process.
   */
  void SetCloud(const pcl::PointCloud<PointType> &in_cloud);
  /** 
   * Cloud Getter
   * @return pcl::PointCloud<PointType>[out]: processed pointcloud in PCL format. 
   */
  pcl::PointCloud<PointType> GetCloud();
  /**
   * Cloud Sensor Msg Getter
   * @return sensor_msgs::PointCloud2[out]: processed pointcloud as a ROS msg. 
   */
  sensor_msgs::PointCloud2 GetCloudMsg();
  /** 
   * Function to filter PointCloud in 1 Dimension 
   * @param dim[in]: dimension to filter (x, y or z)
   * @param minLim[in]: Minimum limit in meters
   * @param maxLim[in]: Maximum limit in meters
   * @return void
   */
  void PassThrough( const std::string &dim, const float &minLim,
                    const float &maxLim);
  /** 
   * Function to filter PointCloud in a CropBox
   * @param minX[in]: Minimum limits in X in meters
   * @param maxX[in]: Maximum limits in X in meters
   * @param minY[in]: Minimum limits in Y in meters
   * @param maxY[in]: Maximum limits in Y in meters
   * @param minZ[in]: Minimum limits in Z in meters
   * @param maxZ[in]: Maximum limits in Z in meters
   * @return void
   */
  void CropBox( const float &minX,const float &maxX,const float &minY,
                const float &maxY,const float &minZ,const float &maxZ);
  /** 
   * Function to filter PointCloud within Radius
   * @param radius[in]: radius in meters to be filtered
   * @return void
   */
  void RadiusFilter(const float &radius);
  /** 
   * Downsample PointCloud using Voxel Gridding 
   * @param leaf_size[in]: Size of leaf to downsample (resolution).
   * @return void
   */
  void DownsampleVoxelGrid(const float &leaf_size);
  /**
   * Separates Point Cloud into 2D Plane Grid
   * @param gridDim[in]: Size of Grid in meters
   * @return void
   */
  void CreateGrid(const float &gridDim); 
  /**
   * Cluster objects on gridImg_
   * @param tracking[in]: Activate Tracking Mode
   * @return void
   */
  void ClusterGrid(const bool &tracking = false); 
  /**
   * Uses PCL Visualizer to look at cloudPtr_
   * @return void
   */
  void viewPointCloud();
  /**
   * Classifies and Publishes the objects detected along their location
   * @param objDetPub[in]: Ros Publisher which will publish the objects detected
   * @return void
   */
  void ObjectDetectionPublish(const ros::Publisher &objDetPub);
  /**
   * Given a gridObj it Classifies it by its dimension as a marker, bouy or marker. 
   * @param obj[in]: gridObj in quary
   * @return string: with the class.
   */
  string ObjectClassifier(const gridObj &obj);
  /**
   * Transforms the objet'c pointcloud to an image and sent to the Dock Server
   */
  std::vector<pcl::PointXYZ> FindDockCorners(const gridObj  &obj);
  /**
   * Set the server to calculate the dock Corners
   * @param dockClientServer[in]: Dock Calc Server
   * @return void
   */
  void setDockService(const ros::ServiceClient &dockClientServer);

  /**
   * Adds random noise
   * @param n[in]: number of noise points
   * @return void
   */
  void addNoise(const int &n); 

  // MEMBERS -------------------------------------------------------------------
  


private:
  // FUNCTIONS -----------------------------------------------------------------

  /**
   * Erase objects from objDetVec_ which have not been detected for the last 
   * number of seconds indicated.
   * @param seconds[in] 
   * @return void
   */ 
  void eraseDisapearingObjects(const float &seconds);

  // MEMBERS -------------------------------------------------------------------
    /** 
   * Cloud Object to be processed
   */
  typename pcl::PointCloud<PointType>::Ptr cloudPtr_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;


  // MEMBERS FOR OBJECT DETECTION ----------------------------------------------

  /**
   * Current grid containing the pointcloud and image representation
   */ 
  gridObj grid_[256][256];
  uint8_t gridImg_[256][256];

  /**
   * Previous grid information
   */ 
  gridObj prevGrid_[256][256];
  uint8_t prevGridImg_[256][256];
  
  /**
   * Map storing the detected objects. <Group Number, < time of frame, Object> >.
   */ 
  map<int, pair<float, gridObj>> objDetVec_;


  /**
   * Group Number counter. 
   */
  int groupNumCnt_;

  /**
   * Flag to start tracking in the second frame.
   */ 
  bool isFirstIteration_;


  /**
   * ROS Server for calculating dock corners
   */
  ros::ServiceClient dock_service_;


}; // End of class VTPC

#endif