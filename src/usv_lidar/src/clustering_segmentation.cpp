#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include "usv_interfaces/msg/clustering_segmentation.hpp"

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
  public:

    std_msgs::msg::Header global_header;

    VoxelGrid_filter()
    : Node("minimal_publisher")
    {

      /* Timers */
      axis_timer = this->create_wall_timer(
        1000ms, std::bind(&VoxelGrid_filter::axis_callback, this));

      /* Parameters */
      this->declare_parameter("voxel_grid_x",rclcpp::PARAMETER_DOUBLE); // Voxel
      this->declare_parameter("voxel_grid_y",rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("voxel_grid_z",rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("ksearch",rclcpp::PARAMETER_INTEGER); // Normals
      this->declare_parameter("normal_distance_weight",rclcpp::PARAMETER_DOUBLE); // Plane Segmentation
      this->declare_parameter("max_iterations",rclcpp::PARAMETER_INTEGER);
      this->declare_parameter("distance_threshold",rclcpp::PARAMETER_DOUBLE);
      this->declare_parameter("cluster_tolerance",rclcpp::PARAMETER_DOUBLE); // Cluster
      this->declare_parameter("min_cluster",rclcpp::PARAMETER_INTEGER);
      this->declare_parameter("max_cluster",rclcpp::PARAMETER_INTEGER);

      /* Subscriptions */
      // subscription_ =
      // this->create_subscription<sensor_msgs::msg::PointCloud2>(
      // "/kitti/point_cloud", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      // subscription_ = // From .txt file
      // this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //   "/test_pointcloud_pub", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      subscription_ = // Gazebo
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/points", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      // subscription_ =  // Real life
      // this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //   "velodyne_points", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      /* Publishers */
      clusters_seg_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/clusters", 10);

      raw_input_pub = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_input", 10);

       boxes_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("boxes_marker_array", 10);

       cropped_cloud_pup = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropped_cloud", 10);

       axis_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/axis_marker_array", 10);

      normals_cloud_pub=
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/normals_cloud_pub", 10);

      voxel_grid_pub = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_grid", 10);

      plane_normal_pub = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_normal", 10);

      clust_seg_data_pub = 
        this->create_publisher<usv_interfaces::msg::ClusteringSegmentation>("/clust_seg_data", 10);

      RCLCPP_INFO(this->get_logger(), "Starting VoxelGrid_filter Node");

    }

  private:

    void print_PointCloud(pcl::PointCloud<PointT> cloud, int points){
      if (points <= int(cloud.size())){
        for(int i=0; i<=points; i++){
          PointT point = cloud.points[i];
          RCLCPP_INFO(get_logger(), "P[%d] %f, %f, %f \n", i, point.x, point.y, point.z);
        }
      }
    }

    std::vector<float> normal2rpy(float x, float y, float z){
      float roll = std::atan2(z,y); 
      float pitch = std::atan2(x,z);
      float yaw = std::atan2(y,x);

      std::vector<float> angles = {roll, pitch, yaw};
      return angles;
    }

    float deg2rad(float deg){
      return deg * (M_PI/180);
    }

    void axis_callback(){
      std_msgs::msg::Header header = global_header;
      visualization_msgs::msg::MarkerArray axis_marker_array;
      visualization_msgs::msg::Marker x_axis_marker;
      visualization_msgs::msg::Marker y_axis_marker;
      visualization_msgs::msg::Marker z_axis_marker;

      // x_axis_marker
      // x_axis_marker.header.frame_id = "map";
      x_axis_marker.header = header;
      x_axis_marker.ns = "axis";
      x_axis_marker.id = 1;
      x_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
      x_axis_marker.action = visualization_msgs::msg::Marker::ADD;
      x_axis_marker.scale.x = 0.05;
      x_axis_marker.color.r = 1;
      x_axis_marker.color.g = 0;
      x_axis_marker.color.b = 0;
      x_axis_marker.color.a = 1.0;

      geometry_msgs::msg::Point startX, endX, extra1, extra2;
      startX.x = 0; startX.y = 0; startX.z = 0;
      endX.x = 7.5; endX.y = 0; endX.z = 0;
      x_axis_marker.points.push_back(startX);
      x_axis_marker.points.push_back(endX);

      // y_axis_marker
      // y_axis_marker.header.frame_id = "map";
      y_axis_marker.header = header;
      y_axis_marker.ns = "axis";
      y_axis_marker.id = 2;
      y_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
      y_axis_marker.action = visualization_msgs::msg::Marker::ADD;
      y_axis_marker.scale.x = 0.05;
      y_axis_marker.color.r = 0;
      y_axis_marker.color.g = 0;
      y_axis_marker.color.b = 1;
      y_axis_marker.color.a = 1.0;

      geometry_msgs::msg::Point startY, endY;
      startY.x = 0; startY.y = 0; startY.z = 0;
      endY.x = 0; endY.y = 3.5; endY.z = 0;
      y_axis_marker.points.push_back(startY);
      y_axis_marker.points.push_back(endY);

      // z_axis_marker
      // z_axis_marker.header.frame_id = "map";
      z_axis_marker.header = header;
      z_axis_marker.ns = "axis";
      z_axis_marker.id = 3;
      z_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
      z_axis_marker.action = visualization_msgs::msg::Marker::ADD;
      z_axis_marker.scale.x = 0.05;
      z_axis_marker.color.r = 1;
      z_axis_marker.color.g = 0;
      z_axis_marker.color.b = 0;
      z_axis_marker.color.a = 1.0;

      geometry_msgs::msg::Point startZ, endZ;
      startZ.x = 0; startZ.y = 0; startZ.z = 0;
      endZ.x = 0; endZ.y = 0; endZ.z = 1;
      z_axis_marker.points.push_back(startZ);
      z_axis_marker.points.push_back(endZ);

      axis_marker_array.markers.push_back(x_axis_marker);
      axis_marker_array.markers.push_back(y_axis_marker);
      // axis_marker_array.markers.push_back(z_axis_marker);

      axis_pub->publish(axis_marker_array);
    }

    int my_map(int maxI, int minI, int maxO, int minO, int input){
      return static_cast<int>( (maxO-minO)*((input-minI)/(maxI-minI))+minO );
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualize_curvature(pcl::PointCloud<pcl::Normal>::Ptr normal){

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (int i=0; i<int(normal->size()); i++){
        pcl::PointXYZRGB p;
        p.x = normal->points[i].normal_x;
        p.y = normal->points[i].normal_y;
        p.z = normal->points[i].normal_z;

        float curvature = normal->points[i].curvature;
        int curvature_int = static_cast<int>(curvature * 100);

        curvature_int = (curvature_int > 100) ? 100 : curvature_int;
        curvature_int = (curvature_int < 0) ? 0 : curvature_int;

        int r = 255; int g = 0; int b = 0;
        int first_mid = 33; int second_mid = 66;
        int percentage = 0;
        if (curvature_int > second_mid){
          percentage = my_map(100, second_mid, 255, 0, curvature_int);
          r = percentage; g = 0; b = 0;
        }
        if (curvature_int > first_mid && curvature_int <= second_mid){
          percentage = my_map(second_mid, first_mid, 255, 0, curvature_int);
          r = 0; g = percentage; b = 0;
        }
        if (curvature_int <= first_mid){
          percentage = my_map(first_mid, 0, 255, 0, curvature_int);
          r = 0; g = 0; b = percentage;
        }

        p.r = r; p.g = g; p.b = b;
        cloud_colored->push_back(p);
      }

      return cloud_colored;
    }

    void draw_normals(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<PointT>::Ptr cloud){
      visualization_msgs::msg::MarkerArray normal_array_markers;
      std_msgs::msg::Header header = global_header;

      int sumRC = 0; int sumBC = 0; int sumGC = 0; 
      int sumR = 0; int sumB = 0; int sumG = 0; 
      int multiplier = 1e3;
      int maxC = 1; int minC = multiplier; int sumC = 0;
      // float avgR; float avgG; float avgB; float avgC;
      for (int i = 0; i<int(cloud->size()); i++) {
          visualization_msgs::msg::Marker normal_marker;
          normal_marker.header = global_header;
          normal_marker.ns = "normal_axis";
          normal_marker.id = i;
          normal_marker.type = visualization_msgs::msg::Marker::ARROW;
          normal_marker.action = visualization_msgs::msg::Marker::ADD;
          normal_marker.scale.x = 0.02;
          normal_marker.scale.y = 0.03;

          // Color according to curvature
          float curvature = normals->points[i].curvature;
          int curvature_int = static_cast<int>(curvature * multiplier);

          maxC = (maxC > curvature_int) ? maxC : curvature_int;
          minC = (minC < curvature_int) ? minC : curvature_int;
          sumC += curvature_int;

          curvature_int = (curvature_int > multiplier) ? multiplier: curvature_int;
          curvature_int = (curvature_int < 0) ? 0 : curvature_int;

          int r = 255; int g = 0; int b = 0;
          int max = static_cast<int>(multiplier * 0.4);
          int first_mid = static_cast<int>(max/3); int second_mid = static_cast<int>(max/3) * 2;
          int percentage = 0;
          if (curvature_int > second_mid){ // Sharp features RED
            sumR++;
            percentage = my_map(max, second_mid, 255, 128, curvature_int);
            sumRC = sumRC + percentage;
            r = percentage; g = 0; b = 0;
          }
          if (curvature_int > first_mid && curvature_int <= second_mid){ // Moderate Bending GREEN
            sumG++;
            percentage = my_map(second_mid, first_mid, 255, 128, curvature_int);
            sumGC = sumGC + percentage;

            r = 0; g = percentage; b = 0;
          }
          if (curvature_int <= first_mid){ // Relatively flat BLUE
            sumB++;
            percentage = my_map(first_mid, 0, 255, 128, curvature_int);
            sumBC = sumBC + percentage;

            r = 0; g = 0; b = percentage;
          }

          normal_marker.color.r = r/255.0;
          normal_marker.color.g = g/255.0;
          normal_marker.color.b = b/255.0;
          normal_marker.color.a = 1.0;

          float scale = 0.3;
          PointT org = cloud->points[i];
          pcl::Normal end = normals->points[i];
          geometry_msgs::msg::Point s, e;
          s.x = org.x; s.y = org.y; s.z = org.z;
          e.x = (end.normal_x * scale) + org.x; e.y = (end.normal_y * scale) + org.y; e.z = (end.normal_z * scale) + org.z;
          normal_marker.points.push_back(s);
          normal_marker.points.push_back(e);

          normal_array_markers.markers.push_back(normal_marker);

      }

      // avgR = (sumR > 0) ? sumRC/sumR : 0;
      // avgG = (sumG > 0) ? sumGC/sumG : 0;
      // avgB = (sumB > 0) ? sumBC/sumB : 0;
      // avgC = sumC/normals->size() * 1.0;

      // RCLCPP_INFO(this->get_logger(), "R: %d[%.2f], G:%d[%.2f], B:%d[%.2f]", sumR, avgR, sumG, avgG, sumB, avgB);
      // RCLCPP_INFO(this->get_logger(), "Max: %d, Min:%d, Avg: %f", maxC, minC, avgC);

      normal_array_markers.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

      normals_cloud_pub->publish(normal_array_markers);
    }


    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
      {
        global_header = input_cloud->header;
        usv_interfaces::msg::ClusteringSegmentation params_msg;
        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>) ; // Original Cloud
        pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>); // Cropped Cloud

        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
        RCLCPP_INFO(this->get_logger(),"Original size: %ld", pcl_cloud->size());

  //==================================== Displaying raw data ====================================

        sensor_msgs::msg::PointCloud2 raw_data;
        raw_data = *input_cloud;
        // raw_input_pub ->publish(raw_data);

  //==================================== Pre Processing Data ====================================
        // Along X Axis
        pcl::PassThrough<PointT> passing_x;
        pcl::PassThrough<PointT> passing_y;
        int radius = 2.5;
        passing_x.setInputCloud(pcl_cloud);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-radius,radius);
        passing_x.filter(*cropped_cloud);
        
        // Along Y Axis
        passing_y.setInputCloud(cropped_cloud);
        // passing_y.setInputCloud(pcl_cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-radius,radius);
        passing_y.filter(*cropped_cloud);

        sensor_msgs::msg::PointCloud2 ros_cropped_cloud;
        pcl::toROSMsg(*cropped_cloud, ros_cropped_cloud);
        cropped_cloud_pup->publish(ros_cropped_cloud);

        // RCLCPP_INFO(this->get_logger(),"Cropped size: %ld", cropped_cloud->size());

        // Voxel Filter
        rclcpp::Parameter voxel_grid_x_param = this->get_parameter("voxel_grid_x");
        float voxel_grid_x = voxel_grid_x_param.as_double();
        params_msg.voxel_grid_x = voxel_grid_x;
        rclcpp::Parameter voxel_grid_y_param = this->get_parameter("voxel_grid_y");
        float voxel_grid_y = voxel_grid_y_param.as_double();
        params_msg.voxel_grid_y = voxel_grid_y;
        rclcpp::Parameter voxel_grid_z_param = this->get_parameter("voxel_grid_z");
        float voxel_grid_z = voxel_grid_z_param.as_double();
        params_msg.voxel_grid_z = voxel_grid_z;

        pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // Voxel Cloud
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(voxel_grid_x ,voxel_grid_y ,voxel_grid_z);
        // voxel_filter.setLeafSize(1.0 ,1.0 ,1.0);
        voxel_filter.filter(*voxel_cloud);

        sensor_msgs::msg::PointCloud2 ros_voxel_cloud;
        pcl::toROSMsg(*voxel_cloud, ros_voxel_cloud);
        voxel_grid_pub->publish(ros_voxel_cloud);

        RCLCPP_INFO(this->get_logger(),"Voxel size: %ld", voxel_cloud->size());
        
  //==================================== Plane Segmentation  ====================================
        pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> plane_seg_frm_normals;
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> plane_extract_indices;
        pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);


        // Normals Extractions
        rclcpp::Parameter ksearch_param = this->get_parameter("ksearch");
        float ksearch = ksearch_param.as_int();
        params_msg.ksearch = ksearch;
        normal_extractor.setSearchMethod(tree);
        normal_extractor.setInputCloud(voxel_cloud);
        normal_extractor.setKSearch(ksearch);
        normal_extractor.compute(*normals);

        draw_normals(normals, voxel_cloud);

        // Parameters for Planar Segmentation
        rclcpp::Parameter normal_distance_weight_param = this->get_parameter("normal_distance_weight");
        float normal_distance_weight = normal_distance_weight_param.as_double();
        params_msg.normal_distance_weight = normal_distance_weight;
        rclcpp::Parameter max_iterations_param = this->get_parameter("max_iterations");
        int max_iterations = max_iterations_param.as_int();
        params_msg.max_iterations = max_iterations;
        rclcpp::Parameter distance_threshold_param = this->get_parameter("distance_threshold");
        float distance_threshold = distance_threshold_param.as_double();
        params_msg.distance_threshold = distance_threshold;

        plane_seg_frm_normals.setOptimizeCoefficients(true);
        plane_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        plane_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
        plane_seg_frm_normals.setNormalDistanceWeight(normal_distance_weight);
        plane_seg_frm_normals.setMaxIterations(max_iterations);
        plane_seg_frm_normals.setDistanceThreshold(distance_threshold);
        plane_seg_frm_normals.setInputCloud(voxel_cloud);
        plane_seg_frm_normals.setInputNormals(normals);
        plane_seg_frm_normals.segment(*plane_inliers,*plane_coefficients);

        //Extracting Cloud based on Inliers indices
        plane_extract_indices.setInputCloud(voxel_cloud);
        plane_extract_indices.setIndices(plane_inliers);
        plane_extract_indices.setNegative(true);
        plane_extract_indices.filter(*plane_cloud);

        sensor_msgs::msg::PointCloud2 ros_plane_normals;
        pcl::toROSMsg(*plane_cloud, ros_plane_normals);
        ros_plane_normals.header = input_cloud->header;
        plane_normal_pub->publish(ros_plane_normals);
        RCLCPP_INFO(this->get_logger(),"plane size: %ld", plane_cloud->size());
  //==================================== Clusters Segmentation  ====================================
    pcl::PointCloud<PointT>::Ptr segmented_cluster (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
    tree->setInputCloud (plane_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;


        struct BBox
    {
      float x_min;
      float x_max;
      float y_min;
      float y_max;
      float z_min;
      float z_max;
      double r = 1.0;
      double g = 0.0;
      double b = 0.0;
    };

    rclcpp::Parameter cluster_tolerance_param = this->get_parameter("cluster_tolerance");
    float cluster_tolerance = cluster_tolerance_param.as_double();
    params_msg.cluster_tolerance = cluster_tolerance;
    rclcpp::Parameter max_cluster_param = this->get_parameter("max_cluster");
    float max_cluster = max_cluster_param.as_int();
    params_msg.max_cluster = max_cluster;
    rclcpp::Parameter min_cluster_param = this->get_parameter("min_cluster");
    float min_cluster = min_cluster_param.as_int();
    params_msg.min_cluster = min_cluster;

    ec.setClusterTolerance (cluster_tolerance); 
    ec.setMinClusterSize (min_cluster);
    ec.setMaxClusterSize (max_cluster);
    ec.setSearchMethod (tree);
    ec.setInputCloud (plane_cloud);
    ec.extract (cluster_indices);
    std::vector<BBox> bboxes;

    size_t min_reasonable_size = size_t(min_cluster * 1.1);
    size_t max_reasonable_size = size_t(max_cluster * 0.9);
    int num_reasonable_clusters = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        if (cluster_indices[i].indices.size() > min_reasonable_size && cluster_indices[i].indices.size() < max_reasonable_size)
        {
            pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
            extract.setInputCloud (plane_cloud);
            extract.setIndices(indices);
            extract.setNegative (false);
            extract.filter (*reasonable_cluster);
            all_clusters->operator+=(*reasonable_cluster);
            num_reasonable_clusters++;

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

            pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
            BBox bbox;
            bbox.x_min = min_pt[0];
            bbox.y_min = min_pt[1];
            bbox.z_min = min_pt[2];
            bbox.x_max = max_pt[0];
            bbox.y_max = max_pt[1];
            bbox.z_max = max_pt[2];

            bboxes.push_back(bbox);
        }
    }
    RCLCPP_INFO(this->get_logger(),"Clusters: %d", num_reasonable_clusters);

    //==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = input_cloud->header;
    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 0.0;
        connecting_lines_marker.color.g = 1.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.4;
        corner_marker.scale.y = 0.4;
        corner_marker.scale.z = 0.4;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        boxes_pub->publish(marker_array);
    }
  //==================================== Cloud publishing to ROS  ====================================

        // Convert cloud to ros2 message
        sensor_msgs::msg::PointCloud2 clusters_seg_ros2;
        pcl::toROSMsg(*all_clusters, clusters_seg_ros2);
        clusters_seg_ros2.header = input_cloud->header;
        // std::cout << "PointCloud size before voxelization: " << pcl_cloud->size() << std::endl;
        // std::cout << "PointCloud size after voxelization: " << voxel_cloud->size() << std::endl;

        clusters_seg_pub->publish(clusters_seg_ros2);
        
        clust_seg_data_pub->publish(params_msg);
        
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr axis_timer;
  /* Subscriptions */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  /* Publishers */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boxes_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axis_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normals_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_seg_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_input_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pup;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_normal_pub;
  rclcpp::Publisher<usv_interfaces::msg::ClusteringSegmentation>::SharedPtr clust_seg_data_pub;


  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}
