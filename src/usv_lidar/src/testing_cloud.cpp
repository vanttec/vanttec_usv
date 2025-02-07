#include <chrono>
#include <linux/limits.h>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class TestingCloud: public rclcpp::Node
{
  public:
    Eigen::MatrixXd cube;
    std::string test_file_name;

    TestingCloud()
    : Node("testing_cloud")
    {
      test_pointcloud_pub = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/test_pointcloud_pub",10);

      test_pointcloud_timer = this->create_wall_timer(
        100ms, std::bind(&TestingCloud::test_pointcloud_callback, this));

      this->declare_parameter("test_file_name", rclcpp::PARAMETER_STRING);
      rclcpp::Parameter str_param = this->get_parameter("test_file_name");
      test_file_name = str_param.as_string();

      cube = this->opening_file();

      RCLCPP_INFO(this->get_logger(), "Test file: %s", test_file_name.c_str());
      RCLCPP_INFO(this->get_logger(), "Starting TestingCloud Node");

    }

  private:

    Eigen::MatrixXd opening_file(){
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("point_cloud_processing");
      std::string file_path = package_share_directory + "/config/"+test_file_name;
      std::ifstream file(file_path); 
      if (!file) {
        std::cerr << "Error: Could not open the file!" << std::endl;
      }
      
      std::string line;
      std::vector<std::vector<float>> points;
      while (std::getline(file, line)) {
        std::vector<float> point;
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ' ')) {
          float coord = std::stof(token);
          point.push_back(coord);
        }
        points.push_back(point);
        if (!line.empty()) {
            // std::cout << line << std::endl; 
        }
      }
      file.close();
    
      int rows=points.size();
      Eigen::MatrixXd cube(rows,3);
      for(int i=0; i<rows; i++){
        std::vector<float> point = points[i];
        cube(i,0)=point[0]; cube(i,1)=point[1]; cube(i,2)=point[2];
        // RCLCPP_INFO(this->get_logger(), "Cube: %f %f %f", cube(i,0), cube(i,1), cube(i,2));
      }

      return cube;
    }

    float angleX = 0;
    // float angleZ = M_PI/100;
    float angleZ = 0;
    int iterator = 0;
    int prev_iterator = 0;
    void test_pointcloud_callback()
    {
      pcl::PointCloud<PointT> cloud;

      // Eigen::MatrixXd cube(8,3);
      // cube << 0.0, 0.0, 0.0, 
      //         0.0, 0.0, 1.0,
      //         0.0, 1.0, 0.0,
      //         1.0, 0.0, 0.0,
      //         0.0, 1.0, 1.0,
      //         1.0, 0.0, 1.0,
      //         1.0, 1.0, 0.0,
      //         1.0, 1.0, 1.0;

      Eigen::Matrix3d rotX(3,3);
      Eigen::Matrix3d rotZ(3,3);

      rotX << 1, 0, 0,
              0, cos(angleX), -sin(angleX),
              0, sin(angleX), cos(angleX);

      rotZ << cos(angleZ), -sin(angleZ), 0,
              sin(angleZ), cos(angleZ), 0,
              0, 0, 1;

      cube = cube * rotX;
      cube = cube * rotZ;


      int numPoints = cube.rows();
      for (int i=0; i < numPoints; i++){
        PointT point(cube(i,0), cube(i,1), cube(i,2));
        cloud.points.push_back(point);
      }


      sensor_msgs::msg::PointCloud2 ros_cloud;
      pcl::toROSMsg(cloud, ros_cloud);

      // ros_cloud.header.frame_id = "map";
      ros_cloud.header.frame_id = "world";
      ros_cloud.header.stamp = this->now();

      test_pointcloud_pub->publish(ros_cloud);

    }

    rclcpp::TimerBase::SharedPtr test_pointcloud_timer;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pointcloud_pub;


  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestingCloud>());
  rclcpp::shutdown();
  return 0;
}
