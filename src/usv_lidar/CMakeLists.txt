cmake_minimum_required(VERSION 3.8)
project(usv_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(usv_interfaces REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(clustering_segmentation src/clustering_segmentation.cpp)
ament_target_dependencies(clustering_segmentation rclcpp sensor_msgs pcl_conversions visualization_msgs tf2_geometry_msgs usv_interfaces)
target_link_libraries(clustering_segmentation ${PCL_LIBRARIES} )


add_executable(testing_cloud src/testing_cloud.cpp)
ament_target_dependencies(testing_cloud rclcpp sensor_msgs pcl_conversions Eigen3 ament_index_cpp)
target_link_libraries(testing_cloud ${PCL_LIBRARIES})

install(TARGETS
  clustering_segmentation testing_cloud 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

