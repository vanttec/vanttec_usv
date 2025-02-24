cmake_minimum_required(VERSION 3.8)
project(usv_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
  rclcpp
  sensor_msgs
  tf2
  nav_msgs
  std_srvs
  tf2_ros
  Eigen3
  tf2_geometry_msgs
  usv_interfaces
)

add_executable(imu_converter_node src/imu_converter_node.cpp)
ament_target_dependencies(imu_converter_node ${DEPENDENCIES})

add_executable(odom_converter_node src/odom_converter_node.cpp)
ament_target_dependencies(odom_converter_node ${DEPENDENCIES})

add_executable(killswitch_node src/killswitch_node.cpp)
ament_target_dependencies(killswitch_node ${DEPENDENCIES})

add_executable(obstacle_viewer_node src/obstacle_viewer_node.cpp)
ament_target_dependencies(obstacle_viewer_node ${DEPENDENCIES})

add_executable(obstacle_nearest_publisher src/obstacle_nearest_publisher.cpp)
ament_target_dependencies(obstacle_nearest_publisher ${DEPENDENCIES})

install(TARGETS 
  imu_converter_node
  odom_converter_node
  killswitch_node
  obstacle_viewer_node
  obstacle_nearest_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
