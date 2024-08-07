cmake_minimum_required(VERSION 3.8)
project(usv_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs)
find_package(geometry_msgs)
find_package(sensor_msgs)
find_package(usv_interfaces)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(imu_converter_node src/imu_converter_node.cpp)
target_include_directories(imu_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_converter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  imu_converter_node
  tf2
  tf2_ros
  nav_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

add_executable(odom_converter_node src/odom_converter_node.cpp)
target_include_directories(odom_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(odom_converter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  odom_converter_node
  tf2
  tf2_ros
  nav_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

add_executable(killswitch_node src/killswitch_node.cpp)
target_include_directories(killswitch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(killswitch_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  killswitch_node
  tf2
  tf2_ros
  nav_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

add_executable(obstacle_viewer_node src/obstacle_viewer_node.cpp)
ament_target_dependencies(obstacle_viewer_node rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs usv_interfaces Eigen3)
target_compile_features(obstacle_viewer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  imu_converter_node
  odom_converter_node
  killswitch_node
  obstacle_viewer_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
