cmake_minimum_required(VERSION 3.8)
project(usv_missions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mission_handler_node src/mission_handler_node.cpp)
ament_target_dependencies(mission_handler_node rclcpp std_msgs geometry_msgs visualization_msgs usv_interfaces Eigen3)
target_compile_features(mission_handler_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(obstacle_publisher_node src/obstacle_publisher.cpp)
ament_target_dependencies(obstacle_publisher_node rclcpp std_msgs std_srvs geometry_msgs visualization_msgs usv_interfaces Eigen3)
target_compile_features(obstacle_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(global_obstacle_register_node src/global_obstacle_register_node.cpp)
ament_target_dependencies(global_obstacle_register_node rclcpp std_msgs std_srvs geometry_msgs visualization_msgs usv_interfaces Eigen3)
target_compile_features(global_obstacle_register_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  mission_handler_node
  obstacle_publisher_node
  global_obstacle_register_node
  DESTINATION lib/${PROJECT_NAME})

# Resources
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

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
