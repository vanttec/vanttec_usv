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
find_package(geometry_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(usv_missions src/usv_missions.cpp)
ament_target_dependencies(usv_missions rclcpp std_msgs geometry_msgs Eigen3)
target_compile_features(usv_missions PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(follow_the_path src/follow_the_path.cpp)
ament_target_dependencies(follow_the_path rclcpp std_msgs geometry_msgs usv_interfaces Eigen3)
target_compile_features(follow_the_path PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(obstacle_publisher_node src/obstacle_publisher.cpp)
ament_target_dependencies(obstacle_publisher_node rclcpp std_msgs geometry_msgs usv_interfaces Eigen3)
target_compile_features(obstacle_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  # usv_master
  usv_missions
  follow_the_path
  obstacle_publisher_node
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
