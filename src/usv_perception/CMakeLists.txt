cmake_minimum_required(VERSION 3.8)
project(usv_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common features filters io segmentation surface)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3)

# ament_python_install_package(${PROJECT_NAME})

add_executable(yolo src/yolo.cpp)
ament_target_dependencies(yolo rclcpp std_msgs OpenCV cv_bridge image_transport)

add_executable(pcd_viz src/pcd_visualizer.cpp)
ament_target_dependencies(pcd_viz rclcpp sensor_msgs Eigen3 pcl_conversions tf2 tf2_geometry_msgs tf2_ros PCL)
target_link_libraries(pcd_viz ${PCL_LIBRARIES})

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

# install python scripts
file(GLOB SCRIPTS "scripts/*.py")
install(PROGRAMS ${SCRIPTS} DESTINATION lib/${PROJECT_NAME})

# install c++ programs
install(TARGETS
  yolo
  pcd_viz
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and data folders
install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
