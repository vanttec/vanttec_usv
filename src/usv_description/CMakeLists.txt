cmake_minimum_required(VERSION 3.8)
project(usv_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# COPIED FROM VRX_GZ's github repository
find_package(ament_cmake_python REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# Custom Bridges
find_package(rclcpp)
find_package(sensor_msgs REQUIRED)


#============================================================================
# Resources
install(DIRECTORY
  launch
  urdf
  meshes
  models
  worlds
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# install(
#   DIRECTORY
#     launch/
#     rviz/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# install(
#   DIRECTORY
#   DESTINATION share/${PROJECT_NAME}/rviz
# )

# install(
#   DIRECTORY
#     models/
#   DESTINATION share/${PROJECT_NAME}/models
# )

# install(
#   DIRECTORY
#     worlds/
#   DESTINATION share/${PROJECT_NAME}/worlds
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
