cmake_minimum_required(VERSION 3.8)
project(usv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(Boost 1.45.0 COMPONENTS filesystem)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(libs/usv_libs)
add_subdirectory(libs/a-star)

add_executable(asmc_node src/asmc_node.cpp)
target_include_directories(asmc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(asmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(asmc_node rclcpp std_msgs Eigen3 geometry_msgs)
target_link_libraries(asmc_node usv_libs)

add_executable(los_node src/los_node.cpp)
target_include_directories(los_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(los_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(los_node rclcpp nav_msgs std_msgs Eigen3 geometry_msgs)

add_executable(aitsmc_node src/aitsmc_node.cpp)
target_include_directories(aitsmc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(asmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(aitsmc_node rclcpp std_msgs Eigen3 geometry_msgs)
target_link_libraries(aitsmc_node usv_libs)

add_executable(aitsmc_new_node src/aitsmc_new_node.cpp)
target_include_directories(aitsmc_new_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(asmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(aitsmc_new_node rclcpp std_msgs Eigen3 geometry_msgs)
target_link_libraries(aitsmc_new_node usv_libs)

add_executable(usv_tf2_broadcaster_node src/usv_tf2_broadcaster_node.cpp)
target_compile_features(asmc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(usv_tf2_broadcaster_node rclcpp std_msgs nav_msgs Eigen3 geometry_msgs tf2 tf2_ros)

add_executable(dynamic_model_node src/dynamic_model_sim.cpp)
target_include_directories(dynamic_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(dynamic_model_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(dynamic_model_node rclcpp std_msgs Eigen3 geometry_msgs nav_msgs tf2_ros)
target_link_libraries(dynamic_model_node usv_libs)

add_executable(kinematic_model_node src/kinematic_model_sim.cpp)
target_include_directories(kinematic_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematic_model_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(kinematic_model_node rclcpp std_msgs Eigen3 geometry_msgs nav_msgs tf2_ros)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
target_include_directories(obstacle_avoidance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/a-star/source)
target_compile_features(obstacle_avoidance_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(obstacle_avoidance_node rclcpp std_msgs Eigen3 usv_interfaces geometry_msgs nav_msgs tf2_ros)
target_link_libraries(obstacle_avoidance_node a-star)

add_subdirectory(libs/fatrop)
add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node fatrop ${Boost_LIBRARIES})
target_include_directories(mpc_node PRIVATE deps/fatrop/fatrop ${Boost_INCLUDE_DIRS})
ament_target_dependencies(mpc_node rclcpp tf2 tf2_ros geometry_msgs nav_msgs usv_interfaces visualization_msgs)

add_executable(waypoint_handler_node src/waypoint_handler_node.cpp)
target_include_directories(waypoint_handler_node PRIVATE deps/fatrop/fatrop ${Boost_INCLUDE_DIRS})
ament_target_dependencies(waypoint_handler_node rclcpp tf2 tf2_ros geometry_msgs nav_msgs usv_interfaces visualization_msgs tf2_geometry_msgs)

add_executable(path_publisher_node src/path_publisher_node.cpp)
target_include_directories(path_publisher_node PRIVATE deps/fatrop/fatrop ${Boost_INCLUDE_DIRS})
ament_target_dependencies(path_publisher_node rclcpp tf2 tf2_ros geometry_msgs nav_msgs usv_interfaces visualization_msgs tf2_geometry_msgs)

# add_executable(path_maker_node src/path_maker_node.cpp)
# target_include_directories(path_maker_node PRIVATE deps/fatrop/fatrop ${Boost_INCLUDE_DIRS})
# ament_target_dependencies(path_maker_node rclcpp tf2 tf2_ros geometry_msgs nav_msgs usv_interfaces visualization_msgs tf2_geometry_msgs)


install(TARGETS
  asmc_node
  mpc_node
  los_node
  usv_tf2_broadcaster_node
  dynamic_model_node
  obstacle_avoidance_node
  kinematic_model_node
  waypoint_handler_node
  path_publisher_node
  # path_maker_node
  aitsmc_node
  aitsmc_new_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config 
  launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # scripts/path_publisher_node.py
  scripts/teleop_aitsmc_node.py
  scripts/teleop_can_node.py
  scripts/teleop_mpc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
