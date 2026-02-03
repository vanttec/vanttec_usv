cmake_minimum_required(VERSION 3.8)
project(usv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(Boost 1.45.0 COMPONENTS filesystem)

find_path(FATROP_INCLUDE_DIR fatrop/fatrop.hpp PATHS /usr/local/include)
find_library(FATROP_LIBRARY fatrop PATHS /usr/local/lib)

add_subdirectory(libs/usv_libs)
add_subdirectory(libs/a-star)

set(DEPENDENCIES
  ament_cmake
  geometry_msgs
  visualization_msgs
  std_msgs
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  std_srvs
  Eigen3
  tf2_geometry_msgs
  usv_interfaces
)

# Low-level controller nodes
add_executable(asmc_node src/asmc_node.cpp)
target_include_directories(asmc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(asmc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(asmc_node ${DEPENDENCIES})
target_link_libraries(asmc_node usv_libs)

add_executable(los_node src/los_node.cpp)
target_include_directories(los_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(los_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(los_node ${DEPENDENCIES})

add_executable(aitsmc_node src/aitsmc_node.cpp)
target_include_directories(aitsmc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(aitsmc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(aitsmc_node ${DEPENDENCIES})
target_link_libraries(aitsmc_node usv_libs)

add_executable(aitsmc_new_node src/aitsmc_new_node.cpp)
target_include_directories(aitsmc_new_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(aitsmc_new_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(aitsmc_new_node ${DEPENDENCIES})
target_link_libraries(aitsmc_new_node usv_libs)

add_executable(pid_node src/pid_node.cpp)
target_include_directories(pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(pid_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pid_node ${DEPENDENCIES})
target_link_libraries(pid_node usv_libs)

# High-level controller nodes
# add_executable(mpc_node src/mpc_node.cpp)
# target_link_libraries(mpc_node ${FATROP_LIBRARY} ${Boost_LIBRARIES})
# target_include_directories(mpc_node PRIVATE ${FATROP_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
# target_compile_options(mpc_node PRIVATE ${FATROP_CFLAGS_OTHER})
# ament_target_dependencies(mpc_node ${DEPENDENCIES})

# ================
# ||| MPC Node |||
# ================
# NOTE: MPC node disabled - requires acados installation
# To enable, install acados and update ACADOS_INSTALL_DIR path

# # Find ACADOS
# set(ACADOS_INSTALL_DIR "/home/max/acados")
# include_directories(
#   ${ACADOS_INSTALL_DIR}/include
#   ${ACADOS_INSTALL_DIR}/include/acados
#   ${ACADOS_INSTALL_DIR}/include/blasfeo/include
#   ${ACADOS_INSTALL_DIR}/include/hpipm/include
# )
# link_directories(${ACADOS_INSTALL_DIR}/lib)

# # Include generated code directory
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/c_generated_code_asv_ocp)
# # Add all generated source files
# file(GLOB ACADOS_GENERATED_SOURCES
#   ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/c_generated_code_asv_ocp/acados_*.c
#   ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/c_generated_code_asv_ocp/*/*.c
# )

# add_executable(mpc_node src/mpc_node.cpp ${ACADOS_GENERATED_SOURCES})
# ament_target_dependencies(mpc_node ${DEPENDENCIES})
# target_link_libraries(mpc_node acados hpipm blasfeo m)


add_executable(a_star_avoidance_node src/a_star_avoidance_node.cpp)
target_include_directories(a_star_avoidance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/a-star/source)
target_compile_features(a_star_avoidance_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(a_star_avoidance_node ${DEPENDENCIES})
target_link_libraries(a_star_avoidance_node a-star)

# Dynamic / kinematic models
add_executable(dynamic_model_node src/dynamic_model_sim.cpp)
target_include_directories(dynamic_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/usv_libs/src)
target_compile_features(dynamic_model_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dynamic_model_node ${DEPENDENCIES})
target_link_libraries(dynamic_model_node usv_libs)

add_executable(kinematic_model_node src/kinematic_model_sim.cpp)
target_include_directories(kinematic_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematic_model_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(kinematic_model_node ${DEPENDENCIES})


# Supporting nodes
add_executable(usv_tf2_broadcaster_node src/usv_tf2_broadcaster_node.cpp)
ament_target_dependencies(usv_tf2_broadcaster_node ${DEPENDENCIES})

add_executable(waypoint_handler_node src/waypoint_handler_node.cpp)
ament_target_dependencies(waypoint_handler_node ${DEPENDENCIES})

add_executable(path_publisher_node src/path_publisher_node.cpp)
ament_target_dependencies(path_publisher_node ${DEPENDENCIES})

add_executable(spline_publisher_node src/spline_publisher_node.cpp)
ament_target_dependencies(spline_publisher_node ${DEPENDENCIES})

install(TARGETS
  asmc_node
  pid_node
  # mpc_node  # Disabled - requires acados
  los_node
  usv_tf2_broadcaster_node
  dynamic_model_node
  a_star_avoidance_node
  kinematic_model_node
  waypoint_handler_node
  path_publisher_node
  aitsmc_node
  aitsmc_new_node
  spline_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config 
  launch
  scripts
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/teleop/teleop_aitsmc_node.py
  scripts/teleop/teleop_can_node.py
  scripts/teleop/teleop_mpc_node.py

  scripts/mpc/mpc_gui.py

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
