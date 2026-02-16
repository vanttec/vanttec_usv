cmake_minimum_required(VERSION 3.8)
project(usv_robocommand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(Protobuf REQUIRED)

# Path to the submodule directory and .proto file
set(PROTO_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../deps/robocommand")
set(PROTO_FILE "${PROTO_BASE_DIR}/proto/report.proto")

# Handle potential imports from the .proto files
set(PROTOBUF_IMPORT_DIRS "${PROTO_BASE_DIR}")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS "${PROTO_FILE}")

add_executable(heartbeat_node 
  src/heartbeat.cpp
  ${PROTO_SRCS})

target_include_directories(heartbeat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  ${PROTOBUF_INCLUDE_DIRS}
)

target_link_libraries(heartbeat_node ${PROTOBUF_LIBRARIES})

ament_target_dependencies(heartbeat_node rclcpp sbg_driver geometry_msgs sensor_msgs std_msgs usv_interfaces)

install(TARGETS
  heartbeat_node
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
