cmake_minimum_required(VERSION 3.8)

include(CMakePrintHelpers)

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

project(usv_perception LANGUAGES CXX CUDA)

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
find_package(visualization_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA 10.2 REQUIRED)
find_package(ZED 4 REQUIRED)

set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)

# add_executable(beeblebrox src_working/beeb_old.cpp)
# ament_target_dependencies(beeblebrox rclcpp std_msgs OpenCV cv_bridge image_transport CUDA ZED usv_interfaces)
# target_include_directories(beeblebrox PRIVATE ${CUDA_INCLUDE_DIRS})
# target_link_libraries(beeblebrox ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES} nvinfer nvinfer_plugin)

add_executable(beeblebrox src/beeblebrox.cpp)
ament_target_dependencies(beeblebrox rclcpp std_msgs OpenCV cv_bridge image_transport CUDA ZED usv_interfaces)
target_include_directories(beeblebrox PRIVATE ${CUDA_INCLUDE_DIRS})
target_link_libraries(beeblebrox ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES} nvinfer nvinfer_plugin)

add_executable(yolo src/detectors/yolo/yolo.cpp)
ament_target_dependencies(yolo rclcpp std_msgs OpenCV cv_bridge image_transport CUDA ZED usv_interfaces)
target_include_directories(yolo PRIVATE ${CUDA_INCLUDE_DIRS})
target_link_libraries(yolo ${CUDA_LIBRARIES} ${TensorRT_LIBRARIES} nvinfer nvinfer_plugin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install python scripts
file(GLOB SCRIPTS "scripts/*.py")
file(GLOB_RECURSE PY_DETECTORS "src/detectors/*.py")
install(PROGRAMS ${SCRIPTS} ${PY_DETECTORS} DESTINATION lib/${PROJECT_NAME})

# install c++ programs
install(TARGETS
  beeblebrox
  yolo
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and data folders
install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
