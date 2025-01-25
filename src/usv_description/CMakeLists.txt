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

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools8 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat13 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================

# Waves
add_library(Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Eigen3::Eigen
)
install(
  TARGETS Waves
  DESTINATION lib)

# Buoyancy
add_library(PolyhedraBuoyancyDrag SHARED
  src/PolyhedraBuoyancyDrag.cc
  src/PolyhedronVolume.cc
  src/ShapeVolume.cc
)
target_link_libraries(PolyhedraBuoyancyDrag PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Waves
)
install(
  TARGETS PolyhedraBuoyancyDrag
  DESTINATION lib)

# Plugins
list(APPEND USV_DESCRIPTION_PLUGINS
  AcousticPingerPlugin
  BallShooterPlugin
  LightBuoyPlugin
  PublisherPlugin
  SimpleHydrodynamics
  Surface
  USVWind
  WaveVisual
)

foreach(PLUGIN ${USV_DESCRIPTION_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    Waves
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS ${USV_DESCRIPTION_PLUGINS}
  DESTINATION lib)

ament_python_install_package(
  usv_description
  PACKAGE_DIR src/usv_description
)

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
