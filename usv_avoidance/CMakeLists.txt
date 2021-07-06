cmake_minimum_required(VERSION 3.0.2)
project(usv_avoidance)

### Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    tf
    roscpp
    rospy 
    std_msgs 
    genmsg
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
	usv_perception
	visualization_msgs
)

find_package(
	Eigen3 REQUIRED
)

set(acados_include "${PROJECT_SOURCE_DIR}/include/acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/include/acados/lib")
#set(acados_include "/home/ivana/MPC_CollisionAvoidance/catkin_ws/src/nmpc_ca/acados/include")
#set(acados_lib "/home/ivana/MPC_CollisionAvoidance/catkin_ws/src/nmpc_ca/acados/lib")

set(full_model_guidance_ca1 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance_ca1/c_generated_code)
set(full_model_pf_ca ${PROJECT_SOURCE_DIR}/scripts/usv_pf_ca/c_generated_code)
set(full_model_pf ${PROJECT_SOURCE_DIR}/scripts/usv_pf/c_generated_code)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usv_avoidance
  CATKIN_DEPENDS rospy usv_perception
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	include
	${full_model_guidance_ca1}
	${full_model_pf_ca}
	${full_model_pf}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_guidance_ca1})
link_directories(${full_model_pf_ca})

# NODE
add_executable(nmpc_guidance_ca1 src/nmpc_guidance_ca1.cpp)
target_link_libraries(nmpc_guidance_ca1
	${full_model_guidance_ca1}/libacados_ocp_solver_usv_model_guidance_ca1.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(nmpc_guidance_ca1 ${catkin_EXPORTED_TARGETS})

# NODE
add_executable(nmpc_pf_ca src/nmpc_pf_ca.cpp)
target_link_libraries(nmpc_pf_ca
	${full_model_pf_ca}/libacados_ocp_solver_usv_model_pf_ca.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(nmpc_pf_ca ${catkin_EXPORTED_TARGETS})

# NODE
add_executable(nmpc_traj_plan src/nmpc_traj_plan.cpp)
target_link_libraries(nmpc_traj_plan
	${full_model_pf_ca}/libacados_ocp_solver_usv_model_pf_ca.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(nmpc_traj_plan ${catkin_EXPORTED_TARGETS})

# NODE
add_executable(nmpc_pf src/nmpc_pf.cpp)
target_link_libraries(nmpc_pf
	${full_model_pf}/libacados_ocp_solver_usv_model_pf.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(nmpc_pf ${catkin_EXPORTED_TARGETS})