cmake_minimum_required(VERSION 3.0.2)
project(whole_body_posture_optimization)

#set(CMAKE_BUILD_TYPE "RelWithDebInfo")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  hector_pose_prediction_interface
  sdf_contact_estimation
  rbdl
  urdf
  moveit_core
  moveit_ros_planning
  ad_kinematics
  move_base_lite_msgs
  ceres_nlopt_wrapper
  hector_stability_metrics
  ddynamic_reconfigure
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whole_body_posture_optimization
  CATKIN_DEPENDS nav_msgs hector_pose_prediction_interface sdf_contact_estimation rbdl urdf moveit_core moveit_ros_planning ad_kinematics move_base_lite_msgs ceres_nlopt_wrapper hector_stability_metrics ddynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/whole_body_posture_optimization.h
  include/${PROJECT_NAME}/robot_model.h
  include/${PROJECT_NAME}/common.h
  include/${PROJECT_NAME}/timing.h
  include/${PROJECT_NAME}/arm/stability_cost.h
  include/${PROJECT_NAME}/arm/stability_cost_parameters.h
  include/${PROJECT_NAME}/arm/arm_optimization.h
  include/${PROJECT_NAME}/arm/sdf_contact_constraint.h
  include/${PROJECT_NAME}/arm/self_contact_constraint.h
  include/${PROJECT_NAME}/flipper/flipper_optimization.h
  include/${PROJECT_NAME}/flipper/flipper_multi_objective_cost.h
  include/${PROJECT_NAME}/flipper/flipper_multi_objective_cost_parameters.h
#  include/${PROJECT_NAME}/combined/combined_optimization.h
#  include/${PROJECT_NAME}/combined/combined_stability_cost.h
#  include/${PROJECT_NAME}/combined/self_contact_constraint_no_gradient.h
#  include/${PROJECT_NAME}/combined/sdf_contact_constraint_no_gradient.h
  include/${PROJECT_NAME}/optimization_parameters_base.h)

set(SOURCES
  src/whole_body_posture_optimization.cpp
  src/robot_model.cpp
  src/common.cpp
  src/arm/stability_cost.cpp
  src/arm/stability_cost_parameters.cpp
  src/arm/arm_optimization.cpp
  src/flipper/flipper_optimization.cpp
  src/flipper/flipper_multi_objective_cost.cpp
  src/flipper/flipper_multi_objective_cost_parameters.cpp
  #  src/combined/combined_optimization.cpp
#  src/combined/combined_stability_cost.cpp
  src/optimization_parameters_base.cpp include/whole_body_posture_optimization/data_logging/data_log.cpp include/whole_body_posture_optimization/data_logging/data_log.h include/whole_body_posture_optimization/data_logging/data_table.cpp include/whole_body_posture_optimization/data_logging/data_table.h src/optimization_logging_node.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare an executable
add_executable(${PROJECT_NAME}_node src/whole_body_posture_optimization_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## Declare an executable
add_executable(manual_control_node src/manual_control_node.cpp)
add_dependencies(manual_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_control_node ${PROJECT_NAME})

## Declare an executable
add_executable(optimization_logging_node src/optimization_logging_node.cpp)
add_dependencies(optimization_logging_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(optimization_logging_node ${PROJECT_NAME})

## Declare an executable
add_executable(com_visualisation_node src/com_visualisation_node.cpp)
add_dependencies(com_visualisation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(com_visualisation_node ${PROJECT_NAME})

## Declare an executable
#add_executable(stability_benchmark_node src/stability_benchmark_node.cpp)
#add_dependencies(stability_benchmark_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(stability_benchmark_node ${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_whole_body_posture_optimization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
