cmake_minimum_required(VERSION 3.0.2)
project(whole_body_posture_optimization)

#set(CMAKE_BUILD_TYPE "RelWithDebInfo")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sdf_contact_estimation
  rbdl
  urdf
  moveit_core
  moveit_ros_planning
  moveit_experimental
  ad_kinematics
  actionlib
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whole_body_posture_optimization
  CATKIN_DEPENDS nav_msgs sdf_contact_estimation rbdl urdf moveit_core moveit_ros_planning moveit_experimental ad_kinematics actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(HEADERS
  include/${PROJECT_NAME}/whole_body_posture_optimization.h
  include/${PROJECT_NAME}/robot_model.h
  include/${PROJECT_NAME}/common.h
  include/${PROJECT_NAME}/driving_controller.h
  include/${PROJECT_NAME}/arm/stability_error.h
  include/${PROJECT_NAME}/arm/arm_optimization.h
  include/${PROJECT_NAME}/arm/sdf_contact_constraint.h
  include/${PROJECT_NAME}/arm/self_contact_constraint.h
  include/${PROJECT_NAME}/flipper/flipper_optimization.h
  include/${PROJECT_NAME}/flipper/convex_hull_area_error.h
  include/${PROJECT_NAME}/flipper/flipper_stability_cost.h
  include/${PROJECT_NAME}/combined/combined_optimization.h
  include/${PROJECT_NAME}/combined/combined_stability_cost.h
  include/${PROJECT_NAME}/combined/self_contact_constraint_no_gradient.h
  include/${PROJECT_NAME}/combined/sdf_contact_constraint_no_gradient.h
)

set(SOURCES
  src/whole_body_posture_optimization.cpp
  src/robot_model.cpp
  src/common.cpp
  src/driving_controller.cpp
  src/arm/stability_error.cpp
  src/arm/arm_optimization.cpp
  src/flipper/flipper_optimization.cpp
  src/flipper/convex_hull_area_error.cpp
  src/flipper/flipper_stability_cost.cpp
  src/combined/combined_optimization.cpp
  src/combined/combined_stability_cost.cpp
)

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
add_executable(com_visualisation_node src/com_visualisation_node.cpp)
add_dependencies(com_visualisation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(com_visualisation_node ${PROJECT_NAME})

## Declare an executable
add_executable(driving_controller_node src/driving_controller_node.cpp)
add_dependencies(driving_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(driving_controller_node ${PROJECT_NAME})

## Declare an executable
add_executable(stability_benchmark_node src/stability_benchmark_node.cpp)
add_dependencies(stability_benchmark_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stability_benchmark_node ${PROJECT_NAME})

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
