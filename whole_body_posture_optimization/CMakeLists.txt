cmake_minimum_required(VERSION 3.16)
project(whole_body_posture_optimization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Optional heightmap-based pose prediction (dependencies not yet ported to ROS 2)
option(WITH_HEIGHTMAP_POSE_PREDICTION "Build with hector_heightmap_pose_prediction support" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hector_pose_prediction_interface REQUIRED)
find_package(sdf_contact_estimation REQUIRED)
find_package(whole_body_planning_plotting_utils REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(ad_kinematics REQUIRED)
find_package(ceres_nlopt_wrapper REQUIRED)
find_package(hector_stability_metrics REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Ceres REQUIRED)
find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)

if(WITH_HEIGHTMAP_POSE_PREDICTION)
  find_package(hector_heightmap_pose_prediction REQUIRED)
  find_package(hector_world_heightmap REQUIRED)
  find_package(hector_world_heightmap_ros REQUIRED)
  find_package(grid_map_msgs REQUIRED)
endif()

set(SOURCES
  src/whole_body_posture_optimization.cpp
  src/robot_model.cpp
  src/markers.cpp
  src/util/common.cpp
  src/util/parameter_utils.cpp
  src/arm/stability_cost.cpp
  src/arm/stability_cost_parameters.cpp
  src/arm/arm_optimization.cpp
  src/flipper/flipper_optimization.cpp
  src/flipper/flipper_multi_objective_cost.cpp
  src/flipper/flipper_multi_objective_cost_parameters.cpp
  src/optimization_parameters_base.cpp
  src/solvers/optimizer_base.cpp
  src/solvers/grid_search_optimizer.cpp
  src/util/worker_thread.cpp
  src/util/worker_thread_pool.cpp
  src/util/iterative_spline_parameterization.cpp
)

set(THIS_PACKAGE_DEPENDENCIES
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  hector_pose_prediction_interface
  sdf_contact_estimation
  whole_body_planning_plotting_utils
  urdf
  moveit_core
  moveit_ros_planning
  ad_kinematics
  ceres_nlopt_wrapper
  hector_stability_metrics
  voxblox
  voxblox_ros
  voxblox_msgs
  tf2_eigen
  pluginlib
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Ceres::ceres
  NLopt::nlopt
  Eigen3::Eigen
)

## Node executables
add_executable(${PROJECT_NAME}_node src/nodes/whole_body_posture_optimization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

add_executable(manual_control_node src/nodes/manual_control_node.cpp)
target_link_libraries(manual_control_node PRIVATE ${PROJECT_NAME})

add_executable(cost_function_logging_node src/nodes/cost_function_logging_node.cpp)
target_link_libraries(cost_function_logging_node PRIVATE ${PROJECT_NAME})

add_executable(com_visualisation_node src/nodes/com_visualisation_node.cpp)
target_link_libraries(com_visualisation_node PRIVATE ${PROJECT_NAME})

if(WITH_HEIGHTMAP_POSE_PREDICTION)
  foreach(target ${PROJECT_NAME}_node manual_control_node)
    target_compile_definitions(${target} PRIVATE WBPO_HAVE_HEIGHTMAP)
    ament_target_dependencies(${target} PRIVATE
      hector_heightmap_pose_prediction
      hector_world_heightmap
      hector_world_heightmap_ros
      grid_map_msgs
    )
  endforeach()
endif()

#############
## Install ##
#############

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "ros1_legacy" EXCLUDE
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node manual_control_node cost_function_logging_node com_visualisation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plan_goal_marker_node.py
  scripts/srdf_with_world_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES} Ceres NLopt Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_common.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_add_gtest(${PROJECT_NAME}_optimization_test test/test_optimization.cpp)
  target_link_libraries(${PROJECT_NAME}_optimization_test ${PROJECT_NAME})
  ament_add_gtest(${PROJECT_NAME}_markers_test test/test_markers.cpp)
  target_link_libraries(${PROJECT_NAME}_markers_test ${PROJECT_NAME})
  ament_add_gtest(${PROJECT_NAME}_publish_markers_test test/test_publish_markers.cpp)
  target_link_libraries(${PROJECT_NAME}_publish_markers_test ${PROJECT_NAME})
endif()

ament_package()
