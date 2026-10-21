#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <iomanip>
#include <filesystem>
#include <limits>

#include <pluginlib/class_loader.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include "whole_body_posture_optimization/util/iterative_spline_parameterization.h"
#include <moveit/robot_state/conversions.hpp>

#include <voxblox_msgs/msg/layer.hpp>
#include <voxblox_ros/conversions.h>
#include <voxblox/io/layer_io.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "whole_body_posture_optimization/util/timing.h"
#include "whole_body_posture_optimization/util/common.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"
#include "whole_body_posture_optimization/markers.h"

INIT_TIMING

namespace whole_body_posture_optimization {

namespace fs = std::filesystem;

static const rclcpp::Logger kLogger = rclcpp::get_logger("whole_body_posture_optimization");

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const rclcpp::Node::SharedPtr& node,
                                                           const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                           const sdf_contact_estimation::SdfModelPtr& sdf_model,
                                                           const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics)
  : node_(node), pose_predictor_(pose_predictor), sdf_model_(sdf_model), stability_limit_(0.0)
{
  param_prefix_ = param_utils::paramPrefix(*node_);
  robot_model_ = std::make_shared<RobotModel>(node_);
  loadParameters(node_);

  flipper_optimization_ = std::make_shared<FlipperOptimization>(node_->create_sub_node("flipper_optimization"), robot_model_, pose_predictor_, prediction_diagnostics);
  arm_optimization_ = std::make_shared<ArmOptimization>(node_->create_sub_node("arm_optimization"), robot_model_, sdf_model_);

  // Publishers
  trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("trajectory", rclcpp::QoS(10).transient_local());
  sparse_trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("sparse_trajectory", rclcpp::QoS(10).transient_local());
  sdf_pub_ = node_->create_publisher<voxblox_msgs::msg::Layer>("planning_sdf", rclcpp::QoS(10).transient_local());
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("input_path", rclcpp::QoS(10).transient_local());
  if (param_utils::param(*node_, param_prefix_ + "publish_markers", true)) {
    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(1).transient_local());
  }
}

bool WholeBodyPostureOptimization::loadParameters(const rclcpp::Node::SharedPtr& node)
{
  // Set ros verbosity level to debug
  bool debug = param_utils::param(*node, param_prefix_ + "posture_optimization_debug", false);
  if (debug) {
    auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_WARN_STREAM(node->get_logger(), "Failed to set logger level to debug.");
    }
  }

  std::map<std::string, double> default_joint_state_map = param_utils::getDoubleMap(*node, param_prefix_ + "default_joint_state");
  if (default_joint_state_map.empty()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "No default joint state found.");
    return false;
  }
  default_joint_state_ = std::make_shared<moveit::core::RobotState>(robot_model_->getMoveitRobotState());
  default_joint_state_->setVariablePositions(default_joint_state_map);
  default_joint_state_vec_ = getVariablePositions(default_joint_state_, robot_model_->getActiveJointNames());


  stability_limit_ = param_utils::param(*node, param_prefix_ + "stability_limit", 0.0);
  front_tip_support_area_limit_ = param_utils::param(*node, param_prefix_ + "front_tip_support_area_limit", 0.5);
  stepping_ = param_utils::param(*node, param_prefix_ + "stepping", false);
  stepping_start_ = param_utils::param(*node, param_prefix_ + "stepping_start", 0);
  sdf_save_folder_ = param_utils::param(*node, param_prefix_ + "sdf_save_folder", std::string(""));

  std::string optimization_type_str = param_utils::param(*node, param_prefix_ + "optimization_type", std::string("split"));
  if (optimization_type_str == "split") {
    optimization_type_ = SPLIT;
  } else if (optimization_type_str == "combined") {
    optimization_type_ = COMBINED;
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Unknown optimization type '" << optimization_type_str << "'.");
  }

  complete_robot_group_ = param_utils::param(*node, param_prefix_ + "complete_robot_group", std::string("complete_robot_group"));
  complete_robot_with_world_group_ = param_utils::param(*node, param_prefix_ + "complete_robot_with_world_group", std::string("complete_robot_group_with_world"));
  trajectory_planner_name_ = param_utils::param(*node, param_prefix_ + "trajectory_planner", std::string("ompl_interface/OMPLPlanner"));
  max_velocity_scaling_factor_ = param_utils::param(*node, param_prefix_ + "max_velocity_scaling_factor", 1.0);
  max_acceleration_scaling_factor_ = param_utils::param(*node, param_prefix_ + "max_acceleration_scaling_factor", 1.0);
  max_base_velocity_ = param_utils::param(*node, param_prefix_ + "max_base_velocity", 0.0);
  max_base_yaw_velocity_ = param_utils::param(*node, param_prefix_ + "max_base_yaw_velocity", 0.0);
  surface_columns_ = param_utils::param(*node, param_prefix_ + "surface_columns", std::vector<double>());
  if (surface_columns_.size() % 2 != 0) {
    throw std::invalid_argument("surface_columns must hold x, y pairs.");
  }
  max_step_up_ = param_utils::param(*node, param_prefix_ + "max_step_up", 0.3);
  max_step_down_ = param_utils::param(*node, param_prefix_ + "max_step_down", 0.5);

  return true;
}

std::shared_ptr<RobotModel> WholeBodyPostureOptimization::getRobotModel() const
{
  return robot_model_;
}

const std::vector<double> &WholeBodyPostureOptimization::getDefaultJointPositions()
{
  return default_joint_state_vec_;
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeOptimalTrajectory(
    const nav_msgs::msg::Path& path, const moveit::core::RobotStateConstPtr& start_state)
{
  auto result = findOptimalPostures(path, start_state);
  auto trajectory = computeTimedTrajectory(result);
  return trajectory;
}

std::vector<PostureOptimizationResult> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::msg::Path& path,
                                                                                         const moveit::core::RobotStateConstPtr& start_state)
{
  // Iterate over path poses and run optimization
  path_pub_->publish(path);
  std::vector<PostureOptimizationResult> results;
  if (path.poses.empty()) {
    RCLCPP_WARN_STREAM(kLogger, "Can't find optimal postures, path is empty.");
    return results;
  }
  PostureOptimizationResult last_result;
  bool first = true;
  int counter = 0;
  for (const geometry_msgs::msg::PoseStamped& pose_msg: path.poses) {
    RCLCPP_INFO_STREAM(kLogger, "Pose: " << counter);
    counter++;
    Eigen::Isometry3d pose;
    tf2::fromMsg(pose_msg.pose, pose);
    if (first && start_state) {
      // We received a start state. The first pose of the path is the current pose,
      // so the state does not need to be optimized
      last_result.result_state = std::make_shared<moveit::core::RobotState>(*start_state);
      results.push_back(last_result);
      first = false;
    } else {
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), last_result.result_state);
      results.push_back(last_result);
      if (!last_result.success) {
        RCLCPP_ERROR_STREAM(kLogger, "Posture optimization failed.");
        break;
      }

      if (stepping_ && stepping_start_ < (counter+1)) {
        std::cout << '\n' << "Press ENTER to continue...";
        do {
        } while (rclcpp::ok() && std::cin.get() != '\n');
      }
      if (!rclcpp::ok()) {
        break;
      }
    }
  }

  // Publish sparse path
  robot_trajectory::RobotTrajectory sparse_trajectory(robot_model_->getMoveitRobotModel(), complete_robot_with_world_group_);
  for (const auto& result: results) {
    sparse_trajectory.addSuffixWayPoint(result.result_state, 1.0);
  }
  publishTrajectoryOnPub(sparse_trajectory_pub_, sparse_trajectory);

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Isometry3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const moveit::core::RobotStateConstPtr& previous_state,
                                                                           const moveit::core::RobotStateConstPtr& last_solution)
{
  START_TIMING("findOptimalPostureSplit")
  RCLCPP_INFO_STREAM(kLogger, "**********************************************");
  RCLCPP_INFO_STREAM(kLogger, "Requested pose: " << poseToString(pose));
  RCLCPP_INFO_STREAM(kLogger, "**********************************************");

  PostureOptimizationResult result;
  result.requested_pose = pose;
  result.init_joint_angles = init_joint_angles;

  Eigen::Isometry3d estimated_pose = pose; // Estimated pose of the robot at the requested position
  if (previous_state) {
    // Use last estimated pose to update estimate, if available
    estimated_pose = sdf_contact_estimation::updatePoseFromLastResult(pose, previous_state->getJointTransform("world_virtual_joint"), true, false);
  }
  // A start below a thin plate sees free space and falls through an open stair to the floor, so the prediction
  // starts with each column on the highest surface under it
  bool all_columns_on_surface = !surface_columns_.empty();
  double lowest_surface = std::numeric_limits<double>::infinity();
  if (sdf_model_ && !sdf_model_->isEmpty() && !surface_columns_.empty()) {
    double raise = 0.0;
    for (size_t i = 0; i + 1 < surface_columns_.size(); i += 2) {
      const Eigen::Vector3d column = estimated_pose * Eigen::Vector3d(surface_columns_[i], surface_columns_[i + 1], 0.0);
      const double surface = highestSurface(*sdf_model_, column.x(), column.y(), column.z() + max_step_up_, column.z() - max_step_down_);
      if (std::isfinite(surface)) {
        raise = std::max(raise, surface - column.z());
        lowest_surface = std::min(lowest_surface, surface);
      } else {
        all_columns_on_surface = false;
      }
    }
    estimated_pose.translation().z() += raise;
  } else {
    all_columns_on_surface = false;
  }

  // Optimize flippers
  RCLCPP_INFO_STREAM(kLogger, "Estimated pose before optimization: " << poseToString(estimated_pose));
  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(estimated_pose, init_joint_angles, previous_state, last_solution);
  result.result_pose = flipper_result.pose;
  result.support_polygon = flipper_result.support_polygon;
  if (markersSubscribed()) {
    result.contact_information = flipper_result.contact_information;
  }
  result.result_joint_angles = flipper_result.joint_state;
  result.stability_flipper_opt = flipper_result.stability;
  result.flipper_cost = flipper_result.final_cost;

  // Optimize the arm
  if (flipper_result.support_polygon.contact_hull_points.size() > 2) {
    ArmOptimizationResult arm_result = arm_optimization_->optimize(init_joint_angles, flipper_result.support_polygon.contact_hull_points,
                                                                   flipper_result.pose, flipper_result.joint_state);
    result.result_joint_angles = arm_result.joint_angles;
    result.result_com = arm_result.center_of_mass;
    result.stability_arm_opt = arm_result.stability;
    result.arm_cost = arm_result.final_cost;
    // The solver returns an arbitrary posture when every evaluated one failed
    const bool feasible = !(flipper_result.final_cost >= std::numeric_limits<double>::max());
    // A settled pose clearly below the surfaces under all columns fell through the terrain
    constexpr double kFallMargin = 0.1;
    const bool fell_through = all_columns_on_surface && flipper_result.pose.translation().z() < lowest_surface - kFallMargin;
    result.success = feasible && !fell_through && result.stability_arm_opt > stability_limit_;
    RCLCPP_INFO_STREAM(kLogger, "Stability: " << result.stability_arm_opt);
    if (!feasible) {
      RCLCPP_WARN_STREAM(kLogger, "No feasible flipper posture found.");
      result.failure_reason = "infeasible";
    } else if (fell_through) {
      RCLCPP_WARN_STREAM(kLogger, "The predicted pose fell through the terrain.");
      result.failure_reason = "fell through";
    } else if (!result.success) {
      RCLCPP_WARN_STREAM(kLogger, "Robot unstable (" << result.stability_arm_opt << " < " << stability_limit_ << ").");
      result.failure_reason = "unstable";
    }
  } else {
    RCLCPP_ERROR_STREAM(kLogger, "No convex hull found.");
    result.success = false;
    result.failure_reason = "no posture";
  }

  // Save to full state
  robot_model_->updateRobotState(result.result_pose, result.result_joint_angles);
  result.result_state = std::make_shared<moveit::core::RobotState>(robot_model_->getMoveitRobotState());

  // Publish results
  robot_model_->publishRobotState();

  STOP_TIMING_AVG

  return result;
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result,
                                                                                       std::vector<size_t>& trajectory_start_indices)
{
  std::vector<Eigen::Isometry3d> trajectory_world_poses;
  robot_trajectory::RobotTrajectory trajectory_with_world = computeTrajectory(opt_result,trajectory_world_poses, trajectory_start_indices);
  return addTimeParameterization(trajectory_with_world, trajectory_world_poses);
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTimedTrajectory(const std::vector<PostureOptimizationResult> &opt_result)
{
  std::vector<size_t> trajectory_start_indices;
  return computeTimedTrajectory(opt_result, trajectory_start_indices);
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTrajectory(const std::vector<PostureOptimizationResult>& opt_result,
                                                                                  std::vector<Eigen::Isometry3d>& trajectory_world_poses,
                                                                                  std::vector<size_t>& trajectory_start_indices)
{
  return computeTrajectory(opt_result, Eigen::Vector2d::Zero(), trajectory_world_poses, trajectory_start_indices);
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTrajectory(const std::vector<PostureOptimizationResult>& opt_result,
                                                                                  const Eigen::Vector2d& odom_start,
                                                                                  std::vector<Eigen::Isometry3d>& trajectory_world_poses,
                                                                                  std::vector<size_t>& trajectory_start_indices)
{
  if (!opt_result.back().success) {
    RCLCPP_WARN_STREAM(kLogger, "Posture optimization failed, can't compute trajectory");
    robot_trajectory::RobotTrajectory empty(robot_model_->getMoveitRobotModel(), complete_robot_with_world_group_);
    return empty;
  }

  // Copy states to vector
  std::vector<moveit::core::RobotStatePtr> states;
  states.reserve(opt_result.size());
  for (const auto& result: opt_result) {
    states.push_back(result.result_state);
  }

  return computeTrajectory(states, odom_start, trajectory_world_poses, trajectory_start_indices);
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::computeTrajectory(
    const std::vector<moveit::core::RobotStatePtr>& states, const Eigen::Vector2d& odom_start,
    std::vector<Eigen::Isometry3d>& trajectory_world_poses, std::vector<size_t>& trajectory_start_indices)
{
  START_TIMING("Trajectory planning")
  robot_trajectory::RobotTrajectory trajectory_fake_odom(robot_model_->getMoveitRobotModel(), complete_robot_with_world_group_);
  if (states.size() < 2) {
    RCLCPP_WARN_STREAM(kLogger, "Can't compute trajectory, path is empty.");
    return trajectory_fake_odom;
  }

  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(complete_robot_group_);

  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_->getMoveitRobotModel());

  // "linear" interpolates the joints between the postures, enough without collisions between moving links
  const bool linear = trajectory_planner_name_ == "linear";
  // Load the trajectory planner (kept as member: the loader must outlive all
  // trajectories because their waypoints are allocated inside the plugin library)
  if (!linear && !planner_instance_) {
    planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>("moveit_core", "planning_interface::PlannerManager");
    std::vector<std::string> available_planners = planner_plugin_loader_->getDeclaredClasses();
    if (std::find(available_planners.begin(), available_planners.end(), trajectory_planner_name_) == available_planners.end()) {
      RCLCPP_ERROR_STREAM(kLogger, "Can't find moveit trajectory planner with the name '" << trajectory_planner_name_ << "'. Available planners are: " << sdf_contact_estimation::vectorToString(available_planners));
      if (!available_planners.empty()) {
        trajectory_planner_name_ = available_planners.front();
        RCLCPP_ERROR_STREAM(kLogger, "Using planner '" << trajectory_planner_name_ << "' instead.");
      } else {
        RCLCPP_ERROR_STREAM(kLogger, "No planners found.");
        return trajectory_fake_odom;
      }
    }
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(trajectory_planner_name_));
    if (!planner_instance_->initialize(robot_model_->getMoveitRobotModel(), node_, "")) {
      RCLCPP_FATAL_STREAM(kLogger, "Could not initialize planner instance");
      planner_instance_.reset();
      return trajectory_fake_odom;
    }
    RCLCPP_INFO_STREAM(kLogger, "Using planning interface '" << planner_instance_->getDescription() << "'");
  }

  // We use x/y position fields to track distance / rotation for time parameterization
  double total_distance = odom_start[0];
  double total_rotation = odom_start[1];
  for (unsigned int i = 0; i < states.size()-1; i++) {
    RCLCPP_INFO_STREAM(kLogger, "Planning trajectory from state " << i << " to " << i + 1);
    planning_interface::MotionPlanRequest req;
    req.group_name = complete_robot_group_;
    req.allowed_planning_time = 2.0;
    moveit_msgs::msg::WorkspaceParameters& default_wp = req.workspace_parameters;
    default_wp.min_corner.x = default_wp.min_corner.y = default_wp.min_corner.z = -1000;
    default_wp.max_corner.x = default_wp.max_corner.y = default_wp.max_corner.z = 1000;

    // Set start state
    planning_scene->setCurrentState(*states[i]);
    moveit::core::robotStateToRobotStateMsg(*states[i], req.start_state);

    // set goal state
    moveit_msgs::msg::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(*states[i+1], joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(goal_constraint);

    // compute trajectory from start to goal
    planning_interface::MotionPlanResponse res;
    if (linear) {
      res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_->getMoveitRobotModel(), complete_robot_group_);
      res.trajectory->addSuffixWayPoint(*states[i], 0.0);
      res.trajectory->addSuffixWayPoint(*states[i+1], 0.0);
    } else {
      planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene, req, res.error_code);
      if (!context) {
        RCLCPP_ERROR_STREAM(kLogger, "Failed to construct planning context. Error: " << res.error_code.val);
        return trajectory_fake_odom;
      }
      context->solve(res);
      if (res.error_code.val != res.error_code.SUCCESS)
      {
        RCLCPP_ERROR(kLogger, "Could not compute plan successfully");
        break;
      }
    }

    // The moveit trajectory planner can't handle/interpolate the virtual world joint
    // So we do it ourselves and add the world virtual joint to all states
    Eigen::Isometry3d start_pose = states[i]->getJointTransform("world_virtual_joint");
    Eigen::Isometry3d end_pose = states[i+1]->getJointTransform("world_virtual_joint");

    Eigen::Vector3d start_position = start_pose.translation();
    Eigen::Vector3d end_position = end_pose.translation();
    Eigen::Vector3d position_diff = end_position - start_position;
    Eigen::Quaterniond start_orientation(start_pose.linear());
    Eigen::Quaterniond end_orientation(end_pose.linear());

    std::vector<Eigen::Isometry3d> world_poses;
    world_poses.reserve(res.trajectory->getWayPointCount());
    for (unsigned int j = 0; j < res.trajectory->getWayPointCount()-1; j++) {
      double t = static_cast<double>(j) / (res.trajectory->getWayPointCount()-1);
      // Compute interpolated world pose
      Eigen::Quaterniond interpolated_orientation = start_orientation.slerp(t, end_orientation);
      Eigen::Isometry3d interpolated_pose(interpolated_orientation);
      Eigen::Vector3d interpolated_position = start_position + t * position_diff;
      interpolated_pose.translation() = interpolated_position;
      world_poses.push_back(interpolated_pose);

      // Compute accumulated distance in x and yaw to properly limit linear and rotational speed of the robot base
      double distance = (t * position_diff).norm();
      Eigen::Quaterniond rotation_diff = start_orientation.inverse() * interpolated_orientation;
      Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);
      double yaw_distance = euler_angles(0);

      Eigen::Isometry3d odom_fake_pose = Eigen::Isometry3d::Identity();
      odom_fake_pose.translation().x() = total_distance + distance; // TODO this does not account for driving direction change
      odom_fake_pose.translation().z() = total_rotation + yaw_distance; // We use translation z for rotation. I consider this a work-around, not a hack!
      updateMoveitRobotState(*res.trajectory->getWayPointPtr(j), odom_fake_pose);
    }
    total_distance += position_diff.norm();
    Eigen::Quaterniond rotation_diff = start_orientation.inverse() * end_orientation;
    Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);
    double yaw_distance = euler_angles(0);
    total_rotation += yaw_distance;

    // Save start index of this trajectory part
    trajectory_start_indices.push_back(trajectory_fake_odom.getWayPointCount());
    // Append trajectory parts to full trajectory
    if (i != states.size()-2) {
      // Append to full trajectory (minus the end point, which would be a duplicate of the next start)
      trajectory_fake_odom.append(*res.trajectory, 0, 0, res.trajectory->getWayPointCount()-1);
    } else {
      // Keep end point on last trajectory
      Eigen::Isometry3d odom_fake_pose = Eigen::Isometry3d::Identity();
      odom_fake_pose.translation().x() = total_distance;
      odom_fake_pose.translation().z() = total_rotation;
      updateMoveitRobotState(*res.trajectory->getLastWayPointPtr(), odom_fake_pose);
      world_poses.push_back(end_pose);
      trajectory_fake_odom.append(*res.trajectory, 0);
    }
    trajectory_world_poses.insert(trajectory_world_poses.end(), world_poses.begin(), world_poses.end());
  }
  trajectory_start_indices.push_back(trajectory_fake_odom.getWayPointCount() - 1); // push last index
  STOP_TIMING_AVG


  return trajectory_fake_odom;
}

robot_trajectory::RobotTrajectory WholeBodyPostureOptimization::addTimeParameterization(
    robot_trajectory::RobotTrajectory& trajectory_fake_odom,
    const std::vector<Eigen::Isometry3d>& trajectory_world_poses)
{
  if (trajectory_fake_odom.getWayPointCount() != trajectory_world_poses.size()) {
    throw std::invalid_argument("trajectory_fake_odom size (" + std::to_string(trajectory_fake_odom.getWayPointCount())
                                + ") does not match trajectory_world_poses size(" + std::to_string(trajectory_world_poses.size()) + ").");
  }

  // Planning group of output trajectory must not contain world, because otherwise moveit wants a controller for floating joints
  robot_trajectory::RobotTrajectory trajectory_out(robot_model_->getMoveitRobotModel(), complete_robot_group_);

  // Perform time parameterization
  trajectory_processing::IterativeSplineParameterization time_parameterization(false);
  // The fake odometry carries the driven distance in trans_x and the yaw in trans_z
  std::unordered_map<std::string, double> base_limits;
  if (max_base_velocity_ > 0.0) {
    base_limits["world_virtual_joint/trans_x"] = max_base_velocity_;
  }
  if (max_base_yaw_velocity_ > 0.0) {
    base_limits["world_virtual_joint/trans_z"] = max_base_yaw_velocity_;
  }
  if (!time_parameterization.computeTimeStamps(trajectory_fake_odom, max_velocity_scaling_factor_, max_acceleration_scaling_factor_, base_limits)) {
    RCLCPP_ERROR_STREAM(kLogger, "Time parameterization failed.");
    return trajectory_out;
  }

  // Perform a deep copy before overwriting the world poses
  robot_trajectory::RobotTrajectory deep_copy(trajectory_fake_odom, true);

  // Replace fake world-poses used for time-parameterization with actual world poses
  for (unsigned int i = 0; i < deep_copy.getWayPointCount(); ++i) {
    updateMoveitRobotState(*deep_copy.getWayPointPtr(i), trajectory_world_poses[i]);
  }

  RCLCPP_INFO_STREAM(kLogger, "Trajectory duration: " << deep_copy.getDuration() << " s.");

  trajectory_out.append(deep_copy, 0);

  return trajectory_out;
}

void WholeBodyPostureOptimization::publishTrajectory(const robot_trajectory::RobotTrajectory &robot_trajectory, const rclcpp::Time& start_time)
{
  publishTrajectoryOnPub(trajectory_pub_, robot_trajectory, start_time);
}

void WholeBodyPostureOptimization::publishMarkers(const nav_msgs::msg::Path& path,
                                                  const std::vector<PostureOptimizationResult>& results,
                                                  const sdf_contact_estimation::SurfaceBand& terrain,
                                                  const std::string& status, const Eigen::Vector3d& robot_position)
{
  if (!markersSubscribed()) {
    return;
  }
  const std::string& frame_id = robot_model_->getMoveitRobotModel()->getModelFrame();
  visualization_msgs::msg::MarkerArray markers = horizonMarkers(path, results, terrain, frame_id);
  if (!status.empty()) {
    const auto status_markers = statusMarkers(robot_position, status, frame_id);
    markers.markers.insert(markers.markers.end(), status_markers.begin(), status_markers.end());
  }
  markers_pub_->publish(markers);
}

void WholeBodyPostureOptimization::publishGoalEnd(const std::string& status, const Eigen::Vector3d& robot_position)
{
  if (!markers_pub_) {
    return;
  }
  markers_pub_->publish(goalEndMarkers(status, robot_position, robot_model_->getMoveitRobotModel()->getModelFrame()));
}

void WholeBodyPostureOptimization::printResultInfo(const std::vector<PostureOptimizationResult> &results)
{
  if (results.empty()) {
    return;
  }
  std::vector<std::string> names = {"initial_stability", "flipper_opt", "arm_opt"};
  std::vector<std::vector<double>> stabilities(names.size());
  for (const PostureOptimizationResult& result: results) {
    stabilities[0].push_back(result.initial_stability);
    stabilities[1].push_back(result.stability_flipper_opt);
    stabilities[2].push_back(result.stability_arm_opt);
  }

  const int column_width_name = 20;
  const int column_width_avg = 10;
  const int column_width_min = 10;
  std::stringstream ss;
  ss << "STABILITY INFOS:" << std::endl;
  ss << "|" << std::setw(column_width_name) << "Name " << "|" << std::setw(column_width_avg) << "Avg" << "|" << std::setw(column_width_min) << "Min" << "|" << std::endl;
  ss << line(column_width_name + column_width_avg + column_width_min + 5) << std::endl;
  for (unsigned int i = 0; i < stabilities.size(); i++) {
    std::string name = names[i];
    const std::vector<double>& s = stabilities[i];
    double min = *std::min_element(begin(s), end(s));
    double avg = std::accumulate(begin(s), end(s), 0.0)/s.size();
    ss << "|" << std::setw(column_width_name) << name << "|" << std::setw(column_width_avg) << avg << "|" << std::setw(column_width_min) << min << "|" << std::endl;
  }
  RCLCPP_INFO_STREAM(kLogger, ss.str());
}


bool WholeBodyPostureOptimization::saveTrajectoryStatistics(const std::vector<PostureOptimizationResult>& results, robot_trajectory::RobotTrajectory& trajectory,
                                                            const std::vector<size_t>& trajectory_start_indices, const std::string& save_path, rclcpp::Time start_time)
{
  if (results.size() != trajectory_start_indices.size()) {
    RCLCPP_WARN_STREAM(kLogger, "Size of results does not match start indices.");
    return false;
  }

  fs::path path(save_path);
  fs::path parent_directory = path.parent_path();
  try {
    fs::create_directories(parent_directory);
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR_STREAM(kLogger, e.what());
    return false;
  }
  catch (const std::bad_alloc& e) {
    RCLCPP_ERROR_STREAM(kLogger, e.what());
    return false;
  }
  std::ofstream file;
  file.open(save_path);


  file << "time, requested_position_x, requested_position_y, result_position_x, result_position_y, flipper_cost, arm_cost, flipper_stability, stability" << std::endl;
  for (unsigned int i = 0; i < results.size(); i++) {
    const PostureOptimizationResult& result = results[i];
    rclcpp::Time time = start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(trajectory_start_indices[i]));
    file << rosTimeToString(time) << ", ";
    file << result.requested_pose.translation().x() << ", ";
    file << result.requested_pose.translation().y() << ", ";
    file << result.result_pose.translation().x() << ", ";
    file << result.result_pose.translation().y() << ", ";
    file << result.flipper_cost << ", ";
    file << result.arm_cost << ", ";
    file << result.stability_flipper_opt << ", ";
    file << result.stability_arm_opt << std::endl;
  }

  file.close();
  return true;
}

void WholeBodyPostureOptimization::publishTrajectoryOnPub(const rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr& pub,
                                                          const robot_trajectory::RobotTrajectory& robot_trajectory,
                                                          const rclcpp::Time& start_time)
{
  if (robot_trajectory.empty()) {
    return;
  }
  robot_trajectory::RobotTrajectory trajectory_with_world(robot_model_->getMoveitRobotModel(), complete_robot_with_world_group_);
  trajectory_with_world.append(robot_trajectory, 0);

  moveit_msgs::msg::DisplayTrajectory display_msg;
  moveit::core::robotStateToRobotStateMsg(trajectory_with_world.getFirstWayPoint(), display_msg.trajectory_start);
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory_with_world.getRobotTrajectoryMsg(trajectory_msg);
  trajectory_msg.joint_trajectory.header.stamp = start_time;
  display_msg.trajectory.push_back(trajectory_msg);

  pub->publish(display_msg);
}

}
