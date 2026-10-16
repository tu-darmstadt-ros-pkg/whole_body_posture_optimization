#include <whole_body_planning_server/whole_body_planning_server.h>

#include <whole_body_planning_server/path_smoother.h>
#include <whole_body_planning_server/util.h>
#include <whole_body_planning_server/terrain_fill.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_posture_optimization/markers.h>
#include <whole_body_posture_optimization/util/parameter_utils.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/conversions.hpp>

namespace whole_body_planning_server {

namespace param_utils = whole_body_posture_optimization::param_utils;

WholeBodyPlanningServer::WholeBodyPlanningServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
  path_updated_(false),
  end_of_path_reached_(false),
  execution_completed_(true),
  map_update_running_(true),
  enabled_(true),
  auto_start_(false),
  planning_horizon_(-1),
  minimum_path_length_(0),
  path_sampling_resolution_(0.02),
  path_stitching_tolerance_(0.15),
  override_path_time_(false),
  continuous_planning_(false),
  execution_position_tolerance_(0.0),
  execution_orientation_tolerance_(0.0),
  reverse_allowed_(true),
  ignore_goal_orientation_(false)
{
  trajectory_start_time_ = rclcpp::Time(0, 0u, RCL_ROS_TIME);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  loadParameters();

  // Publishers
  preprocessed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("preprocessed_path", rclcpp::QoS(10).transient_local());
  goal_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("goal_path", rclcpp::QoS(10).transient_local());
  robot_state_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("execution/goal_state", 10);
  position_deviation_pub_ = node_->create_publisher<std_msgs::msg::Float64>("execution/position_deviation", 10);
  orientation_deviation_pub_ = node_->create_publisher<std_msgs::msg::Float64>("execution/orientation_deviation", 10);

  // Create SDF Model
  // The esdf server runs on its own node which is spun by the map update thread
  // (replaces the ROS 1 custom callback queue)
  esdf_node_ = std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_esdf_server", node_->get_namespace());
  esdf_server_ = std::make_shared<voxblox::EsdfServer>(esdf_node_);
  sdf_model_ = std::make_shared<sdf_contact_estimation::SdfModel>(node_);
  sdf_model_->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), true);

  map_update_thread = std::make_unique<std::thread>(&WholeBodyPlanningServer::spinMapUpdate, this);


  // Create pose predictor
  // ShapeModel and SDFContactEstimation read their parameters from the node root
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(node_);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(node_, shape_model, sdf_model_);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  optimizer_ = std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(node_, pose_predictor, sdf_model_);
  current_robot_state_ = std::make_shared<moveit::core::RobotState>(optimizer_->getRobotModel()->getMoveitRobotState());
  if (optimizer_->markersEnabled()) {
    std::vector<Eigen::Vector3d> main_track_points;
    for (const auto& shape : shape_model->getShape()) {
      if (shape->isMainTrack()) {
        main_track_points.insert(main_track_points.end(), shape->getSamplingPoints().begin(), shape->getSamplingPoints().end());
      }
    }
    track_offsets_ = sdf_contact_estimation::trackOffsets(main_track_points, 3);
  }

  joint_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, param_utils::param(*node_, "joint_trajectory_action", std::string("flipper_trajectory_controller/follow_joint_trajectory")));

  // Connect vehicle controller action client
  RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for vehicle controller client..");
  driving_client_ = rclcpp_action::create_client<FollowPath>(node_, "/controller/follow_path");
  if (!driving_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Timed out waiting for driving action server.");
  }

  // Create FollowPathServer
  follow_path_server_ = rclcpp_action::create_server<FollowPath>(
      node_, "follow_path",
      std::bind(&WholeBodyPlanningServer::followPathGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WholeBodyPlanningServer::followPathCancelCallback, this, std::placeholders::_1),
      std::bind(&WholeBodyPlanningServer::followPathAcceptedCallback, this, std::placeholders::_1));

  // Start subscribers
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>("simple_path_goal", 10,
      std::bind(&WholeBodyPlanningServer::pathCb, this, std::placeholders::_1));
  execute_sub_ = node_->create_subscription<std_msgs::msg::Empty>("execute_planned_trajectory", 10,
      std::bind(&WholeBodyPlanningServer::confirmExecutionCb, this, std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      std::bind(&WholeBodyPlanningServer::jointStateCb, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(node_->get_logger(), "Initialization finished");
}

WholeBodyPlanningServer::~WholeBodyPlanningServer()
{
  stopExecutionThread();
  stopMapUpdateThread();
  if (goal_thread_ && goal_thread_->joinable()) {
    goal_thread_->join();
  }
}

void WholeBodyPlanningServer::loadParameters()
{
  stability_info_save_folder_ = param_utils::param(*node_, "stability_info_save_folder", std::string(""));
  base_frame_ = param_utils::param(*node_, "base_frame", std::string("base_link"));
  override_path_time_ = param_utils::param(*node_, "override_path_time", false);
  execution_position_tolerance_ = param_utils::param(*node_, "execution_position_tolerance", 0.0);
  execution_orientation_tolerance_ = param_utils::param(*node_, "execution_orientation_tolerance", 0.0);
  trajectory_statistics_save_folder_ = param_utils::param(*node_, "trajectory_statistics_save_folder", std::string(""));

  // Path following options (formerly part of the move_base_lite FollowPath goal)
  reverse_allowed_ = param_utils::param(*node_, "reverse_allowed", true);
  ignore_goal_orientation_ = param_utils::param(*node_, "ignore_goal_orientation", false);

  // Dynamically reconfigurable parameters (former dynamic_reconfigure PlanningServer.cfg)
  enabled_ = param_utils::param(*node_, "enabled", true);
  auto_start_ = param_utils::param(*node_, "auto_start", true);
  continuous_planning_ = param_utils::param(*node_, "continuous_planning", false);
  terrain_fill_ = param_utils::param(*node_, "terrain_fill", false);
  retry_time_ = param_utils::param(*node_, "retry_time", 0.0);
  planning_horizon_ = param_utils::param(*node_, "planning_horizon", -1.0);
  minimum_path_length_ = param_utils::param(*node_, "minimum_path_length", 0.2);
  path_sampling_resolution_ = param_utils::param(*node_, "path_sampling_resolution", 0.05);
  path_stitching_tolerance_ = param_utils::param(*node_, "path_stitching_tolerance", 0.15);
  if (minimum_path_length_ < path_sampling_resolution_) {
    minimum_path_length_ = path_sampling_resolution_;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Minimum path length cannot be less than path resolution. Setting minimum path length to " << minimum_path_length_);
  }
  if (planning_horizon_ > 0 && minimum_path_length_ > planning_horizon_) {
    minimum_path_length_ = 0.9 * planning_horizon_;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Minimum path length cannot be greater than planning horizon. Setting minimum path length to " << minimum_path_length_);
  }

  parameter_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&WholeBodyPlanningServer::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult WholeBodyPlanningServer::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  for (const rclcpp::Parameter& parameter: parameters) {
    const std::string& name = parameter.get_name();
    if (name == "enabled") {
      enabled_ = parameter.as_bool();
    } else if (name == "auto_start") {
      auto_start_ = parameter.as_bool();
    } else if (name == "continuous_planning") {
      continuous_planning_ = parameter.as_bool();
    } else if (name == "planning_horizon") {
      planning_horizon_ = parameter.as_double();
    } else if (name == "minimum_path_length") {
      minimum_path_length_ = parameter.as_double();
    } else if (name == "path_sampling_resolution") {
      path_sampling_resolution_ = parameter.as_double();
    } else if (name == "path_stitching_tolerance") {
      path_stitching_tolerance_ = parameter.as_double();
    } else {
      continue;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Parameter '" << name << "' updated.");
  }
  if (minimum_path_length_ < path_sampling_resolution_) {
    minimum_path_length_ = path_sampling_resolution_;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Minimum path length cannot be less than path resolution. Setting minimum path length to " << minimum_path_length_);
  }
  if (planning_horizon_ > 0 && minimum_path_length_ > planning_horizon_) {
    minimum_path_length_ = 0.9 * planning_horizon_;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Minimum path length cannot be greater than planning horizon. Setting minimum path length to " << minimum_path_length_);
  }
  return result;
}

bool WholeBodyPlanningServer::loadTsdfFromPath(const std::string& tsdf_path)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Loading sdf from path '" << tsdf_path << "'.");
  if (!esdf_server_->loadMap(tsdf_path)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to load tsdf from file.");
    return false;
  }
  return true;
}

bool WholeBodyPlanningServer::planAndExecutePath(nav_msgs::msg::Path path_msg, bool replanning) {
  // Override path time with receive time
  if (override_path_time_ || rclcpp::Time(path_msg.header.stamp) == rclcpp::Time(0, 0u, node_->now().get_clock_type()))
    path_msg.header.stamp = node_->now();

  map_frame_ = path_msg.header.frame_id;

  // Preprocess path
  nav_msgs::msg::Path path_smoothed = preprocessPath(path_msg, path_sampling_resolution_);

  if (!continuous_planning_) {
    return planAndExecutePathFull(path_smoothed, replanning);
  } else {
    if (replanning) {
      if (updatePath(goal_path_, path_smoothed)) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Previous path has been updated");
        path_updated_ = true;
        goal_path_pub_->publish(goal_path_);
        return true;
      } else {
       RCLCPP_INFO_STREAM(node_->get_logger(), "New path diverges from previous path. Resetting plan.");
      }
    }
    stopExecutionThread();
    stopExecution();
    path_updated_ = false;
    execution_completed_ = false;
    goal_path_ = path_smoothed;
    goal_path_pub_->publish(goal_path_);
    execution_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::planAndExecutePathContinuous, this, path_smoothed, false);
    execution_watchdog_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::executionWatchDog, this);
    return true;
  }
}

void WholeBodyPlanningServer::abortActiveGoal(const std::string& error_msg, uint16_t error_code, const std::string& status)
{
  if (followPathServerIsActive()) {
    auto result = std::make_shared<FollowPath::Result>();
    result->error_code = error_code;
    result->error_msg = error_msg;
    endGoalMarkers(status);
    follow_path_goal_->abort(result);
  }
}

void WholeBodyPlanningServer::abortExecution(const std::string& error_msg, uint16_t error_code, const std::string& status)
{
  std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
  stopExecution();
  abortActiveGoal(error_msg, error_code, status);
}

void WholeBodyPlanningServer::publishGoalMarkers(const nav_msgs::msg::Path& path,
                                                 const std::vector<whole_body_posture_optimization::PostureOptimizationResult>& results,
                                                 const sdf_contact_estimation::SurfaceBand& terrain, const std::string& status)
{
  std::lock_guard<std::mutex> lock(follow_path_goal_mutex_);
  if (goal_markers_active_ && optimizer_->markersSubscribed()) {
    optimizer_->publishMarkers(path, results, terrain, status, status.empty() ? Eigen::Vector3d::Zero() : latestRobotPosition(path));
  }
}

sdf_contact_estimation::SurfaceBand WholeBodyPlanningServer::terrainSurface(
    const std::vector<whole_body_posture_optimization::PostureOptimizationResult>& results) const
{
  // [m] Row spacing, and the search range around the base height that covers the tracks below it
  constexpr double kSpacing = 0.05;
  constexpr double kRange = 0.5;
  std::vector<Eigen::Isometry3d> poses;
  for (const auto& result : results) {
    poses.push_back(result.result_pose);
  }
  return sdf_contact_estimation::surfaceBand(*sdf_model_, poses, track_offsets_, kSpacing, kRange);
}

void WholeBodyPlanningServer::endGoalMarkers(const std::string& status)
{
  goal_markers_active_ = false;
  if (status.empty() || !optimizer_->markersSubscribed()) {
    optimizer_->publishGoalEnd("", Eigen::Vector3d::Zero());
  } else {
    optimizer_->publishGoalEnd(status, latestRobotPosition(follow_path_goal_->get_goal()->path));
  }
}

Eigen::Vector3d WholeBodyPlanningServer::latestRobotPosition(const nav_msgs::msg::Path& path) const
{
  try {
    return tf2::transformToEigen(tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero)).translation();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Status marker at the path start: " << ex.what());
    Eigen::Vector3d start;
    tf2::fromMsg(path.poses.front().pose.position, start);
    return start;
  }
}

bool WholeBodyPlanningServer::planAndExecutePathFull(const nav_msgs::msg::Path& path_msg, bool replanning)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received new path, planning..");

  // Path pre-processing
  nav_msgs::msg::Path path_slice = slicePath(path_msg, planning_horizon_);
  end_of_path_reached_ = (path_slice.poses.size() == path_msg.poses.size());
  if (end_of_path_reached_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Slice reaches end of path");
  }

  // Plan trajectory
  auto start = std::chrono::steady_clock::now();
  if(!generatePlan(path_slice, planned_trajectory_)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning failed.");
    abortExecution("Planning failed.", FollowPath::Result::INVALID_PATH, "no plan");
    return false;
  }
  auto end = std::chrono::steady_clock::now();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Planning took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms.");


  rclcpp::Time start_time;
  if (replanning) {
    // Trajectory has to start here in time, because planning takes time and robot might not be at the start state anymore
    start_time = path_msg.header.stamp;
  } else {
    // If this is the first trajectory, future-date the starting time, so we do not miss the first waypoint
    start_time = node_->now() + rclcpp::Duration::from_seconds(0.1);
  }

  rclcpp::Duration trajectory_duration = rclcpp::Duration::from_seconds(planned_trajectory_->getDuration());
  rclcpp::Time trajectory_end = start_time + trajectory_duration;

  rclcpp::Time now = node_->now();
  if (now > trajectory_end) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Trajectory end time is in the past by " << (now - trajectory_end).seconds() <<
                    " s. Planning time takes too long.");
  }

  if (auto_start_) {
    if (!executeTrajectoryWithWatchdog(*planned_trajectory_, start_time)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Trajectory execution failed");
      abortExecution("Trajectory execution failed.", FollowPath::Result::UNKNOWN, "exec failed");
      return false;
    }
  }

  return true;
}

void WholeBodyPlanningServer::planAndExecutePathContinuous(const nav_msgs::msg::Path& path_msg, bool replanning)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Received new path, planning continuously..");
  waitForMap(path_msg);

  // Path pre-processing
  const int trajectory_minimum_size = 4; // limited by time parameterization
  const int trajectory_overlap = 2;
  if (static_cast<int>(path_msg.poses.size()) < trajectory_minimum_size) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Path too short for a whole-body trajectory. Driving directly to goal.");
    end_of_path_reached_ = true;
    executeBaseTrajectory(path_msg);
    return;
  }

  // Collect results
  std::vector<whole_body_posture_optimization::PostureOptimizationResult> results;
  robot_trajectory::RobotTrajectoryPtr trajectory_fake_odom;
  std::vector<Eigen::Isometry3d> trajectory_world_poses;
  std::vector<size_t> trajectory_start_indices;

  // Add start state
  whole_body_posture_optimization::PostureOptimizationResult start_result;
  {
    std::lock_guard<std::mutex> lock(current_robot_state_mutex_);
    updateRobotPose(path_msg);
    start_result.result_state = std::make_shared<moveit::core::RobotState>(*current_robot_state_);
  }
  results.push_back(start_result);

  rclcpp::Time start_time;
  bool first_execution = true;
  robot_trajectory::RobotTrajectoryPtr trajectory;
  int time_parameterization_count = 0; // Number of waypoints with time parameterization attached (index of the first waypoint without t/p)
  Eigen::Vector2d fake_odom_pose = Eigen::Vector2d::Zero();
  nav_msgs::msg::Path target_path = path_msg;
  // Markers of planned waypoints wait until their trajectory is sent, so they never delay it
  std::vector<whole_body_posture_optimization::PostureOptimizationResult> pending_window;
  sdf_contact_estimation::SurfaceBand pending_terrain;
  for (current_path_index_ = 0; current_path_index_ < static_cast<int>(target_path.poses.size()) -1; ++current_path_index_) {
    if (path_updated_) {
      std::lock_guard<std::mutex> lock(path_update_mutex_);
      path_updated_ = false;
      target_path = goal_path_;
    }

    nav_msgs::msg::Path path_slice = slicePath(target_path, current_path_index_, current_path_index_ + 2);

    waitUntilPlanningHorizonReached(path_slice.poses.back().pose.position);

    if (execution_completed_) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Execution thread stopped");
      return;
    }
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Waypoint " << current_path_index_);

    end_of_path_reached_ = current_path_index_ == static_cast<int>(target_path.poses.size()) - 2;

    // Compute optimized postures
    // An infeasible pose is retried while the robot drives the planned part (a newer map may make it
    // feasible), and for retry_time after that part ends
    std::vector<whole_body_posture_optimization::PostureOptimizationResult> optimization_results;
    const rclcpp::Time retry_start = node_->now();
    while (true) {
      std::vector<whole_body_posture_optimization::PostureOptimizationResult> window;
      sdf_contact_estimation::SurfaceBand terrain;
      {
        std::lock_guard<std::mutex> lock(esdf_update_mutex_);
        fillTerrain();
        optimization_results = optimizer_->findOptimalPostures(path_slice, results.back().result_state);
        if (!optimization_results.empty() && optimizer_->markersSubscribed()) {
          // The robot has driven past older waypoints, so the newest kMaxGhosts cover the planned window
          const std::size_t planned = std::min(results.size() - 1, whole_body_posture_optimization::kMaxGhosts - 1);
          window.assign(results.end() - planned, results.end());
          window.push_back(optimization_results.back());
          // Sampled before the map thread can change the SDF the solve just used
          terrain = terrainSurface(window);
        }
      }
      if (!execution_completed_ && !window.empty()) {
        if (window.back().success) {
          pending_window = std::move(window);
          pending_terrain = std::move(terrain);
        } else {
          publishGoalMarkers(target_path, window, terrain, whole_body_posture_optimization::failureReason(window.back()));
        }
      }
      if ((!optimization_results.empty() && optimization_results.back().success) || execution_completed_) {
        break;
      }
      const rclcpp::Time planned_end = trajectory ? start_time + rclcpp::Duration::from_seconds(trajectory->getDuration()) : retry_start;
      if (node_->now() > planned_end + rclcpp::Duration::from_seconds(retry_time_)) {
        break;
      }
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Pose optimization failed at waypoint " << current_path_index_ << ", retrying.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (execution_completed_) {
      return;
    }
    if (optimization_results.empty() || !optimization_results.back().success) {
      abortExecution("Pose optimization failed.", FollowPath::Result::INVALID_PATH,
                     optimization_results.empty() ? "no posture" : whole_body_posture_optimization::failureReason(optimization_results.back()));
      break;
    }
    results.push_back(optimization_results.back()); // Use only the second (and last) result, first one is the start state

    // Compute trajectories
    std::vector<Eigen::Isometry3d> trajectory_world_poses_tmp;
    std::vector<size_t> trajectory_start_indices_tmp;
    robot_trajectory::RobotTrajectory trajectory_fake_odom_part = optimizer_->computeTrajectory(optimization_results, fake_odom_pose, trajectory_world_poses_tmp, trajectory_start_indices_tmp);
    if (trajectory_fake_odom_part.empty()) {
      abortExecution("Trajectory planning failed.", FollowPath::Result::INVALID_PATH, "no trajectory");
      break;
    }
    // Extract current fake odom pose to initialize next trajectory
    fake_odom_pose[0] = trajectory_fake_odom_part.getWayPoint(trajectory_fake_odom_part.getWayPointCount()-1).getVariablePosition("world_virtual_joint/trans_x");
    fake_odom_pose[1] = trajectory_fake_odom_part.getWayPoint(trajectory_fake_odom_part.getWayPointCount()-1).getVariablePosition("world_virtual_joint/trans_z");

    // Append to full trajectory (minus the end point, which would be a duplicate of the next start)
    if (!trajectory_fake_odom) {
      trajectory_fake_odom = std::make_shared<robot_trajectory::RobotTrajectory>(
          trajectory_fake_odom_part.getRobotModel(), trajectory_fake_odom_part.getGroupName());
    }
    trajectory_start_indices.push_back(trajectory_fake_odom->getWayPointCount());
    if (!end_of_path_reached_) {
      trajectory_world_poses.insert(trajectory_world_poses.end(), trajectory_world_poses_tmp.begin(), trajectory_world_poses_tmp.end() - 1);
      trajectory_fake_odom->append(trajectory_fake_odom_part, 0.0, 0, trajectory_fake_odom_part.getWayPointCount() - 1);
    } else {
      trajectory_world_poses.insert(trajectory_world_poses.end(), trajectory_world_poses_tmp.begin(), trajectory_world_poses_tmp.end());
      trajectory_fake_odom->append(trajectory_fake_odom_part, 0.0);
      trajectory_start_indices.push_back(trajectory_fake_odom->getWayPointCount() - 1);
    }

    // Check if trajectory is long enough to perform time parameterization
    // and covers enough distance to be executed by the vehicle controller
    if (static_cast<int>(trajectory_fake_odom->getWayPointCount()) < trajectory_minimum_size) {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(reconfigure_mutex_);
      if (trajectory_fake_odom->getWayPoint(trajectory_fake_odom->getWayPointCount()-1).getVariablePosition("world_virtual_joint/trans_x") < minimum_path_length_ && !end_of_path_reached_) {
        continue;
      }
    }

    // Find index of slice start for time parameterization
    int time_parameterization_start_index = time_parameterization_count;
    time_parameterization_start_index = std::max(0, time_parameterization_start_index - trajectory_overlap);
    // Check against minimum trajectory length
    if (static_cast<int>(trajectory_fake_odom->getWayPointCount()) - time_parameterization_start_index < trajectory_minimum_size) {
      time_parameterization_start_index = static_cast<int>(trajectory_fake_odom->getWayPointCount()) - trajectory_minimum_size; // Cannot go below 0 because we check for that earlier
    }

    // Slice trajectory and world poses
    robot_trajectory::RobotTrajectory trajectory_slice(trajectory_fake_odom->getRobotModel(), trajectory_fake_odom->getGroupName());
    trajectory_slice.append(*trajectory_fake_odom, 0.0, time_parameterization_start_index);
    std::vector<Eigen::Isometry3d> trajectory_world_poses_slice(std::next(trajectory_world_poses.begin(), time_parameterization_start_index), trajectory_world_poses.end());
    // Enforce bounds in slice (position, velocity, acceleration)
    for (size_t i = 0; i < trajectory_slice.getWayPointCount(); ++i) {
      trajectory_slice.getWayPointPtr(i)->enforceBounds();
      enforceAccelerationBounds(trajectory_slice.getWayPointPtr(i));
    }
    // Compute time parameterization for the slice (outputs a deep copy)
    auto timed_trajectory_slice = optimizer_->addTimeParameterization(trajectory_slice, trajectory_world_poses_slice);
    if (timed_trajectory_slice.empty() || timed_trajectory_slice.getDuration() == 0.0) {
      abortExecution("Time parameterization failed.", FollowPath::Result::INVALID_PATH, "no timing");
      break;
    }
    // Mark all waypoints to have a time parameterization
    time_parameterization_count = static_cast<int>(trajectory_fake_odom->getWayPointCount());

    // Slice unchanged part from previous trajectory and append new part
    if (!trajectory) {
      trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(timed_trajectory_slice.getRobotModel(),
                                                                        timed_trajectory_slice.getGroup());
    }
    robot_trajectory::RobotTrajectory trajectory_copy(*trajectory); // shallow copy
    trajectory->clear(); // Unfortunately, we can not delete waypoints, so we have to rebuild the trajectory
    trajectory->append(trajectory_copy, 0.0, 0, time_parameterization_start_index); // old part
    trajectory->append(timed_trajectory_slice, trajectory_copy.getWayPointDurationFromPrevious(time_parameterization_start_index)); // new part

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Total trajectory duration: " << trajectory->getDuration() << " s");

    // Set start time once after the first trajectory part has been computed
    if (first_execution) {
      first_execution = false;
      // Check if there are controllers that can handle the execution before sending the first trajectory
      if (!joint_trajectory_client_->action_server_is_ready()) {
        abortExecution("No controllers found that could execute the trajectory.", FollowPath::Result::INVALID_CONTROLLER, "no controller");
        break;
      }
      if (replanning) {
        // Trajectory has to start here in time, because planning takes time and robot might not be at the start state anymore
        start_time = target_path.header.stamp;
      } else {
        // If this is the first trajectory, future-date the starting time, so we do not miss the first waypoint
        start_time = node_->now() + rclcpp::Duration::from_seconds(0.1);
      }
    }

    // Check if trajectory end time is in the past
    rclcpp::Duration trajectory_duration = rclcpp::Duration::from_seconds(trajectory->getDuration());
    rclcpp::Time trajectory_end = start_time + trajectory_duration;
    rclcpp::Time now = node_->now();
    if (now > trajectory_end) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Trajectory end time is in the past by " << (now - trajectory_end).seconds() <<
                      " s. Planning takes too long.");
      // Fail and restart with a new path
      abortExecution("Planning time takes too long", FollowPath::Result::CONTROLLER_TIMED_OUT, "too slow");
      break;
    }

    // Send trajectory
    saveTrajectoryStatistics(results, *trajectory, trajectory_start_indices, start_time);
    bool sent;
    {
      std::lock_guard<std::recursive_mutex> lock(trajectory_mutex_);
      if (execution_completed_) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Stopping execution thread before execution");
        return;
      }
      sent = executeTrajectory(*trajectory, start_time);
    }
    if (!sent) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Trajectory execution failed");
      abortExecution("Trajectory execution failed.", FollowPath::Result::NO_VALID_CONTROL, "exec failed");
      break;
    }
    // Outside trajectory_mutex_, since publishing takes the goal mutex that cancel holds first
    if (!pending_window.empty()) {
      publishGoalMarkers(target_path, pending_window, pending_terrain);
      pending_window.clear();
    }
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Execution thread completed");
}


nav_msgs::msg::Path WholeBodyPlanningServer::preprocessPath(const nav_msgs::msg::Path& path, double path_sampling_resolution) const
{
  nav_msgs::msg::Path path_copy = path;

  // Replace first pose with actual robot pose to account for driving direction and improved planning initialization
  geometry_msgs::msg::TransformStamped base_transform;
  if (getRobotPose(path.header.stamp, base_transform)) {
    path_copy.poses.front().pose.orientation = base_transform.transform.rotation;
    path_copy.poses.front().pose.position.x = base_transform.transform.translation.x;
    path_copy.poses.front().pose.position.y = base_transform.transform.translation.y;
    path_copy.poses.front().pose.position.z = base_transform.transform.translation.z;
  }

  // Re-sample and smooth path
  // Note: In ROS 1, these options were part of the move_base_lite FollowPath goal.
  // nav2_msgs/FollowPath has no comparable options, so they are parameters now.
  Pathsmoother3D path_smoother(reverse_allowed_, ignore_goal_orientation_);
  path_smoother.setSmoothedPathDiscretization(path_sampling_resolution);
  path_copy = path_smoother.smooth(path_copy, true);

  preprocessed_path_pub_->publish(path_copy);

  return path_copy;
}

bool WholeBodyPlanningServer::generatePlan(const nav_msgs::msg::Path& path_msg, robot_trajectory::RobotTrajectoryPtr& trajectory_out)
{
  std::lock_guard<std::mutex> lock(esdf_update_mutex_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Planning a new path");
  if (path_msg.poses.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Path is empty. Can't plan.");
    return true; // Executing an empty plan is always successful
  }
  sdf_model_->publishSdfMesh();

  // *** Compute plan ***
  updateRobotPose(path_msg);
  auto results = optimizer_->findOptimalPostures(path_msg, current_robot_state_);
  if (optimizer_->markersSubscribed()) {
    const std::vector<whole_body_posture_optimization::PostureOptimizationResult> optimized(results.begin() + 1, results.end());
    publishGoalMarkers(path_msg, optimized, terrainSurface(optimized));
  }
  std::vector<size_t> trajectory_start_indices;
  auto trajectory = optimizer_->computeTimedTrajectory(results, trajectory_start_indices);
  if (trajectory.empty() || trajectory.getDuration() == 0.0) {
    return false;
  }

  saveTrajectoryStatistics(results, trajectory, trajectory_start_indices,
                           rclcpp::Time(0, 0u, RCL_ROS_TIME));

  trajectory_out = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory);

  return true;
}


bool WholeBodyPlanningServer::executeTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Starting trajectory execution");
  std::lock_guard<std::recursive_mutex> lock(trajectory_mutex_);
  if (trajectory.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Trajectory is empty, execution is trivially successful.");
    return true;
  }
  trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory, true);
  trajectory_start_time_ = start_time;

  // Send goals simultaneously
  if (!executeJointTrajectory(trajectory, start_time)) {
    return false;
  }
  executeBaseTrajectory(trajectory, start_time);

  optimizer_->publishTrajectory(trajectory, start_time);

  return true;
}

bool WholeBodyPlanningServer::executeTrajectoryWithWatchdog(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time)
{
  if (!executeTrajectory(trajectory, start_time)) {
    return false;
  }
  // A stop ends the watchdog, so the next execution needs a new one
  if (execution_completed_ && execution_watchdog_thread_) {
    execution_watchdog_thread_->join();
    execution_watchdog_thread_.reset();
  }
  execution_completed_ = false;
  if (!execution_watchdog_thread_) {
    execution_watchdog_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::executionWatchDog, this);
  }
  return true;
}

rclcpp_action::GoalResponse WholeBodyPlanningServer::followPathGoalCallback(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const FollowPath::Goal> goal)
{
  if (goal->path.poses.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Rejecting empty path goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "WholeBodyPlanner received new goal to [" << goal->path.poses.back().pose.position.x << ", "
                                                         << goal->path.poses.back().pose.position.y << "]");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WholeBodyPlanningServer::followPathAcceptedCallback(const std::shared_ptr<FollowPathServerGoalHandle>& goal_handle)
{
  bool goal_preempted = false;
  {
    std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
    // Check if another goal exists
    if (followPathServerIsActive()) {
      abortActiveGoal("This goal has been preempted by a newer goal.", FollowPath::Result::NONE, "");
      goal_preempted = true;
    }
    follow_path_goal_ = goal_handle;
    goal_markers_active_ = true;
  }

  // Plan and execute in a separate thread to not block the executor
  if (goal_thread_ && goal_thread_->joinable()) {
    goal_thread_->join();
  }
  goal_thread_ = std::make_unique<std::thread>([this, goal_handle, goal_preempted]() {
    if (enabled_) {
      planAndExecutePath(goal_handle->get_goal()->path, goal_preempted);
    } else {
      end_of_path_reached_ = true;
      executeBaseTrajectory(goal_handle->get_goal()->path);
    }
  });
}

rclcpp_action::CancelResponse WholeBodyPlanningServer::followPathCancelCallback(const std::shared_ptr<FollowPathServerGoalHandle>& goal_handle)
{
  std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
  if (goal_handle == follow_path_goal_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal preempted, stopping vehicle");
    // Cancel vehicle controller and stop moveit execution
    execution_completed_ = true;
    stopExecution();
    endGoalMarkers("");
    // rclcpp moves the goal to CANCELING only after this callback returned
    cancel_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this, goal_handle]() {
      std::lock_guard<std::mutex> lock(follow_path_goal_mutex_);
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<FollowPath::Result>();
        result->error_msg = "Canceled";
        goal_handle->canceled(result);
      }
      if (!goal_handle->is_canceling()) {
        cancel_timer_->cancel();
      }
    });
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Received cancel for unknown goal");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool WholeBodyPlanningServer::executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time)
{
  if (!joint_trajectory_client_->action_server_is_ready()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Execution failed! No joint trajectory controller is serving the joint trajectory action.");
    return false;
  }
  // A replan resends the grown trajectory with the original start stamp; the controller replaces the running one
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  FollowJointTrajectory::Goal goal;
  goal.trajectory = robot_trajectory_msg.joint_trajectory;
  goal.trajectory.header.stamp = start_time;
  joint_trajectory_client_->async_send_goal(goal);
  return true;
}

void WholeBodyPlanningServer::executeBaseTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time)
{
  // Assemble driving plan
  nav_msgs::msg::Path path = extractPathFromTrajectory(trajectory, start_time);
  executeBaseTrajectory(path);
}

void WholeBodyPlanningServer::executeBaseTrajectory(const nav_msgs::msg::Path& path) {
  FollowPath::Goal follow_path_goal;
  follow_path_goal.path = path;
  // Note: The former move_base_lite follow_path_options (desired_speed, use_path_orientation,
  // is_fixed, goal tolerances, ...) have no equivalent in nav2_msgs/FollowPath.
  // These options have to be configured on the receiving controller instead.
  follow_path_goal.controller_id = "";
  follow_path_goal.goal_checker_id = "";

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const FollowPathClientGoalHandle::SharedPtr& goal_handle) {
    std::lock_guard<std::mutex> lock(driving_goal_mutex_);
    --unanswered_driving_goals_;
    if (goal_handle) {
      driving_goal_id_ = goal_handle->get_goal_id();
    }
  };
  send_goal_options.result_callback =
      std::bind(&WholeBodyPlanningServer::followPathClientDoneCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&WholeBodyPlanningServer::followPathClientFeedbackCb, this, std::placeholders::_1, std::placeholders::_2);
  {
    std::lock_guard<std::mutex> lock(driving_goal_mutex_);
    ++unanswered_driving_goals_;
  }
  driving_client_->async_send_goal(follow_path_goal, send_goal_options);
}


nav_msgs::msg::Path WholeBodyPlanningServer::extractPathFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory, const rclcpp::Time& start_time) const
{
  std::vector<Eigen::Isometry3d> poses;
  std::vector<double> times;
  for (unsigned int i = 0; i < trajectory.getWayPointCount(); ++i) {
    poses.push_back(trajectory.getWayPoint(i).getJointTransform("world_virtual_joint"));
    times.push_back(trajectory.getWayPointDurationFromStart(i));
  }
  return timedPath(poses, times, start_time, map_frame_);
}

void WholeBodyPlanningServer::pathCb(const nav_msgs::msg::Path::ConstSharedPtr &path_msg) {
  {
    std::lock_guard<std::mutex> lock(follow_path_goal_mutex_);
    goal_markers_active_ = true;
  }
  planAndExecutePathFull(*path_msg, false);
}

void WholeBodyPlanningServer::confirmExecutionCb(const std_msgs::msg::Empty::ConstSharedPtr&)
{
  if (!planned_trajectory_) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "No trajectory planned yet, cannot execute");
    return;
  }
  rclcpp::Time starting_time = node_->now() + rclcpp::Duration::from_seconds(1);
  if (!executeTrajectoryWithWatchdog(*planned_trajectory_, starting_time)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Trajectory execution failed");
  }
}

void WholeBodyPlanningServer::jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg)
{
  std::lock_guard<std::mutex> lock(current_robot_state_mutex_);
  current_robot_state_->setVariableValues(*joint_state_msg);
}
bool WholeBodyPlanningServer::followPathServerIsActive() const
{
  return follow_path_goal_ && follow_path_goal_->is_active();
}
void WholeBodyPlanningServer::followPathClientDoneCallback(const FollowPathClientGoalHandle::WrappedResult& result)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Follow path client finished.");
  {
    std::lock_guard<std::mutex> lock(driving_goal_mutex_);
    // A result arriving while a newer goal is unanswered belongs to a replaced goal
    if (unanswered_driving_goals_ > 0 || result.goal_id != driving_goal_id_) {
      return;
    }
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (end_of_path_reached_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Following path successful.");
      stopExecutionThread();
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        auto server_result = std::make_shared<FollowPath::Result>();
        server_result->error_msg = "successful";
        follow_path_goal_->succeed(server_result);
        endGoalMarkers("");
      }
    }
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Following path failed with result code " << static_cast<int>(result.code));
    stopExecutionThread();
    // Path following failed
    joint_trajectory_client_->async_cancel_all_goals();
    std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
    abortActiveGoal("Following path failed", result.result ? result.result->error_code : FollowPath::Result::UNKNOWN, "follow failed");
  }
}

void WholeBodyPlanningServer::followPathClientFeedbackCb(
    FollowPathClientGoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> &feedback) {
  // Relay feedback
  std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
  if (followPathServerIsActive()) {
    follow_path_goal_->publish_feedback(std::make_shared<FollowPath::Feedback>(*feedback));
  }
}

void WholeBodyPlanningServer::updateRobotPose(const nav_msgs::msg::Path& path)
{
  Eigen::Isometry3d robot_pose_eigen;
  // First pose is current pose of robot
  const geometry_msgs::msg::PoseStamped& robot_pose_msg = path.poses.front();
  tf2::fromMsg(robot_pose_msg.pose, robot_pose_eigen);
  whole_body_posture_optimization::updateMoveitRobotState(*current_robot_state_, robot_pose_eigen);
}

void WholeBodyPlanningServer::fillTerrain()
{
  if (!terrain_fill_) {
    return;
  }
  const double truncation = esdf_server_->getEsdfMaxDistance();
  geometry_msgs::msg::TransformStamped robot;
  if (!getRobotPose(rclcpp::Time(0, 0u, RCL_ROS_TIME), robot)) {
    sdf_model_->loadEsdf(esdf_server_->getEsdfMapPtr(), truncation, false);
    return;
  }
  auto filled = terrain_fill::fill(*esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr(), tf2::transformToEigen(robot), truncation);
  sdf_model_->loadEsdf(std::make_shared<voxblox::EsdfMap>(filled), truncation, false);
}

bool WholeBodyPlanningServer::getRobotPose(const rclcpp::Time& stamp, geometry_msgs::msg::TransformStamped& transform) const
{
  try {
    // Get world transform
    transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "%s",ex.what());
    return false;
  }
  return true;
}

void WholeBodyPlanningServer::stopExecutionThread()
{
  execution_completed_ = true;
  if (execution_thread_) {
    execution_thread_->join();
    execution_thread_.reset();
  }
  if (execution_watchdog_thread_) {
    execution_watchdog_thread_->join();
    execution_watchdog_thread_.reset();
  }
}

void WholeBodyPlanningServer::spinMapUpdate()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(esdf_node_);
  rclcpp::WallRate rate(10);
  while (rclcpp::ok() && map_update_running_) {
    {
      std::lock_guard<std::mutex> lock(esdf_update_mutex_);
      executor.spin_some();
    }
    first_esdf_update_condition_.notify_all();
    rate.sleep();
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Map update thread finished.");
}
void WholeBodyPlanningServer::stopMapUpdateThread()
{
  map_update_running_ = false;
  if (map_update_thread) {
    map_update_thread->join();
    map_update_thread.reset();
  }
}

void WholeBodyPlanningServer::stopExecution()
{
  std::lock_guard<std::recursive_mutex> trajectory_lock(trajectory_mutex_);
  execution_completed_ = true;
  trajectory_.reset();
  trajectory_start_time_ = rclcpp::Time(0, 0u, RCL_ROS_TIME);

  {
    std::lock_guard<std::mutex> lock(driving_goal_mutex_);
    driving_goal_id_ = rclcpp_action::GoalUUID{};
  }
  driving_client_->async_cancel_all_goals();
  joint_trajectory_client_->async_cancel_all_goals();
}

void WholeBodyPlanningServer::executionWatchDog()
{
  rclcpp::Rate rate(10.0);
  while (!execution_completed_ && rclcpp::ok())  {
    std::unique_lock<std::recursive_mutex> trajectory_lock(trajectory_mutex_);
    if (!trajectory_) {
      trajectory_lock.unlock();
      rate.sleep();
      continue;
    }

    // Get latest robot pose
    geometry_msgs::msg::TransformStamped current_robot_pose_stamped;
    getRobotPose(rclcpp::Time(0, 0u, RCL_ROS_TIME), current_robot_pose_stamped);
    Eigen::Isometry3d current_robot_pose;
    current_robot_pose = tf2::transformToEigen(current_robot_pose_stamped.transform);

    // Find expected robot pose at current time
    rclcpp::Time now(current_robot_pose_stamped.header.stamp);
    rclcpp::Duration duration_after_start = now - trajectory_start_time_;
    int before_index, after_index;
    double progress; // value between 0 and 1
    trajectory_->findWayPointIndicesForDurationAfterStart(duration_after_start.seconds(), before_index, after_index, progress);
    const moveit::core::RobotState& before_state = trajectory_->getWayPoint(before_index);
    const moveit::core::RobotState& after_state = trajectory_->getWayPoint(after_index);

    // Linear interpolation for position and orientation
    const Eigen::Isometry3d& before_pose = before_state.getJointTransform("world_virtual_joint");
    const Eigen::Isometry3d& after_pose = after_state.getJointTransform("world_virtual_joint");

    Eigen::Vector3d position_diff = after_pose.translation() - before_pose.translation();
    Eigen::Vector3d interpolated_position = before_pose.translation() + progress * position_diff;

    Eigen::Quaterniond before_orientation(before_pose.linear());
    Eigen::Quaterniond after_orientation(after_pose.linear());
    Eigen::Quaterniond interpolated_orientation = before_orientation.slerp(progress, after_orientation);

    Eigen::Isometry3d interpolated_pose(interpolated_orientation);
    interpolated_pose.translation() = interpolated_position;

    // Compute full interpolated state and publish for visualization
    moveit::core::RobotState interpolated_state(trajectory_->getRobotModel());
    before_state.interpolate(after_state, progress, interpolated_state);
    whole_body_posture_optimization::updateMoveitRobotState(interpolated_state, interpolated_pose);
    moveit_msgs::msg::DisplayRobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(interpolated_state, robot_state_msg.state);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0;
    color.g = 1.0;
    color.b = 0;
    for (const auto& link: trajectory_->getRobotModel()->getLinkModelNames()) {
      moveit_msgs::msg::ObjectColor object_color;
      object_color.color = color;
      object_color.id = link;
      robot_state_msg.highlight_links.push_back(object_color);
    }
    robot_state_pub_->publish(robot_state_msg);

    // Check position threshold
    double distance = (interpolated_pose.translation() - current_robot_pose.translation()).norm();
    bool position_out_of_limits = execution_position_tolerance_ > 0 && distance > execution_position_tolerance_;

    // Check orientation threshold
    Eigen::Quaterniond current_orientation(current_robot_pose.linear());
    double angular_distance = interpolated_orientation.angularDistance(current_orientation);
    bool orientation_out_of_limits = execution_orientation_tolerance_ > 0 && angular_distance > execution_orientation_tolerance_;

    bool stop_execution = false;
    if (position_out_of_limits) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Position deviation of " << distance << " exceeds tolerance of " << execution_position_tolerance_);
      stop_execution = true;
    }
    if (orientation_out_of_limits) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Orientation deviation of " << angular_distance << " exceeds tolerance of " << execution_orientation_tolerance_);
      stop_execution = true;
    }

    // Publish deviation
    std_msgs::msg::Float64 deviation_msg;
    deviation_msg.data = distance;
    position_deviation_pub_->publish(deviation_msg);
    deviation_msg.data = angular_distance;
    orientation_deviation_pub_->publish(deviation_msg);

    if (stop_execution) {
      // Stop execution and fail goal (with stuck)
      trajectory_lock.unlock();
      abortExecution("Deviation between planned robot pose and current pose exceeds threshold.", FollowPath::Result::FAILED_TO_MAKE_PROGRESS, "deviation");
      break;
    }

    trajectory_lock.unlock();
    rate.sleep();
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Watchdog thread finished");
}

void WholeBodyPlanningServer::waitForMap(const nav_msgs::msg::Path& path)
{
  std::unique_lock<std::mutex> esdf_update_lock(esdf_update_mutex_);
  if (sdf_model_->isEmpty()) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for first ESDF");
    publishGoalMarkers(path, {}, {}, "no map");
    while (!execution_completed_ && sdf_model_->isEmpty()) {
      first_esdf_update_condition_.wait(esdf_update_lock);
    }
  }
}

void WholeBodyPlanningServer::saveTrajectoryStatistics(
    const std::vector<whole_body_posture_optimization::PostureOptimizationResult> &results,
    robot_trajectory::RobotTrajectory &trajectory,
    const std::vector<size_t> &trajectory_start_indices,
    const rclcpp::Time &start_time) {
  if (!trajectory_statistics_save_folder_.empty() && end_of_path_reached_) {
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = trajectory_statistics_save_folder_ + "/trajectory_statistics-" + time_stamp + ".csv";
    if (whole_body_posture_optimization::WholeBodyPostureOptimization::saveTrajectoryStatistics(results, trajectory, trajectory_start_indices, save_path, start_time)) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Saved trajectory statistics in " << save_path);
    }
  }
}

void WholeBodyPlanningServer::waitUntilPlanningHorizonReached(const geometry_msgs::msg::Point& planning_point)
{
  bool too_far_away = false;
  rclcpp::Rate rate(10.0);
  std::unique_lock<std::mutex> reconfigure_lock(reconfigure_mutex_);
  if (planning_horizon_ > 0) {
    reconfigure_lock.unlock();
    do {
      too_far_away = false;
      // Get current pose
      geometry_msgs::msg::TransformStamped robot_transform;
      if (getRobotPose(rclcpp::Time(0, 0u, RCL_ROS_TIME), robot_transform)) {
        // Distance in x-y plane
        double distance = euclideanDistance2D<geometry_msgs::msg::Vector3, geometry_msgs::msg::Point>(robot_transform.transform.translation, planning_point);
        {
          std::lock_guard<std::mutex> lock(reconfigure_mutex_);
          too_far_away = distance > planning_horizon_;
        }
        if (too_far_away && !execution_completed_) {
          rate.sleep();
        }
      }
    } while (too_far_away && !execution_completed_);
  } else {
    reconfigure_lock.unlock();
  }
}


bool WholeBodyPlanningServer::updatePath(nav_msgs::msg::Path& old_path, const nav_msgs::msg::Path& new_path) const {
  std::lock_guard<std::mutex> lock(path_update_mutex_);

  // Get current robot position
  geometry_msgs::msg::TransformStamped robot_transform;
  getRobotPose(rclcpp::Time(0, 0u, RCL_ROS_TIME), robot_transform);

  // Create slices
  nav_msgs::msg::Path new_slice = slicePathDirectDistance(new_path, robot_transform.transform.translation, planning_horizon_, true, 0);
  if (new_slice.poses.empty()) {
    return true;
  }
  nav_msgs::msg::Path old_slice = slicePathDirectDistance(old_path, robot_transform.transform.translation, planning_horizon_, false, current_path_index_);

  // Check if slices connect well
  double slice_distance = euclideanDistance2D(old_slice.poses.back().pose.position, new_slice.poses.front().pose.position);
  bool slices_fit = slice_distance < path_stitching_tolerance_;
  if (slices_fit) {
    old_path = old_slice;
    appendPath(old_path, new_slice);
  }
  return slices_fit;
}

}
