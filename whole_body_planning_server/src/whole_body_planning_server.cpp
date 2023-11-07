#include <whole_body_planning_server/whole_body_planning_server.h>

#include <chrono>
#include <thread>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_planning_server/path_smoother.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <std_msgs/Float64.h>


namespace whole_body_planning_server {

WholeBodyPlanningServer::WholeBodyPlanningServer(const ros::NodeHandle &nh, const ros::NodeHandle& pnh)
  : nh_(nh),
  pnh_(pnh),
  end_of_path_reached_(false),
  execution_completed_(true),
  map_update_running_(true),
  tf_listener_(tf_buffer_),
  enabled_(true),
  auto_start_(false),
  planning_horizon_(-1),
  minimum_path_length_(0),
  path_sampling_resolution_(0.02),
  override_path_time_(false),
  continuous_planning_(false),
  execution_position_tolerance_(0.0),
  execution_orientation_tolerance_(0.0)
{
  // Parameters
  pnh_.param<std::string>("stability_info_save_folder", stability_info_save_folder_, "");
//  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param("override_path_time", override_path_time_, true);
  pnh_.param("execution_position_tolerance", execution_position_tolerance_, 0.0);
  pnh_.param("execution_orientation_tolerance", execution_orientation_tolerance_, 0.0);
  pnh_.param("trajectory_statistics_save_folder", trajectory_statistics_save_folder_, std::string(""));

  // Publishers
  preprocessed_path_pub_ = pnh_.advertise<nav_msgs::Path>("preprocessed_path", 10, true);
  robot_state_pub_ = pnh_.advertise<moveit_msgs::DisplayRobotState>("execution/goal_state", 10, false);
  position_deviation_pub_ = pnh_.advertise<std_msgs::Float64>("execution/position_deviation", 10, false);
  orientation_deviation_pub_ = pnh_.advertise<std_msgs::Float64>("execution/orientation_deviation", 10, false);

  // Dynamic reconfigure
  reconfigure_server_.setCallback(std::bind(&WholeBodyPlanningServer::dynamicReconfigureCallback,
                                            this, std::placeholders::_1, std::placeholders::_2));

  // Create SDF Model
  ros::NodeHandle esdf_server_nh(nh);
  esdf_server_nh.setCallbackQueue(&esdf_server_queue_);
  ros::NodeHandle esdf_server_pnh(pnh, "esdf_server");
  esdf_server_pnh.setCallbackQueue(&esdf_server_queue_);
  esdf_server_ = std::make_shared<voxblox::EsdfServer>(esdf_server_nh, esdf_server_pnh);
  sdf_model_ = std::make_shared<sdf_contact_estimation::SdfModel>(pnh);
  sdf_model_->loadEsdf(esdf_server_->getEsdfMapPtr(), esdf_server_->getEsdfMaxDistance(), true);

  map_update_thread = std::make_unique<std::thread>(&WholeBodyPlanningServer::spinMapUpdate, this);


  // Create pose predictor
  // Create robot model
  ros::NodeHandle pose_predictor_nh(pnh, "pose_predictor");
  ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model_);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  optimizer_ = std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh, pose_predictor, sdf_model_);
  current_robot_state_ = std::make_shared<robot_state::RobotState>(optimizer_->getRobotModel()->getMoveitRobotState());

  // Create moveit planner instance
  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(pnh);

  // Connect vehicle controller action client
  ROS_INFO_STREAM("Waiting for vehicle controller client..");
  driving_client_ = std::make_shared<FollowPathClient>(nh_, "/controller/follow_path", true);
  if (!driving_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for driving action server.");
  }

  // Create FollowPathServer
  follow_path_server_.reset(new actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>(
      pnh, "follow_path",
      std::bind(&WholeBodyPlanningServer::followPathGoalCallback, this, std::placeholders::_1),
      std::bind(&WholeBodyPlanningServer::followPathPreemptCallback, this, std::placeholders::_1),
      false));

  follow_path_server_->start();

  // Start subscribers
  path_sub_ = pnh_.subscribe("simple_path_goal", 10, &WholeBodyPlanningServer::pathCb, this);
  execute_sub_ = pnh_.subscribe("execute_planned_trajectory", 10, &WholeBodyPlanningServer::confirmExecutionCb, this);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &WholeBodyPlanningServer::jointStateCb, this);
  ROS_INFO_STREAM("Initialization finished");
}

WholeBodyPlanningServer::~WholeBodyPlanningServer()
{
  stopExecutionThread();
  stopMapUpdateThread();
}

bool WholeBodyPlanningServer::loadTsdfFromPath(const std::string& tsdf_path)
{
  ROS_INFO_STREAM("Loading sdf from path '" << tsdf_path << "'.");
  if (!esdf_server_->loadMap(tsdf_path)) {
    ROS_ERROR_STREAM("Failed to load tsdf from file.");
    return false;
  }
  return true;
}

bool WholeBodyPlanningServer::planAndExecutePath(nav_msgs::Path path_msg, bool replanning) {
  // Override path time with receive time
  if (override_path_time_)
    path_msg.header.stamp = ros::Time::now();

  map_frame_ = path_msg.header.frame_id;

  // Preprocess path
  nav_msgs::Path path_smoothed = preprocessPath(path_msg, path_sampling_resolution_);

  if (!continuous_planning_) {
    return planAndExecutePathFull(path_smoothed, replanning);
  } else {
    execution_completed_ = false;
    execution_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::planAndExecutePathContinuous, this, path_smoothed, replanning);
    execution_watchdog_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::executionWatchDog, this);
    return true;
  }
}

bool WholeBodyPlanningServer::planAndExecutePathFull(const nav_msgs::Path& path_msg, bool replanning)
{
  ROS_INFO_STREAM("Received new path, planning..");

  // Path pre-processing
  nav_msgs::Path path_slice = slicePath(path_msg, planning_horizon_);
  end_of_path_reached_ = (path_slice.poses.size() == path_msg.poses.size());
  if (end_of_path_reached_) {
    ROS_INFO_STREAM("Slice reaches end of path");
  }

  // Plan trajectory
  auto start = std::chrono::steady_clock::now();
  if(!generatePlan(path_slice, planned_trajectory_)) {
    ROS_ERROR_STREAM("Planning failed.");
    if (followPathServerIsActive()) {
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
      follow_path_goal_.setAborted(result, "Planning failed.");
    }

    return false;
  }
  auto end = std::chrono::steady_clock::now();
  ROS_INFO_STREAM("Planning took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms.");


  ros::Time start_time;
  if (replanning) {
    // Trajectory has to start here in time, because planning takes time and robot might not be at the start state anymore
    start_time = path_msg.header.stamp;
  } else {
    // If this is the first trajectory, future-date the starting time, so we do not miss the first waypoint
    start_time = ros::Time::now() + ros::Duration(0.1);
  }

  ros::Duration trajectory_duration(planned_trajectory_->getDuration());
  ros::Time trajectory_end = start_time + trajectory_duration;

  ros::Time now = ros::Time::now();
  if (now > trajectory_end) {
    ROS_WARN_STREAM("Trajectory end time is in the past by " << (now - trajectory_end).toSec() <<
                    " s. Planning time takes too long.");
  }

  if (auto_start_) {
    if (!executeTrajectoryWithWatchdog(*planned_trajectory_, start_time)) {
      ROS_ERROR_STREAM("Trajectory execution failed");
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::FAILURE;
        follow_path_goal_.setAborted(result, "Trajectory execution failed.");
      }
      return false;
    }
  }

  return true;
}

void WholeBodyPlanningServer::planAndExecutePathContinuous(const nav_msgs::Path& path_msg, bool replanning)
{
  ROS_DEBUG_STREAM("Received new path, planning continuously..");
  waitForMap();

  // Path pre-processing
  const int trajectory_minimum_size = 4; // limited by time parameterization
  const int trajectory_overlap = 2;
  if (path_msg.poses.size() < trajectory_minimum_size) {
    ROS_WARN_STREAM("Path too short for a whole-body trajectory. Driving directly to goal.");
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
    start_result.result_state = std::make_shared<robot_state::RobotState>(*current_robot_state_);
  }
  results.push_back(start_result);

  ros::Time start_time;
  bool first_execution = true;
  robot_trajectory::RobotTrajectoryPtr trajectory;
  int time_parameterization_count = 0; // Number of waypoints with time parameterization attached (index of the first waypoint without t/p)
  Eigen::Vector2d fake_odom_pose = Eigen::Vector2d::Zero();
  for (int current_traj_index = 0; current_traj_index < path_msg.poses.size() -1; ++current_traj_index) {
    nav_msgs::Path path_slice = slicePath(path_msg, current_traj_index, current_traj_index + 2);

    waitUntilPlanningHorizonReached(path_slice.poses.back().pose.position);

    if (execution_completed_) {
      ROS_DEBUG_STREAM("Execution thread stopped");
      return;
    }
    ROS_DEBUG_STREAM("Waypoint " << current_traj_index);

    end_of_path_reached_ = current_traj_index == path_msg.poses.size() - 2;

    // Compute optimized postures
    std::vector<whole_body_posture_optimization::PostureOptimizationResult> optimization_results;
    {
      std::lock_guard<std::mutex> lock(esdf_update_mutex_);
      optimization_results = optimizer_->findOptimalPostures(path_slice, results.back().result_state);
    }
    if (optimization_results.empty() || !optimization_results.back().success) {
      stopExecution();
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        follow_path_goal_.setAborted(result, "Pose optimization failed.");
      }
      break;
    }
    results.push_back(optimization_results.back()); // Use only the second (and last) result, first one is the start state

    // Compute trajectories
    std::vector<Eigen::Isometry3d> trajectory_world_poses_tmp;
    std::vector<size_t> trajectory_start_indices_tmp;
    robot_trajectory::RobotTrajectory trajectory_fake_odom_part = optimizer_->computeTrajectory(optimization_results, fake_odom_pose, trajectory_world_poses_tmp, trajectory_start_indices_tmp);
    if (trajectory_fake_odom_part.empty()) {
      stopExecution();
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        follow_path_goal_.setAborted(result, "Trajectory planning failed.");
      }
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
    if (trajectory_fake_odom->getWayPointCount() < trajectory_minimum_size) {
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
    if (trajectory_fake_odom->getWayPointCount() - time_parameterization_start_index < trajectory_minimum_size) {
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
      stopExecution();
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        follow_path_goal_.setAborted(result, "Time parameterization failed.");
      }
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

    ROS_DEBUG_STREAM("Total trajectory duration: " << trajectory->getDuration() << " s");

    // Set start time once after the first trajectory part has been computed
    if (first_execution) {
      first_execution = false;
      if (replanning) {
        // Trajectory has to start here in time, because planning takes time and robot might not be at the start state anymore
        start_time = path_msg.header.stamp;
      } else {
        // If this is the first trajectory, future-date the starting time, so we do not miss the first waypoint
        start_time = ros::Time::now() + ros::Duration(0.1);
      }
    }

    // Check if trajectory end time is in the past
    ros::Duration trajectory_duration(trajectory->getDuration());
    ros::Time trajectory_end = start_time + trajectory_duration;
    ros::Time now = ros::Time::now();
    if (now > trajectory_end) {
      ROS_ERROR_STREAM("Trajectory end time is in the past by " << (now - trajectory_end).toSec() <<
                      " s. Planning time takes too long.");
      // Fail and restart with a new path
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        follow_path_goal_.setAborted(result, "Planning time takes too long");
      }
      break;
    } else {
      saveTrajectoryStatistics(results, *trajectory, trajectory_start_indices);
      // Send trajectory
      std::lock_guard<std::recursive_mutex> lock(trajectory_mutex_);
      if (execution_completed_) {
        ROS_DEBUG_STREAM("Stopping execution thread before execution");
        return;
      }
      if (!executeTrajectory(*trajectory, start_time)) {
        ROS_ERROR_STREAM("Trajectory execution failed");
        stopExecution();
        std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
        if (followPathServerIsActive()) {
          move_base_lite_msgs::FollowPathResult result;
          result.result.val = move_base_lite_msgs::ErrorCodes::CONTROL_FAILED;
          follow_path_goal_.setAborted(result, "Trajectory execution failed.");
        }
        break;
      }
    }
  }
  ROS_DEBUG_STREAM("Execution thread completed");
}


nav_msgs::Path WholeBodyPlanningServer::preprocessPath(const nav_msgs::Path& path, double path_sampling_resolution) const
{
  nav_msgs::Path path_copy = path;

  // Replace first pose with current pose to account for driving direction and better planning initialization
  geometry_msgs::TransformStamped base_transform;
  if (getRobotPose(path.header.stamp, base_transform)) {
    path_copy.poses.front().pose.orientation = base_transform.transform.rotation;
    path_copy.poses.front().pose.position.x = base_transform.transform.translation.x;
    path_copy.poses.front().pose.position.y = base_transform.transform.translation.y;
    path_copy.poses.front().pose.position.z = base_transform.transform.translation.z;
  }

  // Re-sample and smooth path
  bool allow_reverse_paths = true;
  bool ignore_goal_orientation = false;

  if (followPathServerIsActive()) {
    allow_reverse_paths = follow_path_goal_.getGoal()->follow_path_options.reverse_allowed;
    ignore_goal_orientation = follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance > 3.1415; // TODO ugly
  }

  Pathsmoother3D path_smoother(allow_reverse_paths, ignore_goal_orientation);
  path_smoother.setSmoothedPathDiscretization(path_sampling_resolution);
  path_copy = path_smoother.smooth(path_copy, true);

  preprocessed_path_pub_.publish(path_copy);

  return path_copy;
}

nav_msgs::Path WholeBodyPlanningServer::slicePath(const nav_msgs::Path& path, double distance)
{
  // Special case: always return original path if distance is <=0
  if (distance <= 0) {
    return path;
  }
  // Sum up distance and add poses
  double current_distance = 0.0;
  nav_msgs::Path path_slice;
  path_slice.header = path.header;
  for (const auto& pose: path.poses) {
    if (path_slice.poses.empty()) {
      path_slice.poses.push_back(pose);
    } else {
      Eigen::Vector3d p1, p2;
      tf::pointMsgToEigen(path_slice.poses.back().pose.position, p1);
      tf::pointMsgToEigen(pose.pose.position, p2);
      double d = (p2 - p1).norm();
      current_distance += d;

      if (current_distance > distance) {
        break;
      }
      path_slice.poses.push_back(pose);
    }
  }
  return path_slice;
}

nav_msgs::Path WholeBodyPlanningServer::slicePath(const nav_msgs::Path& path, int start_index, int end_index)
{
  if (end_index < start_index) {
    throw std::invalid_argument("end_index (" + std::to_string(end_index) + ") is smaller than start_index (" + std::to_string(start_index) + ").");
  }
  nav_msgs::Path path_slice;
  path_slice.header = path.header;
  path_slice.poses.assign(path.poses.begin() + start_index, path.poses.begin() + end_index);
  return path_slice;
}

bool WholeBodyPlanningServer::generatePlan(const nav_msgs::Path& path_msg, robot_trajectory::RobotTrajectoryPtr& trajectory_out)
{
  std::lock_guard<std::mutex> lock(esdf_update_mutex_);
  ROS_INFO_STREAM("Planning a new path");
  if (path_msg.poses.empty()) {
    ROS_WARN_STREAM("Path is empty. Can't plan.");
    return true; // Executing an empty plan is always successful
  }
  sdf_model_->publishSdfMesh();

  // *** Compute plan ***
  updateRobotPose(path_msg);
//  auto trajectory = optimizer_->computeOptimalTrajectory(path_msg, current_robot_state_);
  auto results = optimizer_->findOptimalPostures(path_msg, current_robot_state_);
  std::vector<size_t> trajectory_start_indices;
  auto trajectory = optimizer_->computeTimedTrajectory(results, trajectory_start_indices);
  if (trajectory.empty() || trajectory.getDuration() == 0.0) {
    return false;
  }

  saveTrajectoryStatistics(results, trajectory, trajectory_start_indices);

  trajectory_out = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory);

  return true;
}

bool WholeBodyPlanningServer::executeTrajectory(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time)
{
  ROS_DEBUG_STREAM("Starting trajectory execution");
  if (trajectory.empty()) {
    ROS_WARN_STREAM("Trajectory is empty, execution is trivially successful.");
    return true;
  }
  trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory, true);
  trajectory_start_time_ = start_time;

  // Send goals simultaneously
  if (!executeJointTrajectory(trajectory, false, start_time)) {
    return false;
  }
  executeBaseTrajectory(trajectory, start_time);

  // Wait until start time to publish trajectory
  ros::Time now = ros::Time::now();
  if (start_time > now) {
    ros::Duration time_until_start = start_time - now;
    time_until_start.sleep();
  }
  optimizer_->publishTrajectory(trajectory, start_time);

  return true;
}

bool WholeBodyPlanningServer::executeTrajectoryWithWatchdog(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time)
{
  if (!executeTrajectory(trajectory, start_time)) {
    return false;
  }
  execution_completed_ = false;
  if (!execution_watchdog_thread_) {
    execution_watchdog_thread_ = std::make_unique<std::thread>(&WholeBodyPlanningServer::executionWatchDog, this);
  }
  return true;
}

void WholeBodyPlanningServer::followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal)
{
  ROS_INFO_STREAM("WholeBodyPlanner received new goal to [" << goal.getGoal()->target_path.poses.back().pose.position.x << ", "
                                                         << goal.getGoal()->target_path.poses.back().pose.position.y << "]");
  // Check if another goal exists
  bool goal_preempted = false;
  if (followPathServerIsActive()) {
    // Check if incoming one is newer
    if (goal.getGoalID().stamp >= follow_path_goal_.getGoalID().stamp) {
      // Cancel previous goal
      stopExecutionThread(); // Stop before goal is canceled
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      follow_path_goal_.setCanceled(result, "This goal has been preempted by a newer goal.");
      goal_preempted = true;
    } else {
      // Cancel incoming goal
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      goal.setCanceled(result, "This goal has been preempted by a newer goal.");
      return;
    }
  }

  // Accept new goal
  goal.setAccepted();
  follow_path_goal_ = goal;

  stopExecutionThread(); // Make sure any old thread is cleaned up


  // Plan and execute
  if (enabled_) {
    planAndExecutePath(goal.getGoal()->target_path, goal_preempted);
  } else {
    end_of_path_reached_ = true;
    driving_client_->sendGoal(*goal.getGoal(),
                              std::bind(&WholeBodyPlanningServer::followPathClientDoneCallback, this, std::placeholders::_1, std::placeholders::_2),
                              FollowPathClient::SimpleActiveCallback(),
                              FollowPathClient::SimpleFeedbackCallback());
  }
}
void WholeBodyPlanningServer::followPathPreemptCallback(const actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle& preempt)
{
  if (preempt == follow_path_goal_) {
    ROS_INFO_STREAM("Goal preempted, stopping vehicle");
    // Cancel vehicle controller and stop moveit execution
    stopExecutionThread();
    stopExecution();

    if (followPathServerIsActive()) {
      move_base_lite_msgs::FollowPathResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
      follow_path_goal_.setCanceled(result, "Preempt from incoming message to server");
    }
  } else {
    ROS_WARN_STREAM("Received preempt for unknown goal");
  }
}

bool WholeBodyPlanningServer::executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, bool blocking, ros::Time start_time)
{
  // Check if there are controllers that can handle the execution
  if (!moveit_cpp_ptr_->getTrajectoryExecutionManager()->ensureActiveControllersForGroup(trajectory.getGroupName()))
  {
    ROS_ERROR("Execution failed! No active controllers configured for group '%s'", trajectory.getGroupName().c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
  robot_trajectory_msg.joint_trajectory.header.stamp = start_time;
  if (blocking)
  {
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->push(robot_trajectory_msg);
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->execute();
    return true;
  }
  return moveit_cpp_ptr_->getTrajectoryExecutionManager()->pushAndExecute(robot_trajectory_msg);
}

void WholeBodyPlanningServer::executeBaseTrajectory(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time)
{
  // Assemble driving plan
  nav_msgs::Path path = extractPathFromTrajectory(trajectory, start_time);
  executeBaseTrajectory(path);
}

void WholeBodyPlanningServer::executeBaseTrajectory(const nav_msgs::Path& path) {
  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  // Fill parameters
  follow_path_goal.follow_path_options.desired_speed = 0.0; // Implicitly given by pose time stamps
  follow_path_goal.follow_path_options.use_path_orientation = true; // Drive path as planned
  follow_path_goal.follow_path_options.is_fixed = true; // No smoothing
  follow_path_goal.follow_path_options.reset_stuck_history = false;

  // Use values from goal if available
  if (followPathServerIsActive()) {
    follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = follow_path_goal_.getGoal()->follow_path_options.rotate_front_to_goal_pose_orientation;
    follow_path_goal.follow_path_options.reverse_allowed = follow_path_goal_.getGoal()->follow_path_options.reverse_allowed;
    follow_path_goal.follow_path_options.goal_pose_position_tolerance = follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance;
    follow_path_goal.follow_path_options.goal_pose_angle_tolerance = follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance;
  } else {
    follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
    follow_path_goal.follow_path_options.reverse_allowed = true;
    follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.1;
    follow_path_goal.follow_path_options.goal_pose_angle_tolerance = M_PI;
  }

  if (!end_of_path_reached_) {
    // Set low tolerances because the current end of the path is not the goal pose
    follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.0001;
    // Do not care about final orientation inside path
    follow_path_goal.follow_path_options.goal_pose_angle_tolerance = M_PI;
    follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = false;
  }

  driving_client_->sendGoal(follow_path_goal,
                            std::bind(&WholeBodyPlanningServer::followPathClientDoneCallback, this, std::placeholders::_1, std::placeholders::_2),
                            FollowPathClient::SimpleActiveCallback(),
                            FollowPathClient::SimpleFeedbackCallback());
}


nav_msgs::Path WholeBodyPlanningServer::extractPathFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory, const ros::Time& start_time) const
{
  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = start_time;
  Eigen::Affine3d previous_pose;
  for (unsigned int i = 0; i < trajectory.getWayPointCount(); ++i) {
    Eigen::Affine3d goal_pose = trajectory.getWayPoint(i).getJointTransform("world_virtual_joint");
    if (i > 0) {
      // Skip if waypoints are too close in the x-y plane
      double distance = (goal_pose.translation().block<2, 1>(0, 0) - previous_pose.translation().block<2, 1>(0, 0)).norm();
      if (distance < 0.01) {
        // preserve the last waypoint and instead remove the one before
        if (i == trajectory.getWayPointCount() - 1) {
          path.poses.pop_back();
        } else {
          continue;
        }
      }
    }
    previous_pose = goal_pose;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = start_time + ros::Duration(trajectory.getWayPointDurationFromStart(i));
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    path.poses.push_back(pose_msg);
  }
  return path;
}

std::string WholeBodyPlanningServer::pathToString(const nav_msgs::Path& path) const
{
  std::stringstream ss;
  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::Point& point = path.poses[i].pose.position;
    ss << std::endl << i << ": [" << point.x << ", " << point.y << ", " << point.z << "]";
  }
  return ss.str();
}

void WholeBodyPlanningServer::pathCb(const nav_msgs::PathConstPtr &path_msg) {
  planAndExecutePathFull(*path_msg, false);
}

void WholeBodyPlanningServer::confirmExecutionCb(const std_msgs::EmptyConstPtr&)
{
  if (!planned_trajectory_) {
    ROS_ERROR_STREAM("No trajectory planned yet, cannot execute");
    return;
  }
  ros::Time starting_time = ros::Time::now() + ros::Duration(1);
  if (!executeTrajectoryWithWatchdog(*planned_trajectory_, starting_time)) {
    ROS_ERROR_STREAM("Trajectory execution failed");
  }
}

void WholeBodyPlanningServer::jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  std::lock_guard<std::mutex> lock(current_robot_state_mutex_);
  current_robot_state_->setVariableValues(*joint_state_msg);
}
bool WholeBodyPlanningServer::followPathServerIsActive() const
{
  if (!follow_path_goal_.getGoal()) {
    return false;
  }
  unsigned int status = follow_path_goal_.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE ||
         status == actionlib_msgs::GoalStatus::PREEMPTING;
}
void WholeBodyPlanningServer::followPathClientDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                           const move_base_lite_msgs::FollowPathResultConstPtr& result)
{
  ROS_DEBUG_STREAM("Follow path client finished.");
  if (result->result.val == move_base_lite_msgs::ErrorCodes::SUCCESS) {
    if (end_of_path_reached_) {
      ROS_INFO_STREAM("Following path successful.");
      stopExecutionThread();
      if (followPathServerIsActive()) {
        follow_path_goal_.setSucceeded(*result, "successful");
      }
    }
  } else {
    ROS_ERROR_STREAM("Following path failed with error " << result->result.val);
    stopExecutionThread();
    // Path following failed
    // Stop moveit
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();
    if (followPathServerIsActive()) {
      follow_path_goal_.setCanceled(*result, "Following path failed");
    }
  }
}

void WholeBodyPlanningServer::dynamicReconfigureCallback(const whole_body_planning_server::PlanningServerConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure request received.");
  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  enabled_ = config.enabled;
  path_sampling_resolution_ = config.path_sampling_resolution;
  planning_horizon_ = config.planning_horizon;
  minimum_path_length_ = config.minimum_path_length;
  if (minimum_path_length_ < path_sampling_resolution_) {
    minimum_path_length_ = path_sampling_resolution_;
    ROS_WARN_STREAM("Minimum path length cannot be less than path resolution. Setting minimum path length to " << minimum_path_length_);
  }
  if (planning_horizon_ > 0 && minimum_path_length_ > planning_horizon_) {
    minimum_path_length_ = 0.9 * planning_horizon_;
    ROS_WARN_STREAM("Minimum path length cannot be greater than planning horizon. Setting minimum path length to " << minimum_path_length_);
  }
  auto_start_ = config.auto_start;
  continuous_planning_ = config.continuous_planning;
}
void WholeBodyPlanningServer::updateRobotPose(const nav_msgs::Path& path)
{
  Eigen::Isometry3d robot_pose_eigen;
  // First pose is current pose of robot
  const geometry_msgs::PoseStamped& robot_pose_msg = path.poses.front();
  tf::poseMsgToEigen(robot_pose_msg.pose, robot_pose_eigen);
  whole_body_posture_optimization::updateMoveitRobotState(*current_robot_state_, robot_pose_eigen);
}

bool WholeBodyPlanningServer::getRobotPose(const ros::Time& stamp, geometry_msgs::TransformStamped& transform) const
{
  try {
    // Get world transform
    transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, stamp, ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
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

void WholeBodyPlanningServer::enforceAccelerationBounds(const moveit::core::RobotStatePtr& robot_state_ptr)
{
  const std::vector<std::string>& variable_names = robot_state_ptr->getVariableNames();
  const moveit::core::RobotModelConstPtr& robot_model = robot_state_ptr->getRobotModel();
  for (int j = 0; j < robot_state_ptr->getVariableCount(); ++j) {
    const moveit::core::VariableBounds& bounds = robot_model->getVariableBounds(variable_names[j]);
    if (bounds.acceleration_bounded_) {
      double acceleration = robot_state_ptr->getVariableAcceleration(j);
      if (acceleration > bounds.max_acceleration_) {
        robot_state_ptr->setVariableAcceleration(j, bounds.max_acceleration_);
      } else if (acceleration < bounds.min_acceleration_) {
        robot_state_ptr->setVariableAcceleration(j, bounds.min_acceleration_);
      }
    }
  }
}

void WholeBodyPlanningServer::spinMapUpdate()
{
  ros::NodeHandle nh;
  ros::WallRate rate(10);
  while (nh.ok() && map_update_running_) {
    {
      std::lock_guard<std::mutex> lock(esdf_update_mutex_);
      esdf_server_queue_.callAvailable();
    }
    first_esdf_update_condition_.notify_all();
    rate.sleep();
  }
  ROS_DEBUG_STREAM("Map update thread finished.");
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
  trajectory_start_time_ = ros::Time();

  // We do not care about canceled goals
  if (driving_client_->getState() != actionlib::SimpleClientGoalState::LOST) {
    driving_client_->stopTrackingGoal();
  }
  driving_client_->cancelAllGoals();
  moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();
}

void WholeBodyPlanningServer::executionWatchDog()
{
  ros::Rate rate(10.0);
  while (!execution_completed_ && ros::ok())  {
    std::unique_lock<std::recursive_mutex> trajectory_lock(trajectory_mutex_);
    if (!trajectory_) {
      trajectory_lock.unlock();
      rate.sleep();
      continue;
    }

    // Get current robot pose
    ros::Time now = ros::Time::now();
    geometry_msgs::TransformStamped current_robot_pose_stamped;
    getRobotPose(now, current_robot_pose_stamped);
    Eigen::Isometry3d current_robot_pose;
    tf::transformMsgToEigen(current_robot_pose_stamped.transform, current_robot_pose);

    // Find expected robot pose at current time
    ros::Duration duration_after_start = now - trajectory_start_time_;
    int before_index, after_index;
    double progress; // value between 0 and 1
    trajectory_->findWayPointIndicesForDurationAfterStart(duration_after_start.toSec(), before_index, after_index, progress);
    const robot_state::RobotState& before_state = trajectory_->getWayPoint(before_index);
    const robot_state::RobotState& after_state = trajectory_->getWayPoint(after_index);

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
    robot_state::RobotState interpolated_state(trajectory_->getRobotModel());
    before_state.interpolate(after_state, progress, interpolated_state);
    whole_body_posture_optimization::updateMoveitRobotState(interpolated_state, interpolated_pose);
    moveit_msgs::DisplayRobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(interpolated_state, robot_state_msg.state);
    robot_state_pub_.publish(robot_state_msg);

    // Check position threshold
    double distance = (interpolated_pose.translation() - current_robot_pose.translation()).norm();
    bool position_out_of_limits = execution_position_tolerance_ > 0 && distance > execution_position_tolerance_;

    // Check orientation threshold
    Eigen::Quaterniond current_orientation(current_robot_pose.linear());
    double angular_distance = interpolated_orientation.angularDistance(current_orientation);
    bool orientation_out_of_limits = execution_orientation_tolerance_ > 0 && angular_distance > execution_orientation_tolerance_;

    bool stop_execution = false;
    if (position_out_of_limits) {
      ROS_WARN_STREAM("Position deviation of " << distance << " exceeds tolerance of " << execution_position_tolerance_);
      stop_execution = true;
    }
    if (orientation_out_of_limits) {
      ROS_WARN_STREAM("Orientation deviation of " << angular_distance << " exceeds tolerance of " << execution_orientation_tolerance_);
      stop_execution = true;
    }

    // Publish deviation
    std_msgs::Float64 deviation_msg;
    deviation_msg.data = distance;
    position_deviation_pub_.publish(deviation_msg);
    deviation_msg.data = angular_distance;
    orientation_deviation_pub_.publish(deviation_msg);

    if (stop_execution) {
      // Stop execution and fail goal (with stuck)
      stopExecution();
      std::lock_guard<std::mutex> lock_goal(follow_path_goal_mutex_);
      if (followPathServerIsActive()) {
        move_base_lite_msgs::FollowPathResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::STUCK_DETECTED;
        follow_path_goal_.setAborted(result, "Deviation between planned robot pose and current pose exceeds threshold.");
      }
      break;
    }

    trajectory_mutex_.unlock();
    rate.sleep();
  }
  ROS_DEBUG_STREAM("Watchdog thread finished");
}

void WholeBodyPlanningServer::waitForMap()
{
  std::unique_lock<std::mutex> esdf_update_lock(esdf_update_mutex_);
  if (sdf_model_->isEmpty()) {
    ROS_INFO_STREAM("Waiting for first ESDF");
    while (!execution_completed_ && sdf_model_->isEmpty()) {
      first_esdf_update_condition_.wait(esdf_update_lock);
    }
  }
}

void WholeBodyPlanningServer::saveTrajectoryStatistics(
    const std::vector<whole_body_posture_optimization::PostureOptimizationResult>& results,
    robot_trajectory::RobotTrajectory& trajectory, const std::vector<size_t>& trajectory_start_indices)
{
  if (!trajectory_statistics_save_folder_.empty() && end_of_path_reached_) {
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = trajectory_statistics_save_folder_ + "/trajectory_statistics-" + time_stamp + ".csv";
    if (whole_body_posture_optimization::WholeBodyPostureOptimization::saveTrajectoryStatistics(results, trajectory, trajectory_start_indices, save_path)) {
      ROS_INFO_STREAM("Saved trajectory statistics in " << save_path);
    }
  }
}

void WholeBodyPlanningServer::waitUntilPlanningHorizonReached(const geometry_msgs::Point& planning_point)
{
  bool too_far_away = false;
  ros::Rate rate(10.0);
  std::unique_lock<std::mutex> reconfigure_lock(reconfigure_mutex_);
  if (planning_horizon_ > 0) {
    reconfigure_lock.unlock();
    do {
      too_far_away = false;
      // Get current pose
      geometry_msgs::TransformStamped robot_transform;
      if (getRobotPose(ros::Time::now(), robot_transform)) {
        Eigen::Vector3d p1, p2;
        tf::vectorMsgToEigen(robot_transform.transform.translation, p1);
        tf::pointMsgToEigen(planning_point, p2);
        // Distance in x-y plane
        double distance = (p2.block<2, 1>(0, 0) - p1.block<2, 1>(0, 0)).norm();
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

}
