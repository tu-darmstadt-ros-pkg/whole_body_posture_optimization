#include <whole_body_planning_server/whole_body_planning_server.h>

#include <vehicle_controller/ps3d.h>

#include <sdf_contact_estimation/body_shapes/robot_shape_from_urdf.h>
#include <sdf_contact_estimation/util/utils.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>


namespace whole_body_planning_server {

WholeBodyPlanningServer::WholeBodyPlanningServer(const ros::NodeHandle &nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), auto_start_(false)
{
  // Parameters
  pnh_.param("auto_start", auto_start_, false);
  pnh_.param<std::string>("stability_info_save_folder", stability_info_save_folder_, "");
  pnh_.param<std::string>("complete_robot_group", move_group_, "complete_robot_group");
//  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<double>("path_sampling_resolution", path_sampling_resolution_, 0.02);

  // Create whole body path planner
  robot_shape_ = std::make_shared<sdf_contact_estimation::RobotShapeFromUrdf>(pnh_);
  optimizer_= std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh_, robot_shape_);
  joint_names_ = optimizer_->getRobotModel()->getActuatedJointNames();
  joint_states_ = optimizer_->getDefaultJointPositions();

  // Connect get tsdf layer service client
  ROS_INFO_STREAM("Waiting for get tsdf client..");
  get_tsdf_client_ = nh_.serviceClient<voxblox_msgs::GetLayer>("get_tsdf");
  if (!get_tsdf_client_.waitForExistence(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for tsdf server.");
  }

  // Connect trajectory execution service client
  ROS_INFO_STREAM("Waiting for trajectory execution client..");
  execute_trajectory_client_ = std::make_shared<ExecuteTrajectoryClient>(nh_, "/execute_trajectory", true);
  if (!execute_trajectory_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for execute_trajectory action server.");
  }

  // Connect vehicle controller action client
  ROS_INFO_STREAM("Waiting for vehicle controller client..");
  driving_client_ = std::make_shared<FollowPathClient>(nh_, "/controller/follow_path", true);
  if (!driving_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for driving action server.");
  }

  // Connect move group action client
  ROS_INFO_STREAM("Waiting for move_group client..");
  move_group_client_ = std::make_shared<MoveGroupClient>(nh_, "/move_group", true);
  if (!move_group_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for move_group action server.");
  }

  // Create FollowPathServer
  follow_path_as_ = std::make_shared<FollowPathServer>(pnh, "follow_path", boost::bind(&WholeBodyPlanningServer::followPathServerExecuteCb, this, _1), false);
  follow_path_as_->start();

  // Start subscribers
  path_sub_ = pnh_.subscribe("path_command", 10, &WholeBodyPlanningServer::pathCb, this);
  execute_sub_ = pnh_.subscribe("execute_planned_trajectory", 10, &WholeBodyPlanningServer::confirmExecutionCb, this);
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &WholeBodyPlanningServer::jointStateCb, this);
  ROS_INFO_STREAM("Initialization finished");
}

bool WholeBodyPlanningServer::loadTsdfFromPath(std::string tsdf_path)
{
  ROS_INFO_STREAM("Loading tsdf from path '" << tsdf_path << "'.");
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
  if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(tsdf_path, &layer_ptr)) {
    loaded_tsdf_ = std::make_shared<voxblox::TsdfMap>(layer_ptr);
    return true;
  } else {
    ROS_ERROR_STREAM("Failed to load tsdf from file.");
    return false;
  }
}

bool WholeBodyPlanningServer::planAndExecutePath(const nav_msgs::Path& path_msg, bool auto_start)
{
  ROS_INFO_STREAM("Received new path, planning..");
  map_frame_ = path_msg.header.frame_id;

  // Path pre-processing
  nav_msgs::Path path_smoothed = preprocessPath(path_msg);

  if (!continuous_replanning_) {
    // Generate new plan of complete path
    bool planning_success = generatePlan(path_smoothed, trajectory_);
    if (!planning_success) {
      return false;
    }

    // Execute
    if (auto_start) {
      ros::Time starting_time = ros::Time::now() + ros::Duration(2);
      return executeTrajectory(trajectory_, starting_time);
    }
  } else {
    // Start separate thread to plan/execute parts of the trajectory periodically
  }

  return true;
}

nav_msgs::Path WholeBodyPlanningServer::preprocessPath(const nav_msgs::Path& path) const
{
  // Re-sample and smooth path
  Pathsmoother3D path_smoother(true);
  path_smoother.setSmoothedPathDiscretization(path_sampling_resolution_);
  nav_msgs::Path path_copy = path_smoother.smooth(path, true);

  // Replace first pose with current pose for better planning initialization
  geometry_msgs::TransformStamped base_transform;
  try {
    // Get world transform
    base_transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, path.header.stamp);
    path_copy.poses.front().pose.orientation = base_transform.transform.rotation; // TODO only YAW
    path_copy.poses.front().pose.position.z = base_transform.transform.translation.z;

//    Eigen::Affine3d first_pose;
//    tf::poseMsgToEigen(path_copy.poses[0].pose, first_pose);
//    first_pose = sdf_contact_estimation::updatePoseFromLastResult(first_pose, map_to_base_tf, true, false); // Z only
//    tf::poseEigenToMsg(first_pose, path_copy.poses[0].pose);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  return path_copy;
}

bool WholeBodyPlanningServer::generatePlan(const nav_msgs::Path& path_msg, std::vector<robot_trajectory::RobotTrajectory>& trajectory_out)
{
  if (path_msg.poses.size() == 0) {
    ROS_WARN_STREAM("Path is empty. Can't plan.");
    return true; // Executing an empty plan is always successful
  }

  // *** Update worldmodel ***
  if (!updateTsdf()) {
    ROS_ERROR_STREAM("Failed to update world model.");
    return false;
  }

  // *** Compute plan ***
  auto result = optimizer_->findOptimalPostures(path_msg);
  trajectory_out = optimizer_->computeTimedTrajectory(result);
  if (trajectory_out.size() < 1) {
    return false;
  }
  // Statistics
  whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
  optimizer_->printContactEstimationStatistics();
  if (stability_info_save_folder_ != "") {
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = stability_info_save_folder_ + "/stability_" + time_stamp + ".dat";
    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectory_out, save_path);
  }
  return true;
}

bool WholeBodyPlanningServer::executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory> &trajectories, ros::Time starting_time, bool move_to_start_state) {
  ROS_INFO_STREAM("Executing trajectory now.");
  if (trajectories.empty()) {
    ROS_WARN_STREAM("[executeTrajectory] Trajectory is empty.");
    return true;
  }

  // Merge trajectory-parts
  robot_trajectory::RobotTrajectory combined_trajectory(trajectories[0].getRobotModel(), move_group_);
  for (unsigned int i = 0; i < trajectories.size(); i++) {
    combined_trajectory.append(trajectories[i], 0.000000001);
  }

  if (move_to_start_state) {
    // *** Drive to starting pose ***
    ROS_INFO_STREAM("Moving to starting pose");
    if (!planAndMoveToState(trajectories[0].getFirstWayPoint(), combined_trajectory.getGroup())) {
      ROS_WARN_STREAM("Moving to initial state failed.");
      return false;
    }
  }

  // *** Execute whole body trajectory ***

  // Assemble arm motion goal
  moveit_msgs::ExecuteTrajectoryGoal execute_trajectory_goal;
  combined_trajectory.getRobotTrajectoryMsg(execute_trajectory_goal.trajectory);
  execute_trajectory_goal.trajectory.joint_trajectory.header.stamp = starting_time;

  // Assemble driving plan
  nav_msgs::Path path = extractPathFromTrajectories(trajectories, starting_time);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  follow_path_goal.follow_path_options.desired_speed = 0.0; // Implicitly given by pose time stamps
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
  follow_path_goal.follow_path_options.reverse_allowed = true;
  follow_path_goal.follow_path_options.is_fixed = true; // No smoothing
  follow_path_goal.follow_path_options.reset_stuck_history = true; // Mandatory, robot will not drive otherwise
  follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.01;
  follow_path_goal.follow_path_options.goal_pose_angle_tolerance = 0.01;

  ROS_INFO_STREAM("Executing planned trajectory");

  // Send goals simultaneously
  execute_trajectory_client_->sendGoal(execute_trajectory_goal);
  driving_client_->sendGoal(follow_path_goal, FollowPathClient::SimpleDoneCallback(), FollowPathClient::SimpleActiveCallback(),
                            boost::bind(&WholeBodyPlanningServer::followPathClientFeedbackCb, this, _1));

  ros::Duration time_until_start = starting_time - ros::Time::now();
  time_until_start.sleep();
  optimizer_->publishTrajectory(trajectories);

  // Wait for results
  if (!driving_client_->waitForResult()) {
    ROS_INFO_STREAM("FollowPath action failed.");
    return false;
  }
  if (!execute_trajectory_client_->waitForResult()) {
    ROS_INFO_STREAM("ExecuteTrajectory action failed");
    return false;
  }

  moveit_msgs::ExecuteTrajectoryResultConstPtr execute_trajectory_result = execute_trajectory_client_->getResult();
  if (execute_trajectory_result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM("Trajectory execution failed with error code: " << execute_trajectory_result->error_code.val);
    return false;
  }

  move_base_lite_msgs::FollowPathResultConstPtr follow_path_result = driving_client_->getResult();
  if (follow_path_result->result.val != move_base_lite_msgs::ErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM("Follow path action failed with error code: " << follow_path_result->result.val);
    return false;
  }


  ROS_INFO_STREAM("Trajectory execution successful!");
  return true;
}

void WholeBodyPlanningServer::followPathClientFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr &feedback_ptr)
{
}

void WholeBodyPlanningServer::followPathServerExecuteCb(const move_base_lite_msgs::FollowPathGoalConstPtr& goal_ptr)
{
  bool success = planAndExecutePath(goal_ptr->target_path, auto_start_);
  move_base_lite_msgs::FollowPathResult result;
  if (success) {
    result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
  } else {
    result.result.val = move_base_lite_msgs::ErrorCodes::FAILURE;
  }
  follow_path_as_->setSucceeded(result);
}

bool WholeBodyPlanningServer::planAndMoveToState(const moveit::core::RobotState &goal_state, const robot_model::JointModelGroup* jmg)
{
  if (jmg == nullptr) {
    ROS_ERROR_STREAM("JointModelGroup is null.");
    return false;
  }
  // First, drive to pose in goal state
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = map_frame_;
  Eigen::Affine3d goal_pose = goal_state.getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(goal_pose, goal_pose_msg.pose);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path.header.frame_id = map_frame_;
  follow_path_goal.target_path.header.stamp = ros::Time::now();
  follow_path_goal.target_path.poses.push_back(goal_pose_msg);
  follow_path_goal.follow_path_options.reset_stuck_history = true;
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
  follow_path_goal.follow_path_options.reverse_allowed = true;
//  follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.01;
//  follow_path_goal.follow_path_options.goal_pose_angle_tolerance = 0.01;

  driving_client_->sendGoal(follow_path_goal);
  if (!driving_client_->waitForResult()) {
    ROS_ERROR_STREAM("Follow path action timed out.");
    return false;
  }
  move_base_lite_msgs::FollowPathResultConstPtr follow_path_result = driving_client_->getResult();
  if (follow_path_result->result.val != move_base_lite_msgs::ErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM("Follow path action failed with error code: " << follow_path_result->result.val);
    return false;
  }

  // Plan and execute joint motion from current to goal state
  moveit_msgs::MoveGroupGoal move_group_goal;
  move_group_goal.request.group_name = jmg->getName();
  move_group_goal.request.num_planning_attempts = 3;
  move_group_goal.request.allowed_planning_time = 5;
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
  move_group_goal.request.goal_constraints.push_back(goal_constraint);

  move_group_goal.planning_options.plan_only = false;

  // all diffs?
  //  move_group_goal.request.start_state.is_diff = true;
  //  move_group_goal.planning_options.planning_scene_diff = true;

  move_group_client_->sendGoal(move_group_goal);
  if (!move_group_client_->waitForResult()) {
    ROS_ERROR_STREAM("Move group action timed out.");
    return false;
  }

  moveit_msgs::MoveGroupResultConstPtr move_group_result = move_group_client_->getResult();
  if (move_group_result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM("Move group action failed with error code: " << move_group_result->error_code.val);
    return false;
  }

  return true;
}

bool WholeBodyPlanningServer::updateTsdf()
{
  if (loaded_tsdf_) {
    // Use tsdf loaded from disc
    optimizer_->updateWorldModel(loaded_tsdf_);
    return true;
  }

  // Request tsdf from server
  voxblox_msgs::GetLayer get_layer;
//  get_layer.request.action = voxblox_msgs::Layer::ACTION_FULL_MAP;
  if (!get_tsdf_client_.call(get_layer)) {
    ROS_ERROR_STREAM("Failed to retrieve the latest tsdf map.");
    return false;
  }
  if (!tsdf_) {
    voxblox::TsdfMap::Config tsdf_config;
    tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(get_layer.response.layer.voxel_size);
    tsdf_.reset(new voxblox::TsdfMap(tsdf_config));
  }
  voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(get_layer.response.layer, tsdf_->getTsdfLayerPtr());
  optimizer_->updateWorldModel(tsdf_);

  return true;
}

bool WholeBodyPlanningServer::continuousReplanning(const nav_msgs::Path& path_msg)
{
  // Parameters
//  double min_distance_from_robot = 0.3; // Distance where plan starts
  double planning_horizon = 0.3; // Distance where plan ends
  unsigned int max_consecutive_planning_failures = 5;

  whole_body_posture_optimization::PostureOptimizationResult last_opt_result;
  std::vector<robot_trajectory::RobotTrajectory> full_trajectory;
  std::vector<ros::Time> start_times;
  std::vector<double> accumulated_distances;
  bool first_cycle = true;
  unsigned int consecutive_planning_failures = 0;
  size_t current_path_index = 0;
  size_t planning_index = 0;
  // Iterate until path finished or planning failure
  while (consecutive_planning_failures < max_consecutive_planning_failures && planning_index < path_msg.poses.size()) {
    // Compute new path slice
    nav_msgs::Path path_slice;
    path_slice.header.frame_id = path_msg.header.frame_id;

    if (first_cycle) {
      auto end_it = std::upper_bound(accumulated_distances.begin(), accumulated_distances.end(), planning_horizon);
      size_t end_index;
      if (end_it != accumulated_distances.end()) {
        end_index = static_cast<size_t>(end_it - accumulated_distances.begin()) -1;
      } else {
        ROS_WARN_STREAM("Planning horizon has reached end of path.");
        end_index = accumulated_distances.size() - 1;
      }
      path_slice.poses = std::vector<geometry_msgs::PoseStamped>(path_msg.poses.begin(), path_msg.poses.begin() + end_index +1);
      planning_index = end_index;
    } else {
      path_slice.poses = std::vector<geometry_msgs::PoseStamped>(1, path_msg.poses[planning_index]);
    }

    // Update TSDF
    if (!updateTsdf()) {
      ROS_ERROR_STREAM("Failed to update TSDF. Skipping planning.");
      ++consecutive_planning_failures;
      continue;
    }

    // Compute plan for path slice
    auto result = optimizer_->findOptimalPostures(path_slice); // TODO give previous solution

    if (!first_cycle) {
      // Append previous result to front so a trajectory is created from the previous slice to the new one
      result.insert(result.begin(), last_opt_result);
    }
    trajectory_ = optimizer_->computeTimedTrajectory(result); // trajectory.size() == result.size() - 1
    if (trajectory_.size() < 1) {
      ROS_ERROR_STREAM("Trajectory generation failed. Skipping planning.");
      ++consecutive_planning_failures;
      continue;
    }
    last_opt_result = result.back();
    full_trajectory.insert(full_trajectory.end(), trajectory_.begin(), trajectory_.end());

    // TODO: Re-compute time-parameterization on full trajectory

    // Update the start times of each trajectory
    if (first_cycle) {
      start_times.resize(1, ros::Time::now());
    }
    ros::Time next_start_time = start_times[start_times.size()-1];
    start_times.reserve(start_times.size() + trajectory_.size());
    for (unsigned int i = 0; i < trajectory_.size(); ++i) {
      start_times.push_back(start_times[start_times.size()-1] + ros::Duration(trajectory_[i].getDuration()));
    }

    // Reset failure count
    consecutive_planning_failures = 0;

    // Send plan
    executeTrajectory(trajectory_, next_start_time, first_cycle); // TODO Has to be non blocking

    if (first_cycle) {
      first_cycle = false;
    }

    // Sleep until one trajectory is finished
    ros::Time time_of_next_cycle = start_times[current_path_index + 1];
    ros::Time now = ros::Time::now();
    if (time_of_next_cycle >= now) {
      ros::Duration time_to_sleep = time_of_next_cycle - now;
      time_to_sleep.sleep();
    } else {
      ros::Duration late = now - time_of_next_cycle;
      ROS_WARN_STREAM("Next cycle is starting late (" << late.toSec() << ")");
    }

    ++planning_index;
    ++current_path_index; // We should be at the next path index after sleep time
  }

  if (consecutive_planning_failures >= max_consecutive_planning_failures) {
    ROS_ERROR_STREAM("Continuous planning aborted. Maximum consecutive planning failures reached (" << max_consecutive_planning_failures << ").");
    return false;
  }

  return true;
}

nav_msgs::Path WholeBodyPlanningServer::extractPathFromTrajectories(const std::vector<robot_trajectory::RobotTrajectory>& trajectories, const ros::Time& start_time) const
{
  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = ros::Time::now();
  ros::Time next_start_time = start_time;
  for (unsigned int i = 0; i < trajectories.size(); i++) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = next_start_time;
    Eigen::Affine3d goal_pose = trajectories[i].getFirstWayPoint().getJointTransform("world_virtual_joint");
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    path.poses.push_back(pose_msg);

    next_start_time += ros::Duration(trajectories[i].getWayPointDurationFromStart(trajectories[i].getWayPointCount()-1));
  }
  // add last pose
  geometry_msgs::PoseStamped end_pose_msg;
  end_pose_msg.header.frame_id = map_frame_;
  end_pose_msg.header.stamp = next_start_time;
  Eigen::Affine3d end_goal_pose = trajectories.back().getLastWayPoint().getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(end_goal_pose, end_pose_msg.pose);
  path.poses.push_back(end_pose_msg);
  return path;
}

Eigen::Vector3d WholeBodyPlanningServer::projectOntoPath(const nav_msgs::Path& path, geometry_msgs::Point& point)
{
  return Eigen::Vector3d::Zero();
}

std::vector<double> WholeBodyPlanningServer::computeAccumulatedDistances(const nav_msgs::Path& path) const
{
//  vector<double> result(positions.size(), 0);
//  for(unsigned i = 1; i < positions.size(); i++)
//    result[i] = result[i - 1] + (positions[i] - positions[i - 1]).norm();
//  return result;
}

void WholeBodyPlanningServer::pathCb(const nav_msgs::PathConstPtr &path_msg) {
  planAndExecutePath(*path_msg, auto_start_);
}

void WholeBodyPlanningServer::confirmExecutionCb(const std_msgs::EmptyConstPtr&)
{
  ros::Time starting_time = ros::Time::now() + ros::Duration(2);
  executeTrajectory(trajectory_, starting_time);
}

void WholeBodyPlanningServer::jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  bool state_changed = false;
  for (unsigned int joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx) {
    std::vector<std::string>::const_iterator it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), joint_names_[joint_idx]);
    if (it != joint_state_msg->name.end()) {
      size_t index = static_cast<size_t>(it - joint_state_msg->name.begin());
      joint_states_[joint_idx] = joint_state_msg->position[index];
      state_changed = true;
    }
  }
  if (state_changed) {
    optimizer_->setDefaultJointPositions(joint_states_);
  }
}

}
