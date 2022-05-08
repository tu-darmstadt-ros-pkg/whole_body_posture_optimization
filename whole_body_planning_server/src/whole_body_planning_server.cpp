#include <whole_body_planning_server/whole_body_planning_server.h>

#include <vehicle_controller/ps3d.h>

#include <sdf_contact_estimation/util/utils.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

#include <eigen_conversions/eigen_msg.h>


namespace whole_body_planning_server {

WholeBodyPlanningServer::WholeBodyPlanningServer(const ros::NodeHandle &nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), esdf_server_(nh, ros::NodeHandle(pnh, "esdf_server")), auto_start_(false), path_sampling_resolution_(0.02)
{
  // Parameters
  pnh_.param<std::string>("stability_info_save_folder", stability_info_save_folder_, "");
  pnh_.param<std::string>("complete_robot_group", move_group_, "complete_robot_group");
//  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param("override_path_time", override_path_time_, true);

  // Dynamic reconfigure
  reconfigure_server_.setCallback(std::bind(&WholeBodyPlanningServer::dynamicReconfigureCallback,
                                            this, std::placeholders::_1, std::placeholders::_2));

  // Create SDF Model
  sdf_model_ = std::make_shared<sdf_contact_estimation::SdfModel>(pnh);
  sdf_model_->loadEsdf(esdf_server_.getEsdfMapPtr(), esdf_server_.getEsdfMaxDistance(), true);

  // Create pose predictor
  // Create robot model
  ros::NodeHandle pose_predictor_nh(pnh, "pose_predictor");
  ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model_);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  optimizer_ = std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh, pose_predictor, sdf_model_);
  joint_names_ = optimizer_->getRobotModel()->getActuatedJointNames();
  joint_states_ = optimizer_->getDefaultJointPositions();

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

bool WholeBodyPlanningServer::loadTsdfFromPath(std::string tsdf_path)
{
  ROS_INFO_STREAM("Loading sdf from path '" << tsdf_path << "'.");
  if (!esdf_server_.loadMap(tsdf_path)) {
    ROS_ERROR_STREAM("Failed to load tsdf from file.");
    return false;
  }
  return true;
}

bool WholeBodyPlanningServer::planAndExecutePath(const nav_msgs::Path& path_msg, bool auto_start, bool replanning)
{
  ROS_INFO_STREAM("Received new path, planning..");
  map_frame_ = path_msg.header.frame_id;


  // Path pre-processing
  nav_msgs::Path path_smoothed = preprocessPath(path_msg);
  nav_msgs::Path path_slice = slicePath(path_smoothed, path_slice_length_);
  if(!generatePlan(path_slice, trajectory_)) {
    ROS_ERROR_STREAM("Planning failed.");
    return false;
  }


  ros::Time starting_time;
  if (replanning) {
    // Trajectory has to start here in time, because planning takes time and robot might not be at the start state anymore
    starting_time = path_msg.header.stamp;
  } else {
    // If this is the first trajectory, future-date the starting time, so we do not miss the first waypoint
    starting_time = ros::Time::now() + ros::Duration(0.1);
  }

  if (auto_start) {
    if (!executeTrajectory(trajectory_, starting_time)) {
      ROS_ERROR_STREAM("Trajectory execution failed");
      return false;
    }
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
    base_transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, path.header.stamp, ros::Duration(0.1));
    path_copy.poses.front().pose.orientation = base_transform.transform.rotation; // TODO only YAW
    path_copy.poses.front().pose.position.z = base_transform.transform.translation.z;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  return path_copy;
}

nav_msgs::Path WholeBodyPlanningServer::slicePath(const nav_msgs::Path& path, double distance)
{
  // Special case: always return original path if distance is <0
  if (distance < 0) {
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

bool WholeBodyPlanningServer::generatePlan(const nav_msgs::Path& path_msg, std::vector<robot_trajectory::RobotTrajectory>& trajectory_out)
{
  ROS_INFO_STREAM("Planning a new path");
  if (path_msg.poses.empty()) {
    ROS_WARN_STREAM("Path is empty. Can't plan.");
    return true; // Executing an empty plan is always successful
  }

  sdf_model_->publishSdfMesh();

  // *** Compute plan ***
  auto result = optimizer_->findOptimalPostures(path_msg);
  trajectory_out = optimizer_->computeTimedTrajectory(result);
  if (trajectory_out.empty()) {
    return false;
  }
  // Statistics
//  whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
//  if (stability_info_save_folder_.empty()) {
//    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
//    std::string save_path = stability_info_save_folder_ + "/stability_" + time_stamp + ".dat";
//    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectory_out, save_path);
//  }
  return true;
}

//bool WholeBodyPlanningServer::moveToStartAndExecute(const std::vector<robot_trajectory::RobotTrajectory> &trajectories) {
//  ROS_INFO_STREAM("Executing trajectory now.");
//  if (trajectories.empty()) {
//    ROS_WARN_STREAM("[executeTrajectory] Trajectory is empty.");
//    return true;
//  }
//
//  // *** Drive to starting pose ***
//  ROS_INFO_STREAM("Moving to starting pose");
//  if (!planAndMoveToState(trajectories[0].getFirstWayPoint(), move_group_)) {
//    ROS_WARN_STREAM("Moving to initial state failed.");
//    return false;
//  }
//
//  // *** Execute whole body trajectory ***
//  ros::Time starting_time = ros::Time::now() + ros::Duration(2);
//  if (executeTrajectory(trajectories, starting_time, true)) {
//    ROS_INFO_STREAM("Trajectory execution successful!");
//    return true;
//  }
//  return false;
//}

bool WholeBodyPlanningServer::executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectories, ros::Time starting_time)
{
  ROS_INFO_STREAM("Starting trajectory execution");
  if (trajectories.empty()) {
    ROS_WARN_STREAM("Trajectory is empty, execution is trivially successful.");
    return true;
  }

  // Merge trajectory-parts
  robot_trajectory::RobotTrajectory combined_trajectory(trajectories[0].getRobotModel(), move_group_);
  for (const auto & trajectory : trajectories) {
    combined_trajectory.append(trajectory, 0.000000001);
  }

  // Assemble driving plan
  nav_msgs::Path path = extractPathFromTrajectories(trajectories, starting_time);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  follow_path_goal.follow_path_options.desired_speed = 0.0; // Implicitly given by pose time stamps
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = follow_path_goal_.getGoal()->follow_path_options.rotate_front_to_goal_pose_orientation;
  follow_path_goal.follow_path_options.reverse_allowed = follow_path_goal_.getGoal()->follow_path_options.reverse_allowed;
  follow_path_goal.follow_path_options.is_fixed = true; // No smoothing
//  follow_path_goal.follow_path_options.reset_stuck_history = true; // Mandatory, robot will not drive otherwise
  follow_path_goal.follow_path_options.reset_stuck_history = follow_path_goal_.getGoal()->follow_path_options.reset_stuck_history;
  follow_path_goal.follow_path_options.goal_pose_position_tolerance = follow_path_goal_.getGoal()->follow_path_options.goal_pose_position_tolerance;
  follow_path_goal.follow_path_options.goal_pose_angle_tolerance = follow_path_goal_.getGoal()->follow_path_options.goal_pose_angle_tolerance;

  ROS_INFO_STREAM("Executing planned trajectory");

  // Send goals simultaneously
  driving_client_->sendGoal(follow_path_goal,
                            std::bind(&WholeBodyPlanningServer::followPathClientDoneCallback, this, std::placeholders::_1, std::placeholders::_2),
                            FollowPathClient::SimpleActiveCallback(),
                            FollowPathClient::SimpleFeedbackCallback());
  if (!executeJointTrajectory(combined_trajectory, false, starting_time)) {
    return false;
  }


//  ros::Duration time_until_start = starting_time - ros::Time::now();
//  time_until_start.sleep();
//  optimizer_->publishTrajectory(trajectories);

//  if (blocking) {
//    // Wait for results
//    if (!driving_client_->waitForResult()) {
//      ROS_INFO_STREAM("FollowPath action failed.");
//      return false;
//    }
//    if (!moveit_cpp_ptr_->getTrajectoryExecutionManager()->waitForExecution()) {
//      ROS_INFO_STREAM("ExecuteTrajectory action failed");
//      return false;
//    }
//  }

//  moveit_msgs::ExecuteTrajectoryResultConstPtr execute_trajectory_result = execute_trajectory_client_->getResult();
//  if (execute_trajectory_result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
//    ROS_ERROR_STREAM("Trajectory execution failed with error code: " << execute_trajectory_result->error_code.val);
//    return false;
//  }

//  move_base_lite_msgs::FollowPathResultConstPtr follow_path_result = driving_client_->getResult();
//  if (follow_path_result->result.val != move_base_lite_msgs::ErrorCodes::SUCCESS) {
//    ROS_ERROR_STREAM("Follow path action failed with error code: " << follow_path_result->result.val);
//    return false;
//  }

  return true;
}

//void WholeBodyPlanningServer::followPathServerExecuteCb(const move_base_lite_msgs::FollowPathGoalConstPtr& goal_ptr)
//{
//  bool success = planAndExecutePath(goal_ptr->target_path, auto_start_);
//  move_base_lite_msgs::FollowPathResult result;
//  if (success) {
//    result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
//  } else {
//    result.result.val = move_base_lite_msgs::ErrorCodes::FAILURE;
//  }
//  follow_path_as_->setSucceeded(result);
//}

void WholeBodyPlanningServer::followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal)
{
  ROS_INFO_STREAM("New goal received.");
  // Check if another goal exists
  bool goal_preempted = false;
  if (followPathServerIsActive()) {
    // Check if incoming one is newer
    if (goal.getGoalID().stamp >= follow_path_goal_.getGoalID().stamp) {
      // Cancel previous goal
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

  // Override path time with receive time
  nav_msgs::Path target_path = goal.getGoal()->target_path;
  if (override_path_time_)
    target_path.header.stamp = ros::Time::now();


  // Plan and execute
  if (enabled_) {
    planAndExecutePath(target_path, auto_start_, goal_preempted);
  } else {
    driving_client_->sendGoal(*goal.getGoal(),
                              std::bind(&WholeBodyPlanningServer::followPathClientDoneCallback, this, std::placeholders::_1, std::placeholders::_2),
                              FollowPathClient::SimpleActiveCallback(),
                              FollowPathClient::SimpleFeedbackCallback());
  }
}
void WholeBodyPlanningServer::followPathPreemptCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle preempt)
{
  if (preempt == follow_path_goal_) {
    // Cancel vehicle controller and stop moveit execution
    driving_client_->cancelAllGoals();
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();

    move_base_lite_msgs::FollowPathResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    follow_path_goal_.setCanceled(result, "Preempt from incoming message to server");
    ROS_INFO_STREAM("Goal preempted, stopping vehicle");
  } else {
    ROS_WARN_STREAM("Received preempt for unknown goal");
  }
}

//bool WholeBodyPlanningServer::planAndMoveToState(const moveit::core::RobotState &goal_state, std::string group_name)
//{
//  const robot_model::JointModelGroup* jmg = moveit_cpp_ptr_->getRobotModel()->getJointModelGroup(group_name);
//  if (jmg == nullptr) {
//    ROS_ERROR_STREAM("Could not find group '" << group_name << "'");
//    return false;
//  }
//  // First, drive to pose in goal state
//  geometry_msgs::PoseStamped goal_pose_msg;
//  goal_pose_msg.header.frame_id = map_frame_;
//  Eigen::Affine3d goal_pose = goal_state.getJointTransform("world_virtual_joint");
//  tf::poseEigenToMsg(goal_pose, goal_pose_msg.pose);
//
//  move_base_lite_msgs::FollowPathGoal follow_path_goal;
//  follow_path_goal.target_path.header.frame_id = map_frame_;
//  follow_path_goal.target_path.header.stamp = ros::Time::now();
//  follow_path_goal.target_path.poses.push_back(goal_pose_msg);
//  follow_path_goal.follow_path_options.reset_stuck_history = true;
//  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
//  follow_path_goal.follow_path_options.reverse_allowed = true;
////  follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.01;
////  follow_path_goal.follow_path_options.goal_pose_angle_tolerance = 0.01;
//
//  driving_client_->sendGoal(follow_path_goal);
//  if (!driving_client_->waitForResult()) {
//    ROS_ERROR_STREAM("Follow path action timed out.");
//    return false;
//  }
//  move_base_lite_msgs::FollowPathResultConstPtr follow_path_result = driving_client_->getResult();
//  if (follow_path_result->result.val != move_base_lite_msgs::ErrorCodes::SUCCESS) {
//    ROS_ERROR_STREAM("Follow path action failed with error code: " << follow_path_result->result.val);
//    return false;
//  }
//
//  // Plan and execute joint motion from current to goal state
//  moveit_msgs::MoveGroupGoal move_group_goal;
//  move_group_goal.request.group_name = jmg->getName();
//  move_group_goal.request.num_planning_attempts = 3;
//  move_group_goal.request.allowed_planning_time = 5;
//  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
//  move_group_goal.request.goal_constraints.push_back(goal_constraint);
//
//  move_group_goal.planning_options.plan_only = false;
//
//  // all diffs?
//  //  move_group_goal.request.start_state.is_diff = true;
//  //  move_group_goal.planning_options.planning_scene_diff = true;
//
//  move_group_client_->sendGoal(move_group_goal);
//  if (!move_group_client_->waitForResult()) {
//    ROS_ERROR_STREAM("Move group action timed out.");
//    return false;
//  }
//
//  moveit_msgs::MoveGroupResultConstPtr move_group_result = move_group_client_->getResult();
//  if (move_group_result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
//    ROS_ERROR_STREAM("Move group action failed with error code: " << move_group_result->error_code.val);
//    return false;
//  }
//
//  return true;
//}

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

//bool WholeBodyPlanningServer::continuousReplanning(const nav_msgs::Path& path_msg)
//{
//  ROS_INFO_STREAM("Input path: " << pathToString(path_msg));
//  // Parameters
////  double min_distance_from_robot = 0.3; // Distance where plan starts
//  double planning_horizon = 0.3; // Distance where plan ends
//  unsigned int max_consecutive_planning_failures = 5;
//
//  whole_body_posture_optimization::PostureOptimizationResult last_opt_result;
//  std::vector<robot_trajectory::RobotTrajectory> full_trajectory;
//  std::vector<ros::Time> start_times;
//  std::vector<double> accumulated_distances;
//  bool first_cycle = true;
//  unsigned int consecutive_planning_failures = 0;
//  size_t current_path_index = 0;
//  size_t planning_index = 0;
//  // Iterate until path finished or planning failure
//  ROS_INFO_STREAM("Starting replanning");
//  ROS_INFO_STREAM("------------------------------");
//  while (consecutive_planning_failures < max_consecutive_planning_failures && planning_index < path_msg.poses.size()) {
//    // Compute new path slice
//    nav_msgs::Path path_slice;
//    path_slice.header.frame_id = path_msg.header.frame_id;
//
//    if (first_cycle) {
//      auto end_it = std::upper_bound(accumulated_distances.begin(), accumulated_distances.end(), planning_horizon);
//      size_t end_index;
//      if (end_it != accumulated_distances.end()) {
//        end_index = static_cast<size_t>(end_it - accumulated_distances.begin()) -1;
//      } else {
//        ROS_WARN_STREAM("Planning horizon has reached end of path.");
//        end_index = accumulated_distances.size() - 1;
//      }
//      path_slice.poses = std::vector<geometry_msgs::PoseStamped>(path_msg.poses.begin(), path_msg.poses.begin() + end_index +1);
//      planning_index = end_index;
//    } else {
//      path_slice.poses = std::vector<geometry_msgs::PoseStamped>(1, path_msg.poses[planning_index]);
//    }
//    ROS_INFO_STREAM("path index: " << current_path_index << ", planning index: " << planning_index);
//    ROS_INFO_STREAM("Planning for path: " << pathToString(path_slice));
//
//    // Compute plan for path slice
//    auto result = optimizer_->findOptimalPostures(path_slice); // TODO give previous solution
//
//    if (!first_cycle) {
//      // Append previous result to front so a trajectory is created from the previous slice to the new one
//      result.insert(result.begin(), last_opt_result);
//    }
//    trajectory_ = optimizer_->computeTimedTrajectory(result); // trajectory.size() == result.size() - 1
//    if (trajectory_.empty()) {
//      ROS_ERROR_STREAM("Trajectory generation failed. Skipping planning.");
//      ++consecutive_planning_failures;
//      continue;
//    }
//    last_opt_result = result.back();
//    full_trajectory.insert(full_trajectory.end(), trajectory_.begin(), trajectory_.end());
//
//    // TODO: Re-compute time-parameterization on full trajectory
//
//    // Update the start times of each trajectory
//    if (first_cycle) {
//      start_times.resize(1, ros::Time::now());
//    }
//    ros::Time next_start_time = start_times[start_times.size()-1];
//    start_times.reserve(start_times.size() + trajectory_.size());
//    for (unsigned int i = 0; i < trajectory_.size(); ++i) {
//      start_times.push_back(start_times[start_times.size()-1] + ros::Duration(trajectory_[i].getDuration()));
//    }
//
//    // Reset failure count
//    consecutive_planning_failures = 0;
//
//    // Send plan
//    if (first_cycle) {
//      ROS_INFO_STREAM("Moving to starting pose");
//      if (!planAndMoveToState(trajectory_[0].getFirstWayPoint(), move_group_)) {
//        ROS_WARN_STREAM("Moving to initial state failed.");
//        continue;
//      }
//      first_cycle = false;
//    }
//    executeTrajectory(trajectory_, next_start_time, false);
//
//    // Sleep until one trajectory is finished
//    ros::Time time_of_next_cycle = start_times[current_path_index + 1];
//    ros::Time now = ros::Time::now();
//    if (time_of_next_cycle >= now) {
//      ros::Duration time_to_sleep = time_of_next_cycle - now;
//      ROS_INFO_STREAM("Next cycle in " << time_to_sleep.toSec() << " s.");
//      time_to_sleep.sleep();
//    } else {
//      ros::Duration late = now - time_of_next_cycle;
//      ROS_WARN_STREAM("Next cycle is starting late (" << late.toSec() << " s)");
//    }
//    ros::spinOnce();
//
//    ++planning_index;
//    ++current_path_index; // We should be at the next path index after sleep time
//  }
//
//  if (consecutive_planning_failures >= max_consecutive_planning_failures) {
//    ROS_ERROR_STREAM("Continuous planning aborted. Maximum consecutive planning failures reached (" << max_consecutive_planning_failures << ").");
//    return false;
//  }
//
//  return true;
//}

nav_msgs::Path WholeBodyPlanningServer::extractPathFromTrajectories(const std::vector<robot_trajectory::RobotTrajectory>& trajectories, const ros::Time& start_time) const
{
  nav_msgs::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = ros::Time::now();
  ros::Time next_start_time = start_time;
  for (const auto & trajectory : trajectories) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = next_start_time;
    Eigen::Affine3d goal_pose = trajectory.getFirstWayPoint().getJointTransform("world_virtual_joint");
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    path.poses.push_back(pose_msg);

    next_start_time += ros::Duration(trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount()-1));
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
  planAndExecutePath(*path_msg, auto_start_);
}

void WholeBodyPlanningServer::confirmExecutionCb(const std_msgs::EmptyConstPtr&)
{
  ros::Time starting_time = ros::Time::now() + ros::Duration(1);
  if (!executeTrajectory(trajectory_, starting_time)) {
    ROS_ERROR_STREAM("Trajectory execution failed");
  }
}

void WholeBodyPlanningServer::jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  bool state_changed = false;
  for (unsigned int joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), joint_names_[joint_idx]);
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
  ROS_INFO_STREAM("Follow path client finished.");
  // If successful, wait for moveit to finish
  if (result->result.val == move_base_lite_msgs::ErrorCodes::SUCCESS) {
    setFollowPathGoalSuccessful();
  } else {
    // Path following failed
    // Stop moveit
    moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();

    if (followPathServerIsActive()) {
      // Abort goal
      move_base_lite_msgs::FollowPathResult result_server;
      result_server.result.val = result->result.val;
      follow_path_goal_.setCanceled(result_server, "Following path failed");
    }
  }
}
void WholeBodyPlanningServer::setFollowPathGoalSuccessful()
{
  if (followPathServerIsActive()) {
    move_base_lite_msgs::FollowPathResult result_server;
    result_server.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
    follow_path_goal_.setSucceeded(result_server, "successful");
  }
}

void WholeBodyPlanningServer::dynamicReconfigureCallback(const whole_body_planning_server::PlanningServerConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure request received.");
  enabled_ = config.enabled;
  path_sampling_resolution_ = config.path_sampling_resolution;
  path_slice_length_ = config.path_slice_length;
  auto_start_ = config.auto_start;
}

}
