#include <whole_body_posture_optimization/driving_controller.h>

namespace whole_body_posture_optimization {

DrivingController::DrivingController(const ros::NodeHandle &nh)
  : nh_(nh), received_first_tsdf_update_(false), auto_start_(false)
{
  // Create tsdf
  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(0.05);
  tsdf_.reset(new voxblox::TsdfMap(tsdf_config));

  // Create robot shape
  tracker_shape_ = std::make_shared<argo_tracker::ArgoTrackerShape>();
  optimizer_= std::make_shared<WholeBodyPostureOptimization>(nh_, tracker_shape_);

  // Connect trajectory execution service client
//  execute_trajectory_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
//  if (!execute_trajectory_client_.waitForExistence(ros::Duration(5))) {
//    ROS_ERROR_STREAM("Timed out waiting for '/execute_kinematic_path' service.");
//  }

  ROS_INFO_STREAM("Waiting for trajectory execution client..");
  execute_trajectory_client_ = std::make_shared<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>>(nh_, "/execute_trajectory", true);
  if (!execute_trajectory_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for execute_trajectory action client.");
  }

  // Connect vehicle controller action client
  ROS_INFO_STREAM("Waiting for vehicle controller client..");
  driving_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>>(nh_, "/controller/follow_path", true);
  if (!driving_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for driving action client.");
  }

  // Connect vehicle controller action client
  ROS_INFO_STREAM("Waiting for move_group client..");
  move_group_client_ = std::make_shared<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>>(nh_, "/move_group", true);
  if (!move_group_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for move_group action client.");
  }

  // Start subscribers
  path_sub_ = nh_.subscribe("path_command", 10, &DrivingController::pathCb, this);
  tsdf_sub_ = nh_.subscribe("tsdf", 10, &DrivingController::tsdfCb, this);
  execute_sub_ = nh_.subscribe("execute_planned_trajectory", 10, &DrivingController::executeCb, this);
}

bool DrivingController::executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory> &trajectory) {
  ROS_INFO_STREAM("Executing trajectory now.");
  if (trajectory.empty()) {
    ROS_WARN_STREAM("[executeTrajectory] Trajectory is empty.");
    return true;
  }

  // Drive to starting pose
  ROS_INFO_STREAM("Moving to starting pose");
  if (!planAndMove(trajectory[0].getFirstWayPoint(), trajectory[0].getGroup())) {
    ROS_ERROR_STREAM("Moving to initial state failed.");
    return false;
  }

  // Assemble arm motion plan
  robot_trajectory::RobotTrajectory combined_trajectory = trajectory[0];
  for (unsigned int i = 1; i < trajectory.size(); i++) {
    combined_trajectory.append(trajectory[i], 0.0);
  }

  moveit_msgs::ExecuteTrajectoryGoal execute_trajectory_goal;
  combined_trajectory.getRobotTrajectoryMsg(execute_trajectory_goal.trajectory);

  // Set starting time
  ros::Time starting_time = ros::Time::now() + ros::Duration(2);
  execute_trajectory_goal.trajectory.joint_trajectory.header.stamp = starting_time;

  // Assemble driving plan
  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  ros::Time next_start_time = starting_time;
  for (unsigned int i = 0; i < trajectory.size(); i++) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = next_start_time;
    Eigen::Affine3d goal_pose = trajectory[i].getFirstWayPoint().getJointTransform("world_virtual_joint");
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    path.poses.push_back(pose_msg);

    next_start_time += ros::Duration(trajectory[i].getWayPointDurationFromStart(trajectory[i].getWayPointCount()-1));
  }
  // add last pose
  geometry_msgs::PoseStamped end_pose_msg;
  end_pose_msg.header.frame_id = "world";
  end_pose_msg.header.stamp = next_start_time;
  Eigen::Affine3d end_goal_pose = trajectory[trajectory.size()-1].getLastWayPoint().getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(end_goal_pose, end_pose_msg.pose);
  path.poses.push_back(end_pose_msg);


  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;

  ROS_INFO_STREAM("Executing planned trajectory");

  // Execute
  execute_trajectory_client_->sendGoal(execute_trajectory_goal);
  driving_client_->sendGoal(follow_path_goal);

  // Wait for result
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
    ROS_INFO_STREAM("Trajectory execution failed with error code: " << execute_trajectory_result->error_code.val);
    return false;
  }

  return true;
}

bool DrivingController::planAndMove(const moveit::core::RobotState &goal_state, const robot_model::JointModelGroup* jmg)
{
  if (jmg != nullptr) {
    ROS_ERROR_STREAM("JointModelGroup is null.");
    return false;
  }

  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = "world";
  //  pose_msg.header.stamp = ros::Time::now();
  Eigen::Affine3d goal_pose = goal_state.getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(goal_pose, goal_pose_msg.pose);
  path.poses.push_back(goal_pose_msg);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;

  driving_client_->sendGoal(follow_path_goal);
  if (!driving_client_->waitForResult()) {
    ROS_ERROR_STREAM("FollowPath action failed.");
    return false;
  }

  // Plan and execute from current to start state
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
    ROS_ERROR_STREAM("MoveGroupGoal failed.");
    return false;
  }

  return true;
}

void DrivingController::pathCb(const nav_msgs::PathConstPtr &path_msg) {
  ROS_INFO_STREAM("Received new path, planning..");
  while (!received_first_tsdf_update_ && ros::ok()) {
    ROS_WARN_STREAM("Waiting for first tsdf update..");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  optimizer_->updateWorldModel(tsdf_);
  auto result = optimizer_->findOptimalPostures(*path_msg);
  trajectory_ = optimizer_->computeTimedTrajectory(result);
  if (auto_start_) {
    executeTrajectory(trajectory_);
  }
}

void DrivingController::tsdfCb(const voxblox_msgs::Layer &layer_msg) {
  received_first_tsdf_update_ = true;
  voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, tsdf_->getTsdfLayerPtr());
}

void DrivingController::executeCb(const std_msgs::EmptyConstPtr &empty_msg)
{
  executeTrajectory(trajectory_);
}

}
