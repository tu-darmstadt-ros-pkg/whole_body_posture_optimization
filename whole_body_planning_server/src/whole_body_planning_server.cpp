#include <whole_body_planning_server/whole_body_planning_server.h>

#include <sdf_contact_estimation/body_shapes/robot_shape_from_urdf.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>


namespace whole_body_planning_server {

WholeBodyPlanningServer::WholeBodyPlanningServer(const ros::NodeHandle &nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), auto_start_(false)
{
  // Parameters
  pnh_.param("auto_start", auto_start_, false);
  pnh_.param<std::string>("stability_info_save_folder", stability_info_save_folder_, "");
  pnh_.param<std::string>("complete_robot_group", move_group_, "complete_robot_group");

  // Create whole body path planner
  robot_shape_ = std::make_shared<sdf_contact_estimation::RobotShapeFromUrdf>(pnh_);
  optimizer_= std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh_, robot_shape_);

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
  follow_path_as_ = std::make_shared<FollowPathServer>(pnh, "follow_path", boost::bind(&WholeBodyPlanningServer::followPathExecuteCb, this, _1), false);
  follow_path_as_->start();

  // Start subscribers
  path_sub_ = nh_.subscribe("path_command", 10, &WholeBodyPlanningServer::pathCb, this);
  execute_sub_ = nh_.subscribe("execute_planned_trajectory", 10, &WholeBodyPlanningServer::confirmExecutionCb, this);
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

  if (!updateTsdf(tsdf_)) {
    ROS_ERROR_STREAM("Failed to update world model.");
    return false;
  }

  optimizer_->updateWorldModel(tsdf_);
  auto result = optimizer_->findOptimalPostures(path_msg);
  trajectory_ = optimizer_->computeTimedTrajectory(result);
  if (trajectory_.size() < 1) {
    return false;
  }
  // Statistics
  whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
  optimizer_->printContactEstimationStatistics();
  if (stability_info_save_folder_ != "") {
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = stability_info_save_folder_ + "/stability_" + time_stamp + ".dat";
    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectory_, save_path);
  }

  // Execute
  if (auto_start) {
    return executeTrajectory(trajectory_);
  }
  return true;
}

bool WholeBodyPlanningServer::executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory> &trajectories) {
  ROS_INFO_STREAM("Executing trajectory now.");
  if (trajectories.empty()) {
    ROS_WARN_STREAM("[executeTrajectory] Trajectory is empty.");
    return true;
  }

  robot_trajectory::RobotTrajectory combined_trajectory(trajectories[0].getRobotModel(), move_group_);

  // Drive to starting pose
  ROS_INFO_STREAM("Moving to starting pose");
  if (!planAndMoveToGoal(trajectories[0].getFirstWayPoint(), combined_trajectory.getGroup())) {
    ROS_WARN_STREAM("Moving to initial state failed.");
    return false;
  }

  // Assemble arm motion plan
  for (unsigned int i = 0; i < trajectories.size(); i++) {
    combined_trajectory.append(trajectories[i], 0.000000001);
  }

  combined_trajectory.getRobotTrajectoryMsg(execute_trajectory_goal_.trajectory);

  // Set starting time
  starting_time_ = ros::Time::now() + ros::Duration(2);
  execute_trajectory_goal_.trajectory.joint_trajectory.header.stamp = starting_time_;

  // Assemble driving plan
  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  ros::Time next_start_time = starting_time_;
  for (unsigned int i = 0; i < trajectories.size(); i++) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = next_start_time;
    Eigen::Affine3d goal_pose = trajectories[i].getFirstWayPoint().getJointTransform("world_virtual_joint");
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    path.poses.push_back(pose_msg);

    next_start_time += ros::Duration(trajectories[i].getWayPointDurationFromStart(trajectories[i].getWayPointCount()-1));
  }
  // add last pose
  geometry_msgs::PoseStamped end_pose_msg;
  end_pose_msg.header.frame_id = "world";
  end_pose_msg.header.stamp = next_start_time;
  Eigen::Affine3d end_goal_pose = trajectories.back().getLastWayPoint().getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(end_goal_pose, end_pose_msg.pose);
  path.poses.push_back(end_pose_msg);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path = path;
  follow_path_goal.follow_path_options.desired_speed = 0.0;
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;
  follow_path_goal.follow_path_options.reverse_allowed = true;
  follow_path_goal.follow_path_options.is_fixed = true;
  follow_path_goal.follow_path_options.goal_pose_position_tolerance = 0.01;
  follow_path_goal.follow_path_options.goal_pose_angle_tolerance = 0.01;

  ROS_INFO_STREAM("Executing planned trajectory");

  // Execute
  execute_trajectory_client_->sendGoal(execute_trajectory_goal_);
  driving_client_->sendGoal(follow_path_goal, FollowPathClient::SimpleDoneCallback(), FollowPathClient::SimpleActiveCallback(),
                            boost::bind(&WholeBodyPlanningServer::followPathFeedbackCb, this, _1));

  ros::Duration time_until_start = starting_time_ - ros::Time::now();
  time_until_start.sleep();
  optimizer_->publishTrajectory(trajectories);

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

void WholeBodyPlanningServer::followPathFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr &feedback_ptr)
{
}

void WholeBodyPlanningServer::followPathExecuteCb(const move_base_lite_msgs::FollowPathGoalConstPtr& goal_ptr)
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

bool WholeBodyPlanningServer::planAndMoveToGoal(const moveit::core::RobotState &goal_state, const robot_model::JointModelGroup* jmg)
{
  if (jmg == nullptr) {
    ROS_ERROR_STREAM("JointModelGroup is null.");
    return false;
  }

  // Drive to pose in goal state
  geometry_msgs::PoseStamped goal_pose_msg;
  goal_pose_msg.header.frame_id = "world";
  Eigen::Affine3d goal_pose = goal_state.getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(goal_pose, goal_pose_msg.pose);

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.target_path.header.frame_id = "world";
  follow_path_goal.target_path.header.stamp = ros::Time::now();
  follow_path_goal.target_path.poses.push_back(goal_pose_msg);
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

bool WholeBodyPlanningServer::updateTsdf(std::shared_ptr<voxblox::TsdfMap> &tsdf)
{
  if (loaded_tsdf_) {
    tsdf = loaded_tsdf_;
    return true;
  }

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
  voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(get_layer.response.layer, tsdf->getTsdfLayerPtr());
  return true;
}

void WholeBodyPlanningServer::pathCb(const nav_msgs::PathConstPtr &path_msg) {
  planAndExecutePath(*path_msg, auto_start_);
}

void WholeBodyPlanningServer::confirmExecutionCb(const std_msgs::EmptyConstPtr &empty_msg)
{
  executeTrajectory(trajectory_);
}

}
