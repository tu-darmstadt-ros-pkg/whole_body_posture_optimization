#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <nav_msgs/Path.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_lite_msgs/FollowPathAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/kinematic_constraints/utils.h>

#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

#include <eigen_conversions/eigen_msg.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

std::shared_ptr<voxblox::TsdfMap> tsdf_;
sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape_;
std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

ros::ServiceClient execute_trajectory_client_;
std::shared_ptr<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>> driving_client_;
std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>> move_group_client_;

void executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectory) {
  if (trajectory.empty()) {
    ROS_WARN_STREAM("[executeTrajectory] Trajectory was empty.");
    return;
  }

  // Drive to starting pose
  ROS_INFO_STREAM("Driving to starting pose");
  nav_msgs::Path start_path;
  start_path.header.frame_id = "world";
  start_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped start_pose_msg;
  start_pose_msg.header.frame_id = "world";
//  pose_msg.header.stamp = ros::Time::now();
  Eigen::Affine3d start_goal_pose = trajectory[0].getFirstWayPoint().getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(start_goal_pose, start_pose_msg.pose);
  start_path.poses.push_back(start_pose_msg);

  move_base_lite_msgs::FollowPathGoal start_follow_path_goal;
  start_follow_path_goal.target_path = start_path;
  start_follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;

  driving_client_->sendGoal(start_follow_path_goal);
  if (!driving_client_->waitForResult()) {
    ROS_INFO_STREAM("FollowPath action failed.");
  }


  // Plan and execute from current to start state
  moveit_msgs::MoveGroupGoal move_group_goal;
  move_group_goal.request.group_name = trajectory[0].getGroupName();
  move_group_goal.request.num_planning_attempts = 3;
  move_group_goal.request.allowed_planning_time = 5;
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(trajectory[0].getFirstWayPoint(), trajectory[0].getGroup());
  move_group_goal.request.goal_constraints.push_back(goal_constraint);

  move_group_goal.planning_options.plan_only = false;

  // all diffs?
//  move_group_goal.request.start_state.is_diff = true;
//  move_group_goal.planning_options.planning_scene_diff = true;

  ROS_INFO_STREAM("Moving arm to start state..");
  move_group_client_->sendGoal(move_group_goal);
  move_group_client_->waitForResult();

  ROS_INFO_STREAM("Executing planned trajectory");

  // Execute arm motion
  robot_trajectory::RobotTrajectory combined_trajectory = trajectory[0];
  for (unsigned int i = 1; i < trajectory.size(); i++) {
    combined_trajectory.append(trajectory[i], 0.0);
  }

  moveit_msgs::ExecuteKnownTrajectory execute_srv;
  execute_srv.request.wait_for_execution = true;

  moveit_msgs::RobotTrajectory trajectory_msg;
  combined_trajectory.getRobotTrajectoryMsg(trajectory_msg);
  execute_srv.request.trajectory = trajectory_msg;

  if (!execute_trajectory_client_.call(execute_srv)) {
    ROS_ERROR_STREAM("Failed to call trajectory execution service.");
  } else {
    if (execute_srv.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      ROS_ERROR_STREAM("Trajectory execution failed with error code: " << execute_srv.response.error_code.val);
    }
  }

  // Execute driving
  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  ros::Time next_start_time = ros::Time::now();
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

  driving_client_->sendGoal(follow_path_goal);
  if (!driving_client_->waitForResult()) {
    ROS_INFO_STREAM("FollowPath action failed.");
  }
}

void pathCb(const nav_msgs::PathConstPtr& path_msg) {
  optimizer_->updateWorldModel(tsdf_);
  auto result = optimizer_->findOptimalPostures(*path_msg);
  std::vector<robot_trajectory::RobotTrajectory> trajectory = optimizer_->computeTimedTrajectory(result);
  executeTrajectory(trajectory);
}

void sdfCb(const voxblox_msgs::Layer& layer_msg) {
  voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, tsdf_->getTsdfLayerPtr());
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "driving_controller_node");
  ros::NodeHandle pnh("~");

  // Create tsdf
  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(0.05);
  tsdf_.reset(new voxblox::TsdfMap(tsdf_config));

  // Create robot shape
  tracker_shape_ = std::make_shared<argo_tracker::ArgoTrackerShape>();
  optimizer_= std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh, tracker_shape_);

  // Connect trajectory execution service client
  execute_trajectory_client_ = pnh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
  if (!execute_trajectory_client_.waitForExistence(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for '/execute_kinematic_path' service.");
    return 0;
  }

  // Connect vehicle controller action client
  driving_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>>("/controller/follow_path", true);
  if (!driving_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for driving action client.");
    return 0;
  }

  // Connect vehicle controller action client
  move_group_client_ = std::make_shared<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>>("/move_group", true);
  if (!move_group_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for move_group action client.");
    return 0;
  }

  // Start subscribers
  ros::Subscriber path_sub = pnh.subscribe("path_command", 10, &pathCb);
  ros::Subscriber sdf_sub = pnh.subscribe("tsdf_map_out", 10, &sdfCb);
  
  ros::spin();

  return 0;
}
