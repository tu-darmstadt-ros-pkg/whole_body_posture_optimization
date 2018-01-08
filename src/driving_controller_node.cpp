#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <nav_msgs/Path.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_lite_msgs/FollowPathAction.h>

#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

#include <eigen_conversions/eigen_msg.h>

sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape_;
std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

ros::ServiceClient execute_trajectory_client_;
std::shared_ptr<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>> driving_client_;

void executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectory) {
  // Plan and execute from current to start state

  // Drive to starting pose

  // Execute arm motion
  if (trajectory.empty()) {
    ROS_WARN_STREAM("[executeTrajectory] Trajectory was empty.");
    return;
  }
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
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = next_start_time;
  Eigen::Affine3d goal_pose = trajectory[trajectory.size()-1].getLastWayPoint().getJointTransform("world_virtual_joint");
  tf::poseEigenToMsg(goal_pose, pose_msg.pose);
  path.poses.push_back(pose_msg);


  move_base_lite_msgs::FollowPathGoal goal;
  goal.target_path = path;
  goal.follow_path_options.rotate_front_to_goal_pose_orientation = true;

  driving_client_->sendGoal(goal);
  if (!driving_client_->waitForResult()) {
    ROS_INFO_STREAM("FollowPath action failed.");
  }
}

void pathCb(const nav_msgs::PathConstPtr& path_msg) {
  auto result = optimizer_->findOptimalPostures(*path_msg);
  std::vector<robot_trajectory::RobotTrajectory> trajectory = optimizer_->computeTimedTrajectory(result);
  executeTrajectory(trajectory);
}

void sdfCb() {

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "driving_controller_node");
  ros::NodeHandle pnh("~");

  // Create robot shape
  tracker_shape_ = std::make_shared<argo_tracker::ArgoTrackerShape>();
  optimizer_= std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh, tracker_shape_);

  
  execute_trajectory_client_ = pnh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
  if (!execute_trajectory_client_.waitForExistence(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for '/execute_kinematic_path' service.");
    return 0;
  }

  driving_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>>("/controller/follow_path", true);
  if (!driving_client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR_STREAM("Timed out waiting for driving action client.");
    return 0;
  }

  ros::Subscriber path_sub = pnh.subscribe("path_topic", 10, &pathCb);
//  ros::Subscriber sdf_sub = pnh.subscribe("sdf_topic", 10, &sdfCb);
  
  
  
  ros::spin();

  return 0;
}
