#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__DRIVING_CONTROLLER_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__DRIVING_CONTROLLER_H

#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <move_base_lite_msgs/FollowPathAction.h>

#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/kinematic_constraints/utils.h>

#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>


namespace whole_body_posture_optimization {

class DrivingController {
public:
  DrivingController(const ros::NodeHandle& nh);

  void pathCb(const nav_msgs::PathConstPtr& path_msg);
  void tsdfCb(const voxblox_msgs::Layer& layer_msg);
  void executeCb(const std_msgs::EmptyConstPtr& empty_msg);
private:
  bool executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectory);
  bool planAndMove(const robot_state::RobotState& goal_state, const moveit::core::JointModelGroup *jmg);

  ros::NodeHandle nh_;

  ros::Subscriber path_sub_;
  ros::Subscriber tsdf_sub_;
  ros::Subscriber execute_sub_;

  std::shared_ptr<voxblox::TsdfMap> tsdf_;
  bool received_first_tsdf_update_;
  bool auto_start_;
  std::vector<robot_trajectory::RobotTrajectory> trajectory_;
  sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

  std::shared_ptr<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>> driving_client_;
  std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>> move_group_client_;
  std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> execute_trajectory_client_;
};

}

#endif
