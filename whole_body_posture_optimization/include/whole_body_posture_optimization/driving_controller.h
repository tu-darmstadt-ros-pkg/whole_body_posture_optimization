#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_DRIVING_CONTROLLER_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_DRIVING_CONTROLLER_H

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
#include <voxblox_msgs/GetLayer.h>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

typedef actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> FollowPathClient;
typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ExecuteTrajectoryClient;
typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;


namespace whole_body_posture_optimization {

class DrivingController {
public:
  DrivingController(const ros::NodeHandle& nh);
  bool loadTsdfFromPath(std::string tsdf_path);
private:
  void pathCb(const nav_msgs::PathConstPtr& path_msg);
  void executeCb(const std_msgs::EmptyConstPtr& empty_msg);

  bool executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectory);
  bool planAndMove(const robot_state::RobotState& goal_state, const moveit::core::JointModelGroup *jmg);
  bool updateTsdf(std::shared_ptr<voxblox::TsdfMap>& tsdf);

  void followPathFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr& feedback_ptr);

  ros::NodeHandle nh_;

  ros::Subscriber path_sub_;
  ros::Subscriber execute_sub_;

  std::shared_ptr<voxblox::TsdfMap> tsdf_;
  std::vector<robot_trajectory::RobotTrajectory> trajectory_;
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

  std::shared_ptr<voxblox::TsdfMap> loaded_tsdf_;
  ros::ServiceClient get_tsdf_client_;
  std::shared_ptr<FollowPathClient> driving_client_;
  std::shared_ptr<MoveGroupClient> move_group_client_;
  std::shared_ptr<ExecuteTrajectoryClient> execute_trajectory_client_;

  moveit_msgs::ExecuteTrajectoryGoal execute_trajectory_goal_;
  ros::Time starting_time_;

  // Parameters
  bool auto_start_;
  std::string stability_info_save_folder_;
  std::string move_group_;
};

}

#endif
