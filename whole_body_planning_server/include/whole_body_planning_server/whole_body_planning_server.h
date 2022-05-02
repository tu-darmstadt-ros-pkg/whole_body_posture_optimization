#ifndef WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H
#define WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H

#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <move_base_lite_msgs/FollowPathAction.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <voxblox_ros/esdf_server.h>

#include <dynamic_reconfigure/server.h>
#include <whole_body_planning_server/PlanningServerConfig.h>

// Forward declarations
namespace whole_body_posture_optimization {
  class WholeBodyPostureOptimization;
}

namespace sdf_contact_estimation {
  class SdfModel;
}

typedef actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> FollowPathClient;

namespace whole_body_planning_server {

class WholeBodyPlanningServer {
public:
  WholeBodyPlanningServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool loadTsdfFromPath(std::string tsdf_path);
  /**
   * @brief planAndExecutePath Plan and execute a whole-body plan along the given path
   * @param path_msg Path to drive
   */
  bool planAndExecutePath(const nav_msgs::Path& path_msg, bool auto_start=true);
private:
  nav_msgs::Path preprocessPath(const nav_msgs::Path& path) const;
  nav_msgs::Path slicePath(const nav_msgs::Path& path, double distance) const;
  bool generatePlan(const nav_msgs::Path& path_msg, std::vector<robot_trajectory::RobotTrajectory>& trajectory_out);
//  bool moveToStartAndExecute(const std::vector<robot_trajectory::RobotTrajectory>& trajectory);
  bool executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectories, ros::Time starting_time);
  /**
   * @brief planAndMoveToState Move to goal joint state and robot base position
   * @param goal_state
   * @param jmg
   * @return
   */
//  bool planAndMoveToState(const robot_state::RobotState& goal_state, std::string group_name);
  bool executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, bool blocking=true, ros::Time start_time=ros::Time());

//  bool continuousReplanning(const nav_msgs::Path& path_msg);

  /// Util
  nav_msgs::Path extractPathFromTrajectories(const std::vector<robot_trajectory::RobotTrajectory> &trajectories, const ros::Time& start_time) const;
  std::string pathToString(const nav_msgs::Path& path) const;
  bool followPathServerIsActive() const;

  /// Callbacks
  ///! Path following action server interface
  void followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal);
  void followPathPreemptCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle preempt);
  void setFollowPathGoalSuccessful();

  ///! Simple path goal via topic
  void pathCb(const nav_msgs::PathConstPtr& path_msg);
  void confirmExecutionCb(const std_msgs::EmptyConstPtr&);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);

  ///! Follow path client
  void followPathClientDoneCallback(const actionlib::SimpleClientGoalState& state,
                                    const move_base_lite_msgs::FollowPathResultConstPtr& result);

  void dynamicReconfigureCallback(const whole_body_planning_server::PlanningServerConfig &config, uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool enabled_;

  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_states_;
  std::vector<robot_trajectory::RobotTrajectory> trajectory_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  voxblox::EsdfServer esdf_server_;

  // Parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string move_group_;
  bool auto_start_;
  double path_slice_length_;
  std::string stability_info_save_folder_;
  double path_sampling_resolution_;

  // Subscribers
  ros::Subscriber path_sub_;
  ros::Subscriber execute_sub_;
  ros::Subscriber joint_state_sub_;

  // action interface
  std::shared_ptr<actionlib::ActionServer<move_base_lite_msgs::FollowPathAction> > follow_path_server_;
  actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle follow_path_goal_;

  // Clients
  std::shared_ptr<FollowPathClient> driving_client_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<whole_body_planning_server::PlanningServerConfig> reconfigure_server_;
};

}

#endif
