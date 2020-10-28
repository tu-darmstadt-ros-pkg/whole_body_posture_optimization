#ifndef WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H
#define WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H

#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_lite_msgs/FollowPathAction.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <voxblox/core/common.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_msgs/GetLayer.h>
#include <voxblox/core/tsdf_map.h>

#include <sdf_contact_estimation/body_shapes/robot_shape_generator_base.h>

// Forward declarations
namespace whole_body_posture_optimization {
  class WholeBodyPostureOptimization;
}

typedef actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> FollowPathClient;
typedef actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction> FollowPathServer;
typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ExecuteTrajectoryClient;
typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;

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
  bool generatePlan(const nav_msgs::Path& path_msg, std::vector<robot_trajectory::RobotTrajectory>& trajectory_out);
  bool executeTrajectory(const std::vector<robot_trajectory::RobotTrajectory>& trajectory);
  /**
   * @brief planAndMoveToState Move to goal joint state and robot base position
   * @param goal_state
   * @param jmg
   * @return
   */
  bool planAndMoveToState(const robot_state::RobotState& goal_state, const moveit::core::JointModelGroup *jmg);
  bool updateTsdf();

  bool continuousReplanning(const nav_msgs::Path& path_msg);

  // Util
  nav_msgs::Path extractPathFromTrajectories(const std::vector<robot_trajectory::RobotTrajectory> &trajectories, const ros::Time& start_time) const;
  Eigen::Vector3d projectOntoPath(const nav_msgs::Path& path, geometry_msgs::Point& point);
  std::vector<double> computeAccumulatedDistances(const nav_msgs::Path& path) const;

  // Callbacks
  void pathCb(const nav_msgs::PathConstPtr& path_msg);
  void confirmExecutionCb(const std_msgs::EmptyConstPtr&);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  void followPathClientFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr& feedback_ptr);
  void followPathServerExecuteCb(const move_base_lite_msgs::FollowPathGoalConstPtr& goal_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_states_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_;
  std::shared_ptr<voxblox::TsdfMap> loaded_tsdf_;
  std::vector<robot_trajectory::RobotTrajectory> trajectory_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string move_group_;
  bool auto_start_;
  bool continuous_replanning_;
  std::string stability_info_save_folder_;
  double path_sampling_resolution_;

  // Input
  ros::Subscriber path_sub_;
  ros::Subscriber execute_sub_;
  ros::ServiceClient get_tsdf_client_;
  std::shared_ptr<FollowPathServer> follow_path_as_;
  ros::Subscriber joint_state_sub_;

  // Output
  std::shared_ptr<FollowPathClient> driving_client_;
  std::shared_ptr<MoveGroupClient> move_group_client_;
  std::shared_ptr<ExecuteTrajectoryClient> execute_trajectory_client_;


};

}

#endif
