#ifndef WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H
#define WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H

#include <mutex>
#include <condition_variable>
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
  ~WholeBodyPlanningServer();
  bool loadTsdfFromPath(const std::string& tsdf_path);
  /**
   * @brief planAndExecutePath Plan and execute a whole-body plan along the given path
   * @param path_msg Path to drive
   */
  bool planAndExecutePath(const nav_msgs::Path& path_msg, bool auto_start=true, bool replanning=false);
  void planAndExecutePathContinuous(const nav_msgs::Path& path_msg, bool replanning=false);
private:
  void updateRobotPose(const nav_msgs::Path& path);
  bool getRobotPose(const ros::Time& stamp, geometry_msgs::TransformStamped& transform) const;
  nav_msgs::Path preprocessPath(const nav_msgs::Path& path) const;
  bool generatePlan(const nav_msgs::Path& path_msg, robot_trajectory::RobotTrajectoryPtr& trajectory_out);

  bool executeTrajectory(const robot_trajectory::RobotTrajectory& trajectory, ros::Time start_time);
  bool executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, bool blocking=true, ros::Time start_time=ros::Time());
  void stopExecution();

  void stopExecutionThread();

  /**
   * @brief Continuously compare goal state and current state and stop execution if the deviation exceeds the threshold
   */
  void executionWatchDog();

  /// Util
  static nav_msgs::Path slicePath(const nav_msgs::Path& path, double distance);
  static nav_msgs::Path slicePath(const nav_msgs::Path& path, int start_index, int end_index);
  nav_msgs::Path extractPathFromTrajectories(const robot_trajectory::RobotTrajectory& trajectory, const ros::Time& start_time) const;
  static void enforceAccelerationBounds(const moveit::core::RobotStatePtr& robot_state_ptr);
  std::string pathToString(const nav_msgs::Path& path) const;
  bool followPathServerIsActive() const;

  /// Callbacks
  ///! Path following action server interface
  void followPathGoalCallback(actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle goal);
  void followPathPreemptCallback(const actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle& preempt);

  ///! Simple path goal via topic
  void pathCb(const nav_msgs::PathConstPtr& path_msg);
  void confirmExecutionCb(const std_msgs::EmptyConstPtr&);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);

  ///! Follow path client
  void followPathClientDoneCallback(const actionlib::SimpleClientGoalState& state,
                                    const move_base_lite_msgs::FollowPathResultConstPtr& result);

  void dynamicReconfigureCallback(const whole_body_planning_server::PlanningServerConfig &config, uint32_t level);

  ///! Map updates
  void spinMapUpdate();
  void stopMapUpdateThread();
  void waitForMap();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

  robot_state::RobotStatePtr current_robot_state_;
  mutable std::mutex current_robot_state_mutex_;

  robot_trajectory::RobotTrajectoryPtr planned_trajectory_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  ros::Time trajectory_start_time_;
  mutable std::mutex trajectory_mutex_;
  std::atomic<bool> end_of_path_reached_;

  std::unique_ptr<std::thread> execution_thread_;
  std::unique_ptr<std::thread> execution_watchdog_thread_;
  std::atomic<bool> execution_completed_;

  std::unique_ptr<std::thread> map_update_thread;
  std::atomic<bool> map_update_running_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  ros::CallbackQueue esdf_server_queue_;
  mutable std::mutex esdf_update_mutex_;
  std::condition_variable first_esdf_update_condition_;

  // Parameters
  bool enabled_;
  std::string map_frame_;
  std::string base_frame_;
  bool auto_start_;
  double planning_horizon_;
  std::string stability_info_save_folder_;
  double path_sampling_resolution_;
  bool override_path_time_;
  bool continuous_planning_;
  double execution_position_tolerance_;
  double execution_orientation_tolerance_;

  // Subscribers
  ros::Subscriber path_sub_;
  ros::Subscriber execute_sub_;
  ros::Subscriber joint_state_sub_;

  // Publishers
  ros::Publisher preprocessed_path_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher position_deviation_pub_;
  ros::Publisher orientation_deviation_pub_;

  // action interface
  std::shared_ptr<actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>> follow_path_server_;
  actionlib::ActionServer<move_base_lite_msgs::FollowPathAction>::GoalHandle follow_path_goal_;
  mutable std::mutex follow_path_goal_mutex_;

  // Clients
  std::shared_ptr<FollowPathClient> driving_client_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<whole_body_planning_server::PlanningServerConfig> reconfigure_server_;
  mutable std::mutex reconfigure_mutex_;
};

}

#endif
