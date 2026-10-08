#ifndef WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H
#define WHOLE_BODY_PLANNING_SERVER_WHOLE_BODY_PLANNING_SERVER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav2_msgs/action/follow_path.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include <voxblox_ros/esdf_server.h>

// Forward declarations
namespace whole_body_posture_optimization {
  class WholeBodyPostureOptimization;
  struct PostureOptimizationResult;
}

namespace sdf_contact_estimation {
  class SdfModel;
}

namespace whole_body_planning_server {

using FollowPath = nav2_msgs::action::FollowPath;
using FollowPathServerGoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
using FollowPathClientGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class WholeBodyPlanningServer {
public:
  explicit WholeBodyPlanningServer(const rclcpp::Node::SharedPtr& node);
  ~WholeBodyPlanningServer();
  bool loadTsdfFromPath(const std::string& tsdf_path);
  /**
   * @brief planAndExecutePathFull Plan and execute a whole-body plan along the given path
   * @param path_msg Path to drive
   */
  bool planAndExecutePath(nav_msgs::msg::Path path_msg, bool replanning= false);
  bool planAndExecutePathFull(const nav_msgs::msg::Path& path_msg, bool replanning= false);
  void planAndExecutePathContinuous(const nav_msgs::msg::Path& path_msg, bool replanning=false);
private:
  void loadParameters();
  void updateRobotPose(const nav_msgs::msg::Path& path);
  bool getRobotPose(const rclcpp::Time& stamp, geometry_msgs::msg::TransformStamped& transform) const;
  nav_msgs::msg::Path preprocessPath(const nav_msgs::msg::Path& path, double path_sampling_resolution) const;
  bool generatePlan(const nav_msgs::msg::Path& path_msg, robot_trajectory::RobotTrajectoryPtr& trajectory_out);

  bool executeTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time);
  bool executeTrajectoryWithWatchdog(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time);
  bool executeJointTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time);
  void executeBaseTrajectory(const robot_trajectory::RobotTrajectory& trajectory, rclcpp::Time start_time=rclcpp::Time(0, 0u, RCL_ROS_TIME));
  void executeBaseTrajectory(const nav_msgs::msg::Path& path);
  void stopExecution();

  void stopExecutionThread();

  /**
   * @brief Continuously compare goal state and current state and stop execution if the deviation exceeds the threshold
   */
  void executionWatchDog();

  /// Util
  bool updatePath(nav_msgs::msg::Path& old_path, const nav_msgs::msg::Path& new_path) const;
  nav_msgs::msg::Path extractPathFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory, const rclcpp::Time& start_time) const;
  bool followPathServerIsActive() const;
  void abortActiveGoal(const std::string& error_msg, uint16_t error_code);
  void saveTrajectoryStatistics(
      const std::vector<whole_body_posture_optimization::PostureOptimizationResult> &results,
      robot_trajectory::RobotTrajectory &trajectory,
      const std::vector<size_t> &trajectory_start_indices,
      const rclcpp::Time &start_time);
  void waitUntilPlanningHorizonReached(const geometry_msgs::msg::Point& planning_point);

  /// Callbacks
  ///! Path following action server interface
  rclcpp_action::GoalResponse followPathGoalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse followPathCancelCallback(const std::shared_ptr<FollowPathServerGoalHandle>& goal_handle);
  void followPathAcceptedCallback(const std::shared_ptr<FollowPathServerGoalHandle>& goal_handle);

  ///! Simple path goal via topic
  void pathCb(const nav_msgs::msg::Path::ConstSharedPtr& path_msg);
  void confirmExecutionCb(const std_msgs::msg::Empty::ConstSharedPtr&);
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg);

  ///! Follow path client
  void followPathClientDoneCallback(const FollowPathClientGoalHandle::WrappedResult& result);
  void followPathClientFeedbackCb(FollowPathClientGoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback>& feedback);

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  ///! Map updates
  void spinMapUpdate();
  void stopMapUpdateThread();
  void waitForMap();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr esdf_node_;

  std::shared_ptr<sdf_contact_estimation::SdfModel> sdf_model_;
  std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> optimizer_;

  moveit::core::RobotStatePtr current_robot_state_;
  mutable std::mutex current_robot_state_mutex_;

  nav_msgs::msg::Path goal_path_;
  mutable std::mutex path_update_mutex_;
  std::atomic<bool> path_updated_;
  std::atomic<bool> end_of_path_reached_;
  std::atomic<int> current_path_index_;

  robot_trajectory::RobotTrajectoryPtr planned_trajectory_; /// Planned but not yet executed trajectory

  robot_trajectory::RobotTrajectoryPtr trajectory_; /// Trajectory currently in execution
  rclcpp::Time trajectory_start_time_; /// Start time of trajectory in execution
  mutable std::recursive_mutex trajectory_mutex_;


  std::unique_ptr<std::thread> execution_thread_;
  std::unique_ptr<std::thread> execution_watchdog_thread_;
  std::atomic<bool> execution_completed_;

  std::unique_ptr<std::thread> goal_thread_;

  std::unique_ptr<std::thread> map_update_thread;
  std::atomic<bool> map_update_running_;


  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  mutable std::mutex esdf_update_mutex_;
  std::condition_variable first_esdf_update_condition_;

  // Parameters
  bool enabled_;
  std::string map_frame_;
  std::string base_frame_;
  bool auto_start_;
  double planning_horizon_;
  double minimum_path_length_;
  std::string stability_info_save_folder_;
  double path_sampling_resolution_;
  double path_stitching_tolerance_;
  bool override_path_time_;
  bool continuous_planning_;
  double execution_position_tolerance_;
  double execution_orientation_tolerance_;
  std::string trajectory_statistics_save_folder_;
  // Path following options (were part of the move_base_lite FollowPath goal; nav2_msgs/FollowPath
  // has no comparable options, so they are configured as parameters instead)
  bool reverse_allowed_;
  bool ignore_goal_orientation_;

  mutable std::mutex reconfigure_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preprocessed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr goal_path_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_deviation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr orientation_deviation_pub_;

  // action interface
  rclcpp_action::Server<FollowPath>::SharedPtr follow_path_server_;
  std::shared_ptr<FollowPathServerGoalHandle> follow_path_goal_;
  mutable std::mutex follow_path_goal_mutex_;

  // Clients
  rclcpp_action::Client<FollowPath>::SharedPtr driving_client_;
  /// Replanning replaces the driving goal; results of replaced goals are ignored
  rclcpp_action::GoalUUID driving_goal_id_{};
  int unanswered_driving_goals_ = 0;
  std::mutex driving_goal_mutex_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_client_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};

}

#endif
