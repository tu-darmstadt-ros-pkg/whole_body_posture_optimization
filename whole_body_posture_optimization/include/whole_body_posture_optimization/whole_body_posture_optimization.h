#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <voxblox_msgs/msg/layer.hpp>

#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <whole_body_posture_optimization/arm/arm_optimization.h>
#include <whole_body_posture_optimization/flipper/flipper_optimization.h>
#include <whole_body_posture_optimization/robot_model.h>

#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace whole_body_posture_optimization {

enum OptimizationType {
  SPLIT,
  COMBINED
};

struct PostureOptimizationResult {
  Eigen::Isometry3d requested_pose;
  Eigen::Isometry3d result_pose;

  std::vector<double> init_joint_angles;
  std::vector<double> result_joint_angles;

  moveit::core::RobotStatePtr result_state;

  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;

  Eigen::Vector3d init_com;
  Eigen::Vector3d result_com;

  double initial_stability;
  double stability_flipper_opt;
  double stability_arm_opt;

  double flipper_cost;
  double arm_cost;

  bool success;
};

class WholeBodyPostureOptimization {
public:
  WholeBodyPostureOptimization(const rclcpp::Node::SharedPtr& node,
                               const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                               const sdf_contact_estimation::SdfModelPtr& sdf_model);
  bool loadParameters(const rclcpp::Node::SharedPtr& node); // set free parameters, etc

  std::shared_ptr<RobotModel> getRobotModel() const;

  const std::vector<double>& getDefaultJointPositions();

  robot_trajectory::RobotTrajectory computeOptimalTrajectory(const nav_msgs::msg::Path& path, const moveit::core::RobotStateConstPtr& start_state=moveit::core::RobotStateConstPtr());

  /**
   * Return an optimized joint state for each pose in the given path. If start_state is set, the first pose of the path
   * is assumed to be the current state and will not be optimized.
   * @param path Input path
   * @param start_state Optional start state
   * @return Optimized joint states for each pose of the path
   */
  std::vector<PostureOptimizationResult> findOptimalPostures(const nav_msgs::msg::Path& path, const moveit::core::RobotStateConstPtr& start_state=moveit::core::RobotStateConstPtr());
  PostureOptimizationResult findOptimalPosture(const Eigen::Isometry3d& pose,
                                               const std::vector<double> &init_joint_angles,
                                               const moveit::core::RobotStateConstPtr& previous_state=moveit::core::RobotStateConstPtr(),
                                               const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr());

  robot_trajectory::RobotTrajectory computeTimedTrajectory(const std::vector<PostureOptimizationResult>& opt_result);
  robot_trajectory::RobotTrajectory computeTimedTrajectory(const std::vector<PostureOptimizationResult>& opt_result,
                                                           std::vector<size_t>& trajectory_start_indices);
  /**
   * Takes the (sparse) optimization results and connects them via one planned self-collision-free trajectory without time parameterization.
   * The trajectory contains the virtual world link which is filled with fake odom messages.
   * @param opt_result Posture optimization results
   * @param trajectory_world_poses Interpolated base link poses in world frame for each waypoint of the returned trajectory
   * @param trajectory_start_indices Start index of the trajectory part of each input state
   * @return
   */
  robot_trajectory::RobotTrajectory computeTrajectory(const std::vector<PostureOptimizationResult>& opt_result,
                                                      std::vector<Eigen::Isometry3d>& trajectory_world_poses,
                                                      std::vector<size_t>& trajectory_start_indices);
  robot_trajectory::RobotTrajectory computeTrajectory(const std::vector<PostureOptimizationResult>& opt_result,
                                                      const Eigen::Vector2d& odom_start,
                                                      std::vector<Eigen::Isometry3d>& trajectory_world_poses,
                                                      std::vector<size_t>& trajectory_start_indices);
  robot_trajectory::RobotTrajectory computeTrajectory(const std::vector<moveit::core::RobotStatePtr>& states,
                                                      const Eigen::Vector2d& odom_start,
                                                      std::vector<Eigen::Isometry3d>& trajectory_world_poses,
                                                      std::vector<size_t>& trajectory_start_indices);
  /**
   * Adds time parameterization to a trajectory with virtual world link with fake odom poses.
   * @param trajectory_fake_odom Trajectory with fake odom poses
   * @param trajectory_world_poses Base link poses in world frame that are used for the final trajectory
   * @return
   */
  robot_trajectory::RobotTrajectory addTimeParameterization(robot_trajectory::RobotTrajectory& trajectory_fake_odom,
                                                      const std::vector<Eigen::Isometry3d>& trajectory_world_poses);

  void publishTrajectory(const robot_trajectory::RobotTrajectory &robot_trajectory, const rclcpp::Time& start_time=rclcpp::Time(0, 0u, RCL_ROS_TIME));

  static void printResultInfo(const std::vector<PostureOptimizationResult>& results);
  static bool saveTrajectoryStatistics(const std::vector<PostureOptimizationResult>& results, robot_trajectory::RobotTrajectory& trajectory, const std::vector<size_t>& trajectory_start_indices, const std::string& save_path, rclcpp::Time start_time=rclcpp::Time(0, 0u, RCL_ROS_TIME));
private:
  void publishTrajectoryOnPub(const rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr& pub, const robot_trajectory::RobotTrajectory &robot_trajectory, const rclcpp::Time& start_time=rclcpp::Time(0, 0u, RCL_ROS_TIME));

  rclcpp::Node::SharedPtr node_;
  std::string param_prefix_;
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  sdf_contact_estimation::SdfModelPtr sdf_model_;
  std::shared_ptr<ArmOptimization> arm_optimization_;
  std::shared_ptr<FlipperOptimization> flipper_optimization_;
  std::shared_ptr<RobotModel> robot_model_;

  // Trajectory planner plugin. The class loader MUST outlive all trajectories:
  // the waypoint states are allocated by code inside the planner plugin library,
  // so unloading it (loader destruction) leaves dangling vtables behind.
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;

  // Parameters
  OptimizationType optimization_type_;
  moveit::core::RobotStatePtr default_joint_state_;
  std::vector<double> default_joint_state_vec_;
  double stability_limit_;
  double front_tip_support_area_limit_;
  std::string complete_robot_group_;
  std::string complete_robot_with_world_group_;
  std::string trajectory_planner_name_;
  /// Scaling factors (0, 1] applied to the joint velocity/acceleration limits
  /// during time parameterization. Lower values result in slower trajectories.
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;

  bool stepping_;
  int stepping_start_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sparse_trajectory_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr sdf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::string sdf_save_folder_;
};
}

#endif
