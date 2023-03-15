#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H

#include <nav_msgs/Path.h>

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <whole_body_posture_optimization/arm/arm_optimization.h>
#include <whole_body_posture_optimization/flipper/flipper_optimization.h>
#include <whole_body_posture_optimization/robot_model.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

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

  robot_state::RobotStatePtr result_state;

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
  WholeBodyPostureOptimization(const ros::NodeHandle &nh,
                               const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                               const sdf_contact_estimation::SdfModelPtr& sdf_model);
  bool loadParameters(const ros::NodeHandle &nh); // set free parameters, etc

  std::shared_ptr<RobotModel> getRobotModel() const;

  void setDefaultJointPositions(const std::vector<double>& default_joint_positions);
  const std::vector<double>& getDefaultJointPositions();

  robot_trajectory::RobotTrajectory computeOptimalTrajectory(const nav_msgs::Path& path, const robot_state::RobotStateConstPtr& start_state=robot_state::RobotStateConstPtr());
  //  robot_trajectory::RobotTrajectory computeOptimalTrajectory(const nav_msgs::Path& path);

  std::vector<PostureOptimizationResult> findOptimalPostures(const nav_msgs::Path& path, const robot_state::RobotStateConstPtr& start_state=robot_state::RobotStateConstPtr());
  PostureOptimizationResult findOptimalPosture(const Eigen::Isometry3d& pose, const std::vector<double> &init_joint_angles, const robot_state::RobotStateConstPtr& previous_state=robot_state::RobotStateConstPtr());
  PostureOptimizationResult findOptimalPostureSplit(const Eigen::Isometry3d& pose, const std::vector<double> &init_joint_angles, const robot_state::RobotStateConstPtr& previous_state=robot_state::RobotStateConstPtr());

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
  /**
   * Adds time parameterization to a trajectory with virtual world link with fake odom poses.
   * @param trajectory_fake_odom Trajectory with fake odom poses
   * @param trajectory_world_poses Base link poses in world frame that are used for the final trajectory
   * @return
   */
  robot_trajectory::RobotTrajectory addTimeParameterization(robot_trajectory::RobotTrajectory& trajectory_fake_odom,
                                                      const std::vector<Eigen::Isometry3d>& trajectory_world_poses);

  void publishTrajectory(const robot_trajectory::RobotTrajectory &robot_trajectory, const ros::Time& start_time=ros::Time(0));

  static void printResultInfo(const std::vector<PostureOptimizationResult>& results);
//  static void saveResultInfo(const std::vector<PostureOptimizationResult>& results, const std::vector<robot_trajectory::RobotTrajectory>& trajectories, std::string save_path);
private:
  ros::NodeHandle nh_;
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  sdf_contact_estimation::SdfModelPtr sdf_model_;
//  std::shared_ptr<CombinedOptimization> combined_optimization_;
  std::shared_ptr<ArmOptimization> arm_optimization_;
  std::shared_ptr<FlipperOptimization> flipper_optimization_;
  std::shared_ptr<RobotModel> robot_model_;

  // Parameters
  OptimizationType optimization_type_;
  std::vector<double> default_joint_state_;
  double stability_limit_;
  double front_tip_support_area_limit_;
  std::string complete_robot_group_;
  std::string complete_robot_with_world_group_;
  std::string trajectory_planner_name_;

  bool stepping_;
  int stepping_start_;

  ros::Publisher trajectory_pub_;
  ros::Publisher sdf_pub_;
  ros::Publisher path_pub_;

  std::string sdf_save_folder_;
};
}

#endif
