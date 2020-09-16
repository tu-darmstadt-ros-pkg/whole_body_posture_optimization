#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_WHOLE_BODY_POSTURE_OPTIMIZATION_H

#include <nav_msgs/Path.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_posture_optimization/arm/arm_optimization.h>
#include <whole_body_posture_optimization/flipper/flipper_optimization.h>
#include <whole_body_posture_optimization/combined/combined_optimization.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace whole_body_posture_optimization {

enum OptimizationType {
  SPLIT,
  COMBINED
};

struct PostureOptimizationResult {
  Eigen::Affine3d requested_pose;
  std::vector<double> init_joint_angles;
  std::vector<double> result_joint_angles;

  sdf_contact_estimation::ContactEstimationResult init_contact;
  sdf_contact_estimation::ContactEstimationResult result_contact;

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
  WholeBodyPostureOptimization(const ros::NodeHandle &nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen);
  bool loadParameters(const ros::NodeHandle &nh); // set free parameters, etc
  void updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  void updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  bool loadSdfFromFile(std::string tsdf_file_path, float truncation_distance);
  bool saveSdfToFile(std::string save_folder);

  const std::shared_ptr<RobotModel> getRobotModel() const;


  void setDefaultJointPositions(const std::vector<double>& default_joint_positions);
  const std::vector<double>& getDefaultJointPositions();
  std::vector<PostureOptimizationResult> findOptimalPostures(const nav_msgs::Path& path);
  PostureOptimizationResult findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double> &init_joint_angles, const PostureOptimizationResult* previous_result=nullptr);
  PostureOptimizationResult findOptimalPostureSplit(const Eigen::Affine3d& pose, const std::vector<double> &init_joint_angles, const PostureOptimizationResult* previous_result=nullptr);
//  PostureOptimizationResult findOptimalPostureCombined(const Eigen::Affine3d& pose, const std::vector<double> &init_joint_angles, const std::vector<double> &previous_joint_angles,
//                                               const sdf_contact_estimation::ContactEstimationResult *init_contact_result);




  std::vector<robot_trajectory::RobotTrajectory> computeTimedTrajectory(const std::vector<PostureOptimizationResult>& opt_result);
  void publishTrajectory(const std::vector<robot_trajectory::RobotTrajectory> &robot_trajectory);

  void printContactEstimationStatistics();
  static void printResultInfo(const std::vector<PostureOptimizationResult>& results);
  static void saveResultInfo(const std::vector<PostureOptimizationResult>& results, const std::vector<robot_trajectory::RobotTrajectory>& trajectories, std::string save_path);
private:
  ros::NodeHandle nh_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::shared_ptr<CombinedOptimization> combined_optimization_;
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

  bool stepping_;
  int stepping_start_;

  ros::Publisher trajectory_pub_;
  ros::Publisher sdf_pub_;

  std::string sdf_save_folder_;
};
}

#endif
