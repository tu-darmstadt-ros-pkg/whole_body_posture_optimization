#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__WHOLE_BODY_POSTURE_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__WHOLE_BODY_POSTURE_OPTIMIZATION_H

#include <nav_msgs/Path.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_posture_optimization/arm/arm_optimization.h>
#include <whole_body_posture_optimization/flipper/flipper_optimization.h>

namespace whole_body_posture_optimization {

struct PostureOptimizationResult {
  std::vector<double> init_joint_angles;
  std::vector<double> result_joint_angles;

  sdf_contact_estimation::ContactEstimationResult init_contact;
  sdf_contact_estimation::ContactEstimationResult result_contact;

  Eigen::Vector3d init_com;
  Eigen::Vector3d result_com;
};

class WholeBodyPostureOptimization {
public:
  WholeBodyPostureOptimization(const ros::NodeHandle &nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen);
  bool loadParameters(const ros::NodeHandle &nh); // set free parameters, etc
  void updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  void updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void setDefaultJointPositions(const std::vector<double>& default_joint_positions);
  const std::vector<double>& getDefaultJointPositions();
  std::vector<PostureOptimizationResult> findOptimalPostures(const nav_msgs::Path& path);
  PostureOptimizationResult findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double> &init_joint_state,
                                               const sdf_contact_estimation::ContactEstimationResult *init_contact_result);
  PostureOptimizationResult findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double> &init_joint_state);
private:
  ros::NodeHandle nh_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::shared_ptr<ArmOptimization> arm_optimization_;
  std::shared_ptr<FlipperOptimization> flipper_optimization_;
  std::shared_ptr<RobotModel> robot_model_;
  std::string collision_group_;

  std::string robot_description_;
  std::vector<double> default_joint_state_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_;
};
}

#endif
