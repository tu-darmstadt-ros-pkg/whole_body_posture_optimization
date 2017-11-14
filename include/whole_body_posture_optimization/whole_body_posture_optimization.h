#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__WHOLE_BODY_POSTURE_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__WHOLE_BODY_POSTURE_OPTIMIZATION_H

#include <nav_msgs/Path.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <whole_body_posture_optimization/arm_optimization.h>

namespace whole_body_posture_optimization {
class WholeBodyPostureOptimization {
public:
  WholeBodyPostureOptimization(const std::string& robot_description, const sdf_contact_estimation::RobotShape& robot_shape);
  bool loadOptimizationParameters(ros::NodeHandle& nh); // set free parameters, etc
  void updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  void updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void setDefaultJointPositions(const std::vector<double>& default_joint_positions);
  std::vector<std::vector<double>> findOptimalPostures(const nav_msgs::Path& path);
  std::vector<double> findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double> &joint_state);
private:
  sdf_contact_estimation::SDFContactEstimation contact_estimation_;
  ArmOptimization arm_optimization_;
  std::vector<double> default_joint_state_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_;

};
}

#endif
