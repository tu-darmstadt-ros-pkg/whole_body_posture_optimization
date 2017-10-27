#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION
#define WHOLE_BODY_POSTURE_OPTIMIZATION

#include <nav_msgs/Path.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

namespace whole_body_posture_optimization {
class WholeBodyPostureOptimization {
public:
  WholeBodyPostureOptimization(const std::string& robot_description, const sdf_contact_estimation::RobotShape& robot_shape);
  void loadOptimizationParameters(ros::NodeHandle& ns); // set free parameters, etc
  void updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  void setDefaultJointPositions(const std::vector<double>& default_joint_positions);
  std::vector<std::vector<double>> findOptimalPostures(const nav_msgs::Path& path);
private:
  sdf_contact_estimation::SDFContactEstimation contact_estimation_;
  std::vector<double> default_joint_positions_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_;

};
}

#endif
