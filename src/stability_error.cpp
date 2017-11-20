#include <whole_body_posture_optimization/arm/stability_error.h>

namespace whole_body_posture_optimization {

StabilityError::StabilityError(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints,
                               const std::vector<Eigen::Vector3d> &contact_points, const std::vector<double>& initial_pose)
  : robot_model_(robot_model), compute_com_(new ComputeCenterOfMass(robot_model)), contact_points_(contact_points), initial_pose_(initial_pose)
{
  param_idx_to_joint_idx_ = robot_model_->getBodyIndices(free_joints); // TODO remove duplicate mapping calculation
  ROS_INFO_STREAM("Joint mappings");
  for (unsigned int i = 0; i < param_idx_to_joint_idx_.size(); i++) {
    ROS_INFO_STREAM("Free joint " << i << ": " << free_joints[i] << " -> id: " << param_idx_to_joint_idx_[i]);
  }
}

ceres::CostFunction* StabilityError::Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints,
                                            const std::vector<Eigen::Vector3d> &contact_points, const std::vector<double>& initial_pose)
{
  ceres::DynamicAutoDiffCostFunction<StabilityError, 4>* cost_function = new ceres::DynamicAutoDiffCostFunction<StabilityError, 4>(
        new StabilityError(robot_model, free_joints, contact_points, initial_pose));

  cost_function->AddParameterBlock(free_joints.size());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
