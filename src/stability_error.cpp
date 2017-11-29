#include <whole_body_posture_optimization/arm/stability_error.h>

namespace whole_body_posture_optimization {

StabilityError::StabilityError(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints,
                               const std::vector<Eigen::Vector3d> &contact_points, const std::vector<double>& initial_pose, const Eigen::Affine3d &robot_pose)
  : robot_model_(robot_model), contact_points_(contact_points), initial_angles_(initial_pose), robot_pose_(robot_pose)
{
  param_idx_to_joint_idx_ = robot_model_->getJointIndices(free_joints); // TODO remove duplicate mapping calculation
  compute_com_ = std::make_shared<ceres::DynamicCostFunctionToFunctor>(new ComputeCenterOfMass(robot_model, initial_pose, param_idx_to_joint_idx_));
  if (contact_points.empty()) {
    ROS_ERROR_STREAM("Creating stability error with empty contact points list.");
  }
}

ceres::CostFunction* StabilityError::Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints,
                                            const std::vector<Eigen::Vector3d> &contact_points, const std::vector<double>& initial_pose, const Eigen::Affine3d &robot_pose)
{
  ceres::DynamicAutoDiffCostFunction<StabilityError, 4>* cost_function = new ceres::DynamicAutoDiffCostFunction<StabilityError, 4>(
        new StabilityError(robot_model, free_joints, contact_points, initial_pose, robot_pose));

  cost_function->AddParameterBlock(free_joints.size());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
