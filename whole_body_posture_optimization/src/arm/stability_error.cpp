#include <whole_body_posture_optimization/arm/stability_error.h>

namespace whole_body_posture_optimization {

StabilityError::StabilityError(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                               const std::vector<double>& full_state, const std::vector<double> &previous_param_angles, const Eigen::Isometry3d &robot_pose,
                               const StabilityErrorParameters &parameters)
  : robot_model_(robot_model), contact_points_(contact_points), robot_pose_(robot_pose), previous_param_angles_(previous_param_angles),
    parameters_(parameters)
{
  param_idx_to_joint_idx_ = robot_model_->getActuatedJointIndices(free_joints); // TODO remove duplicate mapping calculation

  compute_com_ = std::make_shared<ceres::DynamicCostFunctionToFunctor>(new ComputeCenterOfMass(robot_model, full_state, param_idx_to_joint_idx_));
  if (contact_points.empty()) {
    ROS_ERROR_STREAM("Creating stability error with empty contact points list.");
  }
}

ceres::CostFunction* StabilityError::Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                                            const std::vector<double>& full_state, const std::vector<double> &previous_param_angles, const Eigen::Isometry3d &robot_pose,
                                            const StabilityErrorParameters &parameters)
{
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<StabilityError, 4>(
        new StabilityError(robot_model, free_joints, contact_points, full_state, previous_param_angles, robot_pose, parameters));
  cost_function->AddParameterBlock(free_joints.size());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

ComputeCenterOfMass::ComputeCenterOfMass(const std::shared_ptr<RobotModel>& robot_model, const std::vector<double>& full_state, const std::vector<std::size_t>& param_idx_to_joint_idx)
  : robot_model_(robot_model), full_state_(full_state), param_idx_to_joint_idx_(param_idx_to_joint_idx) {
  this->mutable_parameter_block_sizes()->push_back(static_cast<int>(param_idx_to_joint_idx.size()));
  this->set_num_residuals(3);
}

bool ComputeCenterOfMass::Evaluate(const double* const * parameters, double* residuals, double** jacobians) const {
  std::vector<double> joint_angles = full_state_;
  for (unsigned int i = 0; i < this->parameter_block_sizes()[0]; ++i) {
    joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
  }

  Eigen::Vector3d com;
  Eigen::MatrixXd ad_com;
  robot_model_->computeCenterOfMass(joint_angles, com, ad_com);

  for (unsigned int i = 0; i < 3; i++) {
    residuals[i] = com(i);
  }

  // Compute the Jacobian if asked for.
  if (jacobians != nullptr && jacobians[0] != nullptr) {
    for (unsigned int r = 0; r < 3; r++) {
      for (unsigned int p = 0; p < this->parameter_block_sizes()[0]; p++) {
        jacobians[0][r * this->parameter_block_sizes()[0] + p] = ad_com(r, param_idx_to_joint_idx_[p]);
      }
    }
  }

  return true;
}

}
