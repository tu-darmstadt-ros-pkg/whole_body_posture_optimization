#include <whole_body_posture_optimization/arm/stability_cost.h>

namespace whole_body_posture_optimization {
StabilityCost::StabilityCost(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                               const std::vector<double>& full_state, const std::vector<double> &previous_param_angles, const Eigen::Isometry3d &robot_pose,
                               const StabilityCostParameters &parameters)
  : full_state_(full_state), robot_model_(robot_model), contact_points_(contact_points), robot_pose_(robot_pose), previous_param_angles_(previous_param_angles),
    parameters_(parameters)
{
  param_idx_to_joint_idx_ = robot_model_->getActiveJointIndices(free_joints);

  if (contact_points.empty()) {
    ROS_ERROR_STREAM("Creating stability error with empty contact points list.");
  }
}

ceres::CostFunction* StabilityCost::Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                                            const std::vector<double>& full_state, const std::vector<double> &previous_param_angles, const Eigen::Isometry3d &robot_pose,
                                            const StabilityCostParameters &parameters)
{
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<StabilityCost, 4>(
        new StabilityCost(robot_model, free_joints, contact_points, full_state, previous_param_angles, robot_pose, parameters));
  cost_function->AddParameterBlock(free_joints.size());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
