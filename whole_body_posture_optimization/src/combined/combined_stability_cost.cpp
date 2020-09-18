#include <whole_body_posture_optimization/combined/combined_stability_cost.h>

#include <algorithm>
#include <ceres/dynamic_numeric_diff_cost_function.h>

namespace whole_body_posture_optimization {

CombinedStabilityCost::CombinedStabilityCost(const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                             const std::vector<std::string>& free_joints_, const std::vector<std::string>& free_flipper_joints,
                                             const CombinedStabilityCostParameters& parameters, Eigen::Affine3d* result_robot_pose)
 : robot_model_(robot_model), contact_estimation_(contact_estimation), free_flipper_joints_(free_flipper_joints), free_joints_(free_joints_),
   parameters_(parameters), result_robot_pose_(result_robot_pose)
{
  param_idx_to_joint_idx_ = robot_model_->getActuatedJointIndices(free_joints_);
  if (free_flipper_joints.size() > free_joints_.size()) {
    ROS_ERROR_STREAM("Free flipper joints can't be larger than all free joints.");
  }
  if (parameters.lorentzian_sigma == 0) {
    ROS_ERROR_STREAM("lorentzian sigma can't be zero.");
  }
}

CombinedStabilityCost* CombinedStabilityCost::updateData(const std::vector<double>& full_joint_state, const std::vector<double>& previous_solution, const Eigen::Affine3d& robot_pose,
                                       const sdf_contact_estimation::ContactEstimationResult& init_contact)
{
  full_joint_state_ = full_joint_state;
  previous_solution_ = previous_solution;
  if (!previous_solution.empty() && previous_solution_.size() != free_joints_.size()) {
    ROS_ERROR_STREAM("Previous solution vector has different size than parameter vector.");
  }
  robot_pose_ = robot_pose;
  init_contact_ = init_contact;
  return this;
}

bool CombinedStabilityCost::operator()(double const* const* parameters, double* cost) const
{
  // Copy parameters into full joint state
  std::vector<double> joint_angles = full_joint_state_;
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
  }

  // Compute COM
  Eigen::Vector3d com_base;
  robot_model_->computeCenterOfMass(joint_angles, com_base);
//  ROS_INFO_STREAM("COM_base: " << com_base);

  // Compute contact
  std::vector<double> flipper_angles(parameters[0], parameters[0] + free_flipper_joints_.size());
  contact_estimation_->updateJointStates(flipper_angles);
  Eigen::Affine3d base_to_com; base_to_com = Eigen::Translation3d(com_base);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(robot_pose_, base_to_com, init_contact_);
  *result_robot_pose_ = result.estimated_pose;
//  ROS_INFO_STREAM("Setting robot pose to " << result.estimated_pose.affine());
  if (result.convex_hull_points.empty()) {
    return false;
  }

  // Compute stability
  Eigen::Vector3d com_world = result.estimated_pose * com_base;
//  ROS_INFO_STREAM("com_world: " << com_world);
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity(result.convex_hull_points, com_world);
//  for (unsigned int i = 0; i < stabilities.size(); i++) {
//    ROS_INFO_STREAM("Stability " << i << ": " << stabilities[i]);
//  }

  // Transform stability to cost
  std::transform(stabilities.begin(), stabilities.end(), stabilities.begin(), [this](double stability) -> double {
    return parameters_.exp_parameters[0] * exp(parameters_.exp_parameters[1]*stability+parameters_.exp_parameters[2]);
  });
//  for (unsigned int i = 0; i < stabilities.size(); i++) {
//    ROS_INFO_STREAM("Stability transformed" << i << ": " << stabilities[i]);
//  }

  // Sum up and normalize by number of support vector edges
  double stability_sum = std::accumulate(stabilities.begin(), stabilities.end(), 0.0);
//  ROS_INFO_STREAM("Stability sum: " << stability_sum);
  stability_sum = stability_sum / result.convex_hull_points.size(); // average cost per axis
//  ROS_INFO_STREAM("Normalized: " << stability_sum);

  // Compute regularization
  double regularization_cost = 0.0;
  if (!previous_solution_.empty()) {
    for (unsigned int idx = 0; idx < previous_solution_.size(); idx++) {
      double angle_diff = previous_solution_[idx] - parameters[0][idx];
      regularization_cost += lorentzian<double>(angle_diff, parameters_.lorentzian_sigma);
    }
  }
//  ROS_INFO_STREAM("regularization_cost: " << regularization_cost);

  cost[0] = stability_sum + parameters_.regularization_weight * regularization_cost;

  return true;
}

ceres::CostFunction* CombinedStabilityCost::Create(const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                                   const std::vector<std::string>& free_joints_, const std::vector<std::string>& free_flipper_joints,
                                                   const CombinedStabilityCostParameters& parameters, const std::vector<double>& initial_full_state,
                                                   const std::vector<double>& previous_solution, const Eigen::Affine3d& robot_pose,
                                                   const sdf_contact_estimation::ContactEstimationResult& init_contact, Eigen::Affine3d* result_robot_pose)
{
  CombinedStabilityCost* stability_cost = new CombinedStabilityCost(robot_model, contact_estimation, free_joints_, free_flipper_joints, parameters, result_robot_pose);
  ceres::DynamicNumericDiffCostFunction<CombinedStabilityCost>* cost_function = new ceres::DynamicNumericDiffCostFunction<CombinedStabilityCost>(
        stability_cost->updateData(initial_full_state, previous_solution, robot_pose, init_contact));

  cost_function->AddParameterBlock(free_joints_.size());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
