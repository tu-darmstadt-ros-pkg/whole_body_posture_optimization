#include <whole_body_posture_optimization/flipper/flipper_stability_cost.h>

namespace whole_body_posture_optimization {

FlipperStabilityCost::FlipperStabilityCost(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                           const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles,
                                           const FlipperStabilityCostParameters &parameters)
  : parameter_size_(parameter_size), contact_estimation_(contact_estimation), prev_flipper_angles_(prev_flipper_angles), pose_(pose), base_to_com_(base_to_com),
    parameters_(parameters)
{}

bool FlipperStabilityCost::operator()(double const* const* flipper_angles, double* cost) const
{
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + parameter_size_);
  contact_estimation_->updateJointStates(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_);
  if (result.convex_hull_points.empty()) {
    return false;
  }

  // Penalize joint movement
  double movement_cost = 0.0;
  if (!prev_flipper_angles_.empty()) {
    for (unsigned int idx = 0; idx < prev_flipper_angles_.size(); idx++) {
      double diff = prev_flipper_angles_[idx] - flipper_angles[0][idx];
      movement_cost += lorentzian<double>(diff, parameters_.lorentzian_sigma);
    }
    movement_cost /= prev_flipper_angles_.size(); // Normalize by number of joints
  }

  Eigen::Affine3d com_pose = result.estimated_pose * base_to_com_;
  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(result.convex_hull_points, com_pose.translation());
//  for (unsigned int i = 0; i < stabilities.size(); i++) {
//    ROS_INFO_STREAM("Stability " << i << ": " << stabilities[i]);
//  }

  std::transform(stabilities.begin(), stabilities.end(), stabilities.begin(), [this](double stability) -> double {
    return parameters_.exp_parameters[0] * exp(parameters_.exp_parameters[1]*stability+parameters_.exp_parameters[2]);
  });
//  for (unsigned int i = 0; i < stabilities.size(); i++) {
//    ROS_INFO_STREAM("Stability transformed" << i << ": " << stabilities[i]);
//  }

  // Sum up and normalize by number of support vector edges
  double stability_sum = std::accumulate(stabilities.begin(), stabilities.end(), 0.0);
  stability_sum /= result.convex_hull_points.size(); // average cost per axis

  cost[0] = stability_sum + parameters_.movement_cost_weight * movement_cost;
  return true;
}

ceres::CostFunction* FlipperStabilityCost::Create(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation,
                                                 const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles, const FlipperStabilityCostParameters &parameters)
{
  ceres::DynamicNumericDiffCostFunction<FlipperStabilityCost>* cost_function = new ceres::DynamicNumericDiffCostFunction<FlipperStabilityCost>(
        new FlipperStabilityCost(parameter_size, contact_estimation, pose, base_to_com, prev_flipper_angles, parameters));

  cost_function->AddParameterBlock(static_cast<int>(parameter_size));
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
