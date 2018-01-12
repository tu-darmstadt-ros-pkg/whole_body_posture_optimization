#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                         const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles,
                                         const sdf_contact_estimation::ContactEstimationResult &init_contact)
  : parameter_size_(parameter_size), contact_estimation_(contact_estimation), pose_(pose), base_to_com_(base_to_com), init_contact_(init_contact), prev_flipper_angles_(prev_flipper_angles)
{}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + parameter_size_);
  contact_estimation_->updateJointStates(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_, init_contact_);
  if (result.convex_hull_points.empty()) {
    return false;
  }


  // Robust regularization
  double regularization_weight = 0.5;
  double regularization_cost = 0.0;
  for (unsigned int idx = 0; idx < prev_flipper_angles_.size(); idx++) {
    double diff = prev_flipper_angles_[idx] - flipper_angles[0][idx];
    regularization_cost += lorentzian<double>(diff, 0.6);
//    regularisation_cost += pow(diff, 2);
  }
//  ROS_INFO_STREAM("Regularization cost: " << regularization_cost);

  double track_contact_weight = 2;
  double track_contact_pct = static_cast<double>(result.contact_track_count)/result.total_track_count;
  double body_contact_weight = 3;
  double body_contact_pct = static_cast<double>(result.contact_body_count)/result.total_body_count;
  cost[0] = -result.support_area - track_contact_weight * track_contact_pct + body_contact_weight * body_contact_pct + regularization_weight * regularization_cost;

//  Eigen::Affine3d com_pose = result.estimated_pose * base_to_com_;
//  std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(result.convex_hull_points, com_pose.translation());
//  std::vector<double> stabilities_transformed(stabilities.size());
//  std::transform(stabilities.begin(), stabilities.end(), stabilities_transformed.begin(), [](double stability) -> double { return 1.0/(pow(std::max(stability, 0.01), 2)); });
//  double stability_transformed_sum = std::accumulate(stabilities_transformed.begin(), stabilities_transformed.end(), 0.0);
////  double stability_min = *std::min_element(stabilities.begin(), stabilities.end());
////  double stability_sum = std::accumulate(begin(stabilities), end(stabilities), 0.0);
////  ROS_INFO_STREAM("Stability: " << stability_min);
//  cost[0] = stability_transformed_sum + regularization_weight * regularization_cost;
  return true;
}

ceres::CostFunction *ConvexHullAreaError::Create(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation,
                                                 const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles, const sdf_contact_estimation::ContactEstimationResult &init_contact)
{
  ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
        new ConvexHullAreaError(parameter_size, contact_estimation, pose, base_to_com, prev_flipper_angles, init_contact));

  cost_function->AddParameterBlock(parameter_size);
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
