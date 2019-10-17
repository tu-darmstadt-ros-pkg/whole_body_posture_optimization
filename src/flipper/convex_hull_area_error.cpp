#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                         const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles,
                                         const sdf_contact_estimation::ContactEstimationResult &init_contact, const ConvexHullAreaErrorParameters &parameters)
  : parameter_size_(parameter_size), contact_estimation_(contact_estimation), prev_flipper_angles_(prev_flipper_angles), pose_(pose), base_to_com_(base_to_com),
    init_contact_(init_contact), parameters_(parameters)
{}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  // Simulate contact with given joint positions
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + parameter_size_);
  contact_estimation_->updateJointStates(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_, init_contact_);
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

  // Penalize base orientation
  // We want the chassis to be upright
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d up_world = result.estimated_pose.linear() * up;
  double inclination_weight = 0.0;
  double inclination_angle = std::acos(up.dot(up_world));

  // Reward track contact and penalize body contact
  double track_contact_pct = static_cast<double>(result.contact_track_count)/result.total_track_count;
  double body_contact_pct = static_cast<double>(result.contact_body_count)/result.total_body_count;

  // Penalyze contact normals that point against the moving direction
  // For now, the moving direction is hard-coded to x+
//  Eigen::Vector3d moving_direction = Eigen::Vector3d::UnitX();
//  Eigen::Vector3d neg_moving_direction = -moving_direction;
  double contact_against_moving_penalty_weight = 0.0;
  double contact_against_moving_penalty_sum = 0;
//  for (Eigen::Vector3d& contact_point: result.contact_points) {
//    Eigen::Vector3d normal;
//    contact_estimation_->getSdf().getDistanceAndGradient(contact_point, normal);
//    double angle = std::acos(neg_moving_direction.dot(normal)/(neg_moving_direction.norm() * normal.norm()));
//    double a = 1.85;
//    double c = -0.85;
//    double penalty = std::max(a * std::exp(-angle) + c, 0.0);
//    contact_against_moving_penalty_sum += penalty;
//  }

  // Sum up cost terms
  cost[0] = -result.support_area - parameters_.track_contact_weight * track_contact_pct
      + parameters_.body_contact_weight * body_contact_pct
      + parameters_.movement_cost_weight * movement_cost
      + inclination_weight * inclination_angle * inclination_angle
      + contact_against_moving_penalty_weight * contact_against_moving_penalty_sum;

  ROS_INFO_STREAM("Support area: " << result.support_area << std::endl
                  << "track contact: " << track_contact_pct << " (" << parameters_.track_contact_weight * track_contact_pct << ")" << std::endl
                  << "body contact: " << body_contact_pct << " (" << parameters_.body_contact_weight * body_contact_pct << ")" << std::endl
                  << "movement penalty: " << movement_cost << " (" << parameters_.movement_cost_weight * movement_cost << ")" << std::endl
                  << "inclination angle: " << inclination_angle << " (" << inclination_weight * inclination_angle * inclination_angle << ")" << std::endl
                  << "contact against moving: " << contact_against_moving_penalty_sum << "(" << contact_against_moving_penalty_weight * contact_against_moving_penalty_sum << ")");

  return true;
}

ceres::CostFunction* ConvexHullAreaError::Create(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation,
                                                 const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double> &prev_flipper_angles,
                                                 const sdf_contact_estimation::ContactEstimationResult &init_contact, const ConvexHullAreaErrorParameters &parameters)
{
  ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
        new ConvexHullAreaError(parameter_size, contact_estimation, pose, base_to_com, prev_flipper_angles, init_contact, parameters));

  cost_function->AddParameterBlock(static_cast<int>(parameter_size));
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
