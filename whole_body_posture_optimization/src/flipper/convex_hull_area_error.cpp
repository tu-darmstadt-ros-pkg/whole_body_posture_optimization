#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

#include <whole_body_posture_optimization/common.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <algorithm>
#include <ceres/dynamic_numeric_diff_cost_function.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(const std::vector<std::string>& free_joints, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                         const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double>& init_flipper_angles,
                                         const std::vector<double> &prev_flipper_angles,
                                         const ConvexHullAreaErrorParameters &parameters)
  : free_joints_(free_joints), contact_estimation_(contact_estimation), pose_(pose), base_to_com_(base_to_com), init_flipper_angles_(init_flipper_angles),
    prev_flipper_param_angles_(prev_flipper_angles), parameters_(parameters)
{
  param_idx_to_flipper_state_ = contact_estimation_->getJointIndices(free_joints_);
}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  // *** Simulation ** //
  // Fill parameters into flipper positon vector
  std::vector<double> flipper_angles_vec = init_flipper_angles_;
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    flipper_angles_vec[param_idx_to_flipper_state_[i]] = flipper_angles[0][i];
  }
  // Simulate contact with given joint positions
  contact_estimation_->updateJointStates(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_);
  if (result.convex_hull_points.empty()) {
    cost[0] = std::numeric_limits<double>::max();
    return true;
  }

  // *** Cost functions ** //
  // Penalize deviation from requested position
  Eigen::Affine3d original_to_estimated = pose_.inverse(Eigen::Isometry) * result.estimated_pose;
  double position_difference = original_to_estimated.translation().block<2,1>(0,0).norm();
  double position_deviation_cost = std::max(0.0, position_difference - 0.02); // 2cm leeway

  // Joint movement cost
  double movement_cost = 0.0;
  if (!prev_flipper_param_angles_.empty()) {
    for (unsigned int idx = 0; idx < prev_flipper_param_angles_.size(); idx++) {
      double angle_difference = prev_flipper_param_angles_[idx] - flipper_angles[0][idx];
      movement_cost += lorentzian<double>(angle_difference, parameters_.lorentzian_sigma);
    }
    movement_cost /= static_cast<double>(prev_flipper_param_angles_.size()); // Normalize by number of joints
  }

  // Penalize chassis roll angle
  // We want the chassis to be upright
  double chassis_roll_angle = result.estimated_pose.linear().eulerAngles(0, 1, 2)[0];
  if (chassis_roll_angle > M_PI_2) {
    chassis_roll_angle = M_PI - chassis_roll_angle;
  }

  // Traction cost
  // Weight track contact by angle between contact normal and (inverse) gravity vector
  // Approximates normal force
  double track_contact_sum = 0;
  Eigen::Vector3d gravity_vector = Eigen::Vector3d::UnitZ();
  for (Eigen::Vector3d& contact_point: result.contact_points) {
    Eigen::Vector3d normal;
    contact_estimation_->getSdf().getDistanceAndGradient(contact_point, normal);
    if (normal == Eigen::Vector3d::Zero()) continue; // No gradient
    double angle = std::acos(gravity_vector.dot(normal)/(normal.norm()));
    double a = -2/M_PI;
    double b = 1;
    double v = std::max(a * angle + b, 0.0); // Value between 0 (low traction) and 1 (high traction)
    track_contact_sum += v;
  }
  // Reward track contact and penalize body contact
  double track_contact_pct = static_cast<double>(track_contact_sum)/result.total_track_count;
  double body_contact_pct = static_cast<double>(result.contact_body_count)/result.total_body_count;

  // Penalyze contact normals that point against the moving direction
  // For now, the moving direction is hard-coded to x+
//  Eigen::Vector3d moving_direction = Eigen::Vector3d::UnitX();
//  Eigen::Vector3d neg_moving_direction = -moving_direction;
//  double contact_against_moving_penalty_weight = 0.0;
//  double contact_against_moving_penalty_sum = 0;
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
      + parameters_.roll_cost_weight * chassis_roll_angle * chassis_roll_angle
      + parameters_.position_deviation_weight * position_deviation_cost;

  if (parameters_.verbose) {
    ROS_INFO_STREAM("Flipper cost function (weighted value in brackets): " << std::endl <<
    "Support area: " << result.support_area << std::endl <<
    "Track contact: " << track_contact_pct << " (" << parameters_.track_contact_weight * track_contact_pct << ")" << std::endl <<
    "Body contact: " << body_contact_pct << " (" << parameters_.body_contact_weight * body_contact_pct << ")" << std::endl <<
    "Movement penalty: " << movement_cost << " (" << parameters_.movement_cost_weight * movement_cost << ")" << std::endl <<
    "Position deviation cost: " << position_deviation_cost << " (" << parameters_.position_deviation_weight * position_deviation_cost << ")" << std::endl <<
    "Chassis roll angle: " << chassis_roll_angle << " (" << parameters_.roll_cost_weight * chassis_roll_angle << ")" << std::endl <<
    "Total cost: " << cost[0]);
  }

  return true;
}

ceres::CostFunction* ConvexHullAreaError::Create(const std::vector<std::string>& free_joints, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation,
                                                 const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const std::vector<double>& init_flipper_angles,
                                                 const std::vector<double> &prev_flipper_angles, const ConvexHullAreaErrorParameters &parameters)
{
  auto* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
      new ConvexHullAreaError(free_joints, contact_estimation, pose, base_to_com, init_flipper_angles, prev_flipper_angles, parameters));

  cost_function->AddParameterBlock(static_cast<int>(free_joints.size()));
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
