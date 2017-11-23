#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &shape_gen,
                                         const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com, const sdf_contact_estimation::ContactEstimationResult &init_contact)
  : shape_gen_(shape_gen), contact_estimation_(contact_estimation), pose_(pose), base_to_com_(base_to_com), init_contact_(init_contact)
  {

}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + shape_gen_->getDof());
  contact_estimation_->updateJointStates(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_, init_contact_);
  if (result.convex_hull_points.empty()) {
    return false;
  }
  double track_contact_weight = 1;
  double track_contact_pct = static_cast<double>(result.contact_track_count)/result.total_track_count;
  double body_contact_weight = 3;
  double body_contact_pct = static_cast<double>(result.contact_body_count)/result.total_body_count;
  cost[0] = -result.support_area - track_contact_weight * track_contact_pct + body_contact_weight * body_contact_pct;

//  Eigen::Affine3d com_pose = result.estimated_pose * base_to_com_;
//  std::vector<double> stabilities = computeForceAngleStabilities<double>(result.convex_hull_points, Eigen::Vector3d(0, 0, -9.81), com_pose.translation());
//  double stability_min = *std::min_element(stabilities.begin(), stabilities.end());
//  double stability_sum = std::accumulate(begin(stabilities), end(stabilities), 0.0);
//  ROS_INFO_STREAM("Stability: " << stability_min);
//  cost[0] = -stability_min;
  return true;
}

ceres::CostFunction *ConvexHullAreaError::Create(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &shape_gen,
                                                 const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation, const Eigen::Affine3d &pose,
                                                 const Eigen::Affine3d &base_to_com, const sdf_contact_estimation::ContactEstimationResult &init_contact)
{
  ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
        new ConvexHullAreaError(shape_gen, contact_estimation, pose, base_to_com, init_contact));

  cost_function->AddParameterBlock(shape_gen->getDof());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
