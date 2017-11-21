#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &shape_gen,
                                         const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com)
  : shape_gen_(shape_gen), contact_estimation_(contact_estimation), pose_(pose), base_to_com_(base_to_com)
  {

}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + shape_gen_->getDof());
  shape_gen_->updateShape(flipper_angles_vec);
  sdf_contact_estimation::ContactEstimationResult result = contact_estimation_->estimateSupportPolygon(pose_, base_to_com_);
  double track_contact_weight = 0.4;
  double track_contact_pct = static_cast<double>(result.contact_track_count)/result.total_track_count;
  double body_contact_weight = 5;
  double body_contact_pct = static_cast<double>(result.contact_body_count)/result.total_body_count;
  cost[0] = -result.support_area - track_contact_weight * track_contact_pct + body_contact_weight * body_contact_pct;
  return true;
}

ceres::CostFunction *ConvexHullAreaError::Create(const sdf_contact_estimation::RobotShapeGeneratorBasePtr &shape_gen, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> &contact_estimation, const Eigen::Affine3d &pose, const Eigen::Affine3d &base_to_com)
{
  ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
        new ConvexHullAreaError(shape_gen, contact_estimation, pose, base_to_com));

  cost_function->AddParameterBlock(shape_gen->getDof());
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
