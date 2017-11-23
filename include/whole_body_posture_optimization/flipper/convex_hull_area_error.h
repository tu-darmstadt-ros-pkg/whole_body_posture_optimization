#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__CONVEX_HULL_AREA_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__CONVEX_HULL_AREA_ERROR_H

#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <sdf_contact_estimation/body_shapes/robot_shape_generator_base.h>
#include <whole_body_posture_optimization/arm/stability_error.h>
#include <algorithm>

namespace whole_body_posture_optimization {
class ConvexHullAreaError {
public:
  ConvexHullAreaError(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                      const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const sdf_contact_estimation::ContactEstimationResult &init_contact);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(unsigned int parameter_size,
                                     const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                     const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const sdf_contact_estimation::ContactEstimationResult &init_contact);
private:
  unsigned int parameter_size_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::vector<double> flipper_angles_;

  Eigen::Affine3d pose_;
  Eigen::Affine3d base_to_com_;
  sdf_contact_estimation::ContactEstimationResult init_contact_;

};
}

#endif
