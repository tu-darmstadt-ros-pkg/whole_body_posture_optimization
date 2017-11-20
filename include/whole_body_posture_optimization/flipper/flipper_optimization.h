#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__FLIPPER_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__FLIPPER_OPTIMIZATION_H

#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>
#include <whole_body_posture_optimization/ceres_wrapper.h>

namespace whole_body_posture_optimization {

class FlipperOptimization {
public:
  FlipperOptimization(const sdf_contact_estimation::RobotShapeGeneratorBasePtr& robot_shape_gen, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);
  std::vector<double> optimize(const Eigen::Affine3d& robot_pose, const Eigen::Affine3d& base_to_com, const std::vector<double> &init_position);
private:
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape_gen_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
};

}

#endif
