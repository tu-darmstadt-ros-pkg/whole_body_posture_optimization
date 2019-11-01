#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H

#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <sdf_contact_estimation/body_shapes/robot_shape_generator_base.h>
#include <whole_body_posture_optimization/common.h>
#include <algorithm>

namespace whole_body_posture_optimization {

struct ConvexHullAreaErrorParameters {
  static ConvexHullAreaErrorParameters createFromServer(const ros::NodeHandle& nh) {
    ConvexHullAreaErrorParameters parameters_;
    nh.param("movement_cost_weight", parameters_.movement_cost_weight, 0.2);
    nh.param("lorentzian_sigma", parameters_.lorentzian_sigma, 0.2);
    nh.param("track_contact_weight", parameters_.track_contact_weight, 2.0);
    nh.param("body_contact_weight", parameters_.body_contact_weight, 4.0);
    nh.param("verbose", parameters_.verbose, false);
    return parameters_;
  }

  std::string toString() {
    std::stringstream ss;
    ss << "Heuristic cost parameters:" << std::endl;
    ss << "movement_cost_weight = " << movement_cost_weight << std::endl;
    ss << "lorentzian_sigma = " << lorentzian_sigma << std::endl;
    ss << "track_contact_weight = " << track_contact_weight << std::endl;
    ss << "body_contact_weight = " << body_contact_weight << std::endl;
    return ss.str();
  }
  double movement_cost_weight;
  double lorentzian_sigma;
  double track_contact_weight;
  double body_contact_weight;
  bool verbose;
};

class ConvexHullAreaError {
public:
  ConvexHullAreaError(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                      const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const std::vector<double>& prev_flipper_angles,
                      const sdf_contact_estimation::ContactEstimationResult &init_contact, const ConvexHullAreaErrorParameters& parameters);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                     const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const std::vector<double>& prev_flipper_angles,
                                     const sdf_contact_estimation::ContactEstimationResult &init_contact, const ConvexHullAreaErrorParameters &parameters);
private:
  unsigned int parameter_size_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::vector<double> prev_flipper_angles_;

  Eigen::Affine3d pose_;
  Eigen::Affine3d base_to_com_;
  sdf_contact_estimation::ContactEstimationResult init_contact_;

  ConvexHullAreaErrorParameters parameters_;
};

}

#endif
