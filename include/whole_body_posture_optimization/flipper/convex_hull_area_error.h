#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H

#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <ceres/cost_function.h>

namespace whole_body_posture_optimization {

struct ConvexHullAreaErrorParameters {
  static ConvexHullAreaErrorParameters createFromServer(const ros::NodeHandle& nh) {
    ConvexHullAreaErrorParameters parameters_;
    nh.param("movement_cost_weight", parameters_.movement_cost_weight, 0.2);
    nh.param("lorentzian_sigma", parameters_.lorentzian_sigma, 0.2);
    nh.param("track_contact_weight", parameters_.track_contact_weight, 2.0);
    nh.param("body_contact_weight", parameters_.body_contact_weight, 4.0);
    nh.param("position_deviation_weight", parameters_.position_deviation_weight, 10.0);
    nh.param("roll_cost_weight", parameters_.roll_cost_weight, 2.0);
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
    ss << "position_deviation_weight = " << position_deviation_weight << std::endl;
    ss << "roll_cost_weight = " << roll_cost_weight << std::endl;
    return ss.str();
  }
  double movement_cost_weight;
  double lorentzian_sigma;
  double track_contact_weight;
  double body_contact_weight;
  double position_deviation_weight;
  double roll_cost_weight;
  bool verbose;
};

class ConvexHullAreaError {
public:
  ConvexHullAreaError(const std::vector<std::string>& free_joints, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                      const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const std::vector<double>& init_flipper_angles,
                      const std::vector<double>& prev_flipper_angles, const ConvexHullAreaErrorParameters& parameters);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<std::string>& free_joints, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                     const Eigen::Affine3d& pose, const Eigen::Affine3d& base_to_com, const std::vector<double>& init_flipper_angles,
                                     const std::vector<double>& prev_flipper_angles, const ConvexHullAreaErrorParameters &parameters);
private:
  const std::vector<std::string>& free_joints_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  Eigen::Affine3d pose_;
  Eigen::Affine3d base_to_com_;
  std::vector<double> init_flipper_angles_;
  std::vector<double> prev_flipper_param_angles_;
  std::vector<size_t> param_idx_to_flipper_state_;



  const ConvexHullAreaErrorParameters& parameters_;
};

}

#endif
