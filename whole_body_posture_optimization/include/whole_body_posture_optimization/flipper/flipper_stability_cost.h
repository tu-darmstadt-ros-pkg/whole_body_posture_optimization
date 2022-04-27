#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_STABILITY_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_STABILITY_COST_H

#include <ros/node_handle.h>
#include <Eigen/Eigen>
#include <ceres/cost_function.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

//#include <whole_body_posture_optimization/common.h>

namespace whole_body_posture_optimization {

struct FlipperStabilityCostParameters {
  static FlipperStabilityCostParameters createFromServer(const ros::NodeHandle& nh) {
    FlipperStabilityCostParameters parameters;

    std::vector<double> exp_parameters = {1.6, -1.7, 0.0};
    nh.param("exp_parameters", exp_parameters, exp_parameters);
    if (exp_parameters.size() != 3) {
      ROS_ERROR_STREAM("Invalid size for parameter 'exp_parameter'. Got " << exp_parameters.size() << " expected 3");
      parameters.exp_parameters[0] = parameters.exp_parameters[1] = parameters.exp_parameters[2] = 0;
    } else {
      std::copy(begin(exp_parameters), end(exp_parameters), parameters.exp_parameters);
    }
    nh.param("lorentzian_sigma", parameters.lorentzian_sigma, 0.1);
    nh.param("movement_cost_weight", parameters.movement_cost_weight, 0.0);
    return parameters;
  }

  std::string toString() {
    std::stringstream ss;
    ss << "FlipperStabilityCostParameters:" << std::endl;
    ss << "lorentzian_sigma = " << lorentzian_sigma << std::endl;
    ss << "movement_cost_weight = " << movement_cost_weight << std::endl;
    ss << "exp_parameters = [" << exp_parameters[0] << ", " << exp_parameters[1] << ", " << exp_parameters[2] << "]" << std::endl;
    return ss.str();
  }

  double exp_parameters[3];
  double movement_cost_weight;
  double lorentzian_sigma;
};

class FlipperStabilityCost {
public:
  FlipperStabilityCost(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                      const Eigen::Isometry3d& pose, const Eigen::Isometry3d& base_to_com, const std::vector<double>& prev_flipper_angles,
                       const FlipperStabilityCostParameters& parameters);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(unsigned int parameter_size, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                     const Eigen::Isometry3d& pose, const Eigen::Isometry3d& base_to_com, const std::vector<double>& prev_flipper_angles,
                                     const FlipperStabilityCostParameters &parameters);
private:
  unsigned int parameter_size_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::vector<double> prev_flipper_angles_;

  Eigen::Isometry3d pose_;
  Eigen::Isometry3d base_to_com_;

  FlipperStabilityCostParameters parameters_;
};

}

#endif
