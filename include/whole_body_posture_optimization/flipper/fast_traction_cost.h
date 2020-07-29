#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_FAST_TRACTION_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_FAST_TRACTION_COST_H

#include <ceres/ceres.h>
#include <ros/ros.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>

namespace whole_body_posture_optimization {

class FastTractionCost {
  struct Parameters{
    static Parameters createFromServer(const ros::NodeHandle& nh) {
      Parameters parameters;
      return parameters;
    }
  };

public:
  FastTractionCost(const Eigen::Affine3d& pose, const std::vector<std::string>& free_joints, const std::vector<double>& initial_flipper_position, const std::vector<double>& previous_flipper_position,
                   const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Parameters& parameters);
  bool operator()(double const* const* parameters, double* cost) const;

  static ceres::CostFunction* Create(const Eigen::Affine3d& pose, const std::vector<std::string>& free_joints, const std::vector<double>& initial_flipper_position, const std::vector<double>& previous_flipper_position,
                                     const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Parameters& parameters);
private:
  Eigen::Affine3d pose_;

  std::vector<std::string> free_joints_;
  std::vector<size_t> parameter_idx_to_flipper_idx_;
  std::vector<double> initial_flipper_position_;
  std::vector<double> previous_flipper_position_;

  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  Parameters parameters_;
};

}

#endif
