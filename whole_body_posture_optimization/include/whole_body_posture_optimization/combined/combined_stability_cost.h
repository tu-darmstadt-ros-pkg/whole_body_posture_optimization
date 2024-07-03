#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_COMBINED_STABILITY_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_COMBINED_STABILITY_COST_H

#include <whole_body_posture_optimization/robot_model.h>
#include "whole_body_posture_optimization/util/common.h"
#include <sdf_contact_estimation/sdf_contact_estimation.h>

namespace whole_body_posture_optimization {

struct CombinedStabilityCostParameters {
  double regularization_weight = 0.5;
  double lorentzian_sigma = 0.6;
  double exp_parameters[3];
};

class CombinedStabilityCost {
public:
  CombinedStabilityCost(const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                        const std::vector<std::string>& free_joints_, const std::vector<std::string>& free_flipper_joints, const CombinedStabilityCostParameters& parameters,
                        Eigen::Isometry3d* result_robot_pose);

  CombinedStabilityCost* updateData(const std::vector<double>& initial_full_state, const std::vector<double>& previous_solution, const Eigen::Isometry3d& robot_pose,
                                    const sdf_contact_estimation::ContactEstimationResult &init_contact);

  bool operator()(double const* const* parameters, double* cost) const;

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation,
                                     const std::vector<std::string>& free_joints_, const std::vector<std::string>& free_flipper_joints,
                                     const CombinedStabilityCostParameters& parameters, const std::vector<double>& initial_full_state,
                                     const std::vector<double>& previous_solution, const Eigen::Isometry3d& robot_pose,
                                     const sdf_contact_estimation::ContactEstimationResult &init_contact, Eigen::Isometry3d* result_robot_pose);
private:
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  std::vector<std::string> free_joints_;
  std::vector<std::string> free_flipper_joints_;

  std::vector<double> full_joint_state_;
  std::vector<double> previous_solution_;

  sdf_contact_estimation::ContactEstimationResult init_contact_;

  Eigen::Isometry3d robot_pose_;
  Eigen::Isometry3d* result_robot_pose_;
  std::vector<std::size_t> param_idx_to_joint_idx_;

  CombinedStabilityCostParameters parameters_;
};

}

#endif
