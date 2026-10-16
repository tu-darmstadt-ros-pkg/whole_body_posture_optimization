#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H

#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>
#include <ceres_nlopt_wrapper/nlopt_parameter_server.h>

#include <sdf_contact_estimation/visualisation.h>
#include <hector_pose_prediction_interface/pose_predictor.h>

#include "whole_body_posture_optimization/util/common.h"
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>

#include <whole_body_posture_optimization/solvers/optimizer_base.h>

namespace whole_body_posture_optimization {

struct FlipperOptimizationResult {
  std::vector<double> joint_state;
  double stability;
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> contact_information;
  Eigen::Isometry3d pose;
  double final_cost;
};

class FlipperOptimization {
public:
  FlipperOptimization(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<RobotModel>& robot_model,
                      const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor);
  FlipperOptimizationResult optimize(const Eigen::Isometry3d& robot_pose,
                                     const std::vector<double> &init_joint_angles,
                                     const moveit::core::RobotStateConstPtr& previous_state,
                                     const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr());

  bool enabled() const;
  void setEnabled(bool enabled);

private:
  bool loadParameters(rclcpp::Node& node);
  bool loadLimits();

  rclcpp::Node::SharedPtr node_;
  std::string param_prefix_;

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_jnt_idx_;
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;

  std::shared_ptr<ceres_nlopt_wrapper::NloptParameterServer> opt_server_;
  std::shared_ptr<ceres_nlopt_wrapper::NloptParameterServer> local_opt_server_;
  std::shared_ptr<OptimizerBase> custom_solver_;
  bool enabled_;
  unsigned int parameter_count_;
  std::vector<std::string> free_joints_;
  /// coupled_joints_[i] follows free_joints_[i]; empty entries follow nothing
  std::vector<std::string> coupled_joints_;
  std::vector<std::size_t> coupled_idx_to_jnt_idx_;
  std::vector<double> upper_bounds_;
  std::vector<double> lower_bounds_;

  FlipperMultiObjectiveCostParameters flipper_multi_objective_cost_parameters_;
};

}

#endif
