#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H

#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

#include <sdf_contact_estimation/visualisation.h>

#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>
#include <whole_body_posture_optimization/flipper/flipper_stability_cost.h>
#include <whole_body_posture_optimization/flipper/fast_traction_cost.h>

namespace whole_body_posture_optimization {

struct FlipperOptimizationResult {
  std::vector<double> flipper_angles;
  std::vector<double> joint_state;
  sdf_contact_estimation::ContactEstimationResult contact_result;
  double final_cost;
};

class FlipperOptimization {
public:
  FlipperOptimization(ros::NodeHandle& nh, const std::shared_ptr<RobotModel>& robot_model, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen,
                      const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);
  FlipperOptimizationResult optimize(const Eigen::Affine3d& robot_pose, const Eigen::Affine3d& base_to_com, const std::vector<double> &init_joint_angles,
                                     const std::vector<double>& previous_joint_angles);
  sdf_contact_estimation::ContactEstimationResult computeContactEstimation(const std::vector<double>& joint_angles,const Eigen::Affine3d& pose, const Eigen::Affine3d &base_to_com,
                                                                           const sdf_contact_estimation::ContactEstimationResult& last_result);
  sdf_contact_estimation::ContactEstimationResult computeContactEstimation(const std::vector<double>& joint_angles,const Eigen::Affine3d& pose, const Eigen::Affine3d &base_to_com);
  bool enabled() const;
  void setEnabled(bool enabled);

private:
  std::vector<double> jointVectorToParamVector(const std::vector<double>& joint_vec);
  std::vector<double> jointVectorToFlipperVector(const std::vector<double>& joint_vec);
  bool loadParameters(const ros::NodeHandle &nh);
  bool loadLimits();

  ros::NodeHandle nh_;

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_jnt_idx_;
  std::vector<std::size_t> flippers_idx_to_jnt_idx_;
  std::vector<std::size_t> param_idx_to_flippers_idx_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape_gen_;

  nlopt::opt opt_;
  bool enabled_;
  unsigned int parameter_count_;
  std::vector<std::string> free_joints_;
  std::vector<double> upper_bounds_;
  std::vector<double> lower_bounds_;
  std::string objective_;

  ConvexHullAreaErrorParameters convex_hull_error_parameters_;
  FlipperStabilityCostParameters stability_cost_parameters_;
  FastTractionCost::Parameters fast_traction_cost_parameters_;

  ros::Publisher start_state_pub_;
  ros::Publisher result_state_pub_;
  ros::Publisher contact_points_pub_;
  ros::Publisher result_com_pub_;
};

}

#endif
