#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_OPTIMIZATION_H

#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

#include <sdf_contact_estimation/visualisation.h>
#include <hector_pose_prediction_interface/pose_predictor.h>

#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>
//#include <whole_body_posture_optimization/flipper/flipper_stability_cost.h>
//#include <whole_body_posture_optimization/flipper/fast_traction_cost.h>

namespace whole_body_posture_optimization {

struct FlipperOptimizationResult {
  std::vector<double> joint_state;
  double stability;
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  Eigen::Isometry3d pose;
  double final_cost;
};

class FlipperOptimization {
public:
  FlipperOptimization(ros::NodeHandle& nh, const std::shared_ptr<RobotModel>& robot_model,
                      const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor);
  FlipperOptimizationResult optimize(const Eigen::Isometry3d& robot_pose, const std::vector<double> &init_joint_angles,
                                     const robot_state::RobotStateConstPtr& previous_state);

  bool enabled() const;
  void setEnabled(bool enabled);

private:
  bool loadParameters(const ros::NodeHandle &nh);
  bool loadLimits();

  ros::NodeHandle nh_;

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_jnt_idx_;
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;

  nlopt::opt opt_;
  bool enabled_;
  unsigned int parameter_count_;
  std::vector<std::string> free_joints_;
  std::vector<double> upper_bounds_;
  std::vector<double> lower_bounds_;
  std::string objective_;

  FlipperMultiObjectiveCostParameters flipper_multi_objective_cost_parameters_;
//  FlipperStabilityCostParameters stability_cost_parameters_;
//  FastTractionCost::Parameters fast_traction_cost_parameters_;

  ros::Publisher start_state_pub_;
  ros::Publisher result_state_pub_;
  ros::Publisher contact_points_pub_;
  ros::Publisher result_com_pub_;
};

}

#endif
