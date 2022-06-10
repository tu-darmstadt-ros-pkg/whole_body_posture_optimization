#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_CONVEX_HULL_AREA_ERROR_H

#include <ros/node_handle.h>
#include <ceres/cost_function.h>
#include <sstream>
#include <Eigen/Eigen>
#include <hector_pose_prediction_interface/pose_predictor.h>
#include <moveit/robot_state/robot_state.h>

namespace whole_body_posture_optimization {

struct ConvexHullAreaErrorParameters {
  static ConvexHullAreaErrorParameters createFromServer(const ros::NodeHandle& nh) {
    ConvexHullAreaErrorParameters parameters_{};
    nh.param("support_area_weight", parameters_.support_area_weight, 1.0);
    nh.param("movement_cost_weight", parameters_.movement_cost_weight, 0.2);
    nh.param("lorentzian_sigma", parameters_.lorentzian_sigma, 0.2);
    nh.param("track_contact_weight", parameters_.track_contact_weight, 2.0);
    nh.param("body_contact_weight", parameters_.body_contact_weight, 4.0);
    nh.param("position_deviation_weight", parameters_.position_deviation_weight, 10.0);
    nh.param("roll_cost_weight", parameters_.roll_cost_weight, 2.0);
    nh.param("verbose", parameters_.verbose, false);
    return parameters_;
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "Heuristic cost parameters:" << std::endl;
    ss << "support_area_weight = " << support_area_weight << std::endl;
    ss << "movement_cost_weight = " << movement_cost_weight << std::endl;
    ss << "lorentzian_sigma = " << lorentzian_sigma << std::endl;
    ss << "track_contact_weight = " << track_contact_weight << std::endl;
    ss << "body_contact_weight = " << body_contact_weight << std::endl;
    ss << "position_deviation_weight = " << position_deviation_weight << std::endl;
    ss << "roll_cost_weight = " << roll_cost_weight << std::endl;
    return ss.str();
  }
  double support_area_weight;
  double movement_cost_weight;
  double lorentzian_sigma;
  double track_contact_weight;
  double body_contact_weight;
  double turning_resistance_weight;
  double position_deviation_weight;
  double roll_cost_weight;
  bool verbose;
};

class ConvexHullAreaError {
public:
  ConvexHullAreaError(const std::vector<std::string>& free_joints,
                      const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                      const Eigen::Isometry3d& pose,
                      const robot_state::RobotStateConstPtr& previous_state,
                      const ConvexHullAreaErrorParameters& parameters);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<std::string>& free_joints,
                                     const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                     const Eigen::Isometry3d& pose,
                                     const robot_state::RobotStateConstPtr& previous_state,
                                     const ConvexHullAreaErrorParameters &parameters);
private:
  const std::vector<std::string>& free_joints_;
  const hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  Eigen::Isometry3d pose_;
  hector_math::Pose<double> pose_math_;
  std::vector<double> prev_flipper_param_angles_;
  Eigen::Isometry3d prev_pose_;
  std::vector<size_t> param_idx_to_state_idx;

  const ConvexHullAreaErrorParameters& parameters_;
};

}

#endif
