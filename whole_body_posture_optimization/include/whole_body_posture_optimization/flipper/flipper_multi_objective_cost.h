#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_H

#include <rclcpp/rclcpp.hpp>
#include <ceres/cost_function.h>
#include <sstream>
#include <Eigen/Eigen>
#include <hector_pose_prediction_interface/pose_predictor.h>
#include <moveit/robot_state/robot_state.hpp>
#include <future>

#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost_parameters.h>
#include <whole_body_posture_optimization/util/worker_thread_pool.h>
#include <numeric>

namespace whole_body_posture_optimization {

const std::string PROBABILITY_DISTRIBUTION_TABLE_NAME = "probability_distribution";

class FlipperMultiObjectiveCost
{
public:
  FlipperMultiObjectiveCost(const std::vector<std::string>& free_joints,
                      const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                      const Eigen::Isometry3d& pose,
                      const moveit::core::RobotStateConstPtr& previous_state,
                      const FlipperMultiObjectiveCostParameters& parameters,
                      const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr());

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<std::string>& free_joints,
                                     const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                     const Eigen::Isometry3d& pose,
                                     const moveit::core::RobotStateConstPtr& previous_state,
                                     const FlipperMultiObjectiveCostParameters& parameters,
                                     const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr());
private:
  struct CostSample {
    double cost;
    double relative_probability;
    friend std::ostream& operator<<(std::ostream& os, const CostSample& s)
    {
      os << "{" << s.cost << "," << s.relative_probability << "}";
      return os;
    }
  };
  double evaluateCost(const Eigen::Isometry3d& pose_sample, const std::vector<double>& flipper_angles) const;
  void evaluateCostAsync(const Eigen::Isometry3d& pose_sample, const std::vector<double>& flipper_angles, std::promise<double>& cost_promise) const;
  double computeCVaR(std::vector<CostSample> samples, double risk_level) const;

  const std::vector<std::string>& free_joints_;
  const hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  Eigen::Isometry3d pose_;
  hector_math::Pose<double> pose_math_;
  std::vector<double> prev_flipper_param_angles_;
  std::vector<double> last_solution_;
  Eigen::Isometry3d prev_pose_;
  Eigen::Vector3d prev_com_;
  hector_pose_prediction_interface::SupportPolygon<double> prev_support_polygon_;
  std::vector<size_t> param_idx_to_state_idx;

  const FlipperMultiObjectiveCostParameters& parameters_;

  std::shared_ptr<WorkerThreadPool> worker_pool_;
};

}

#endif
