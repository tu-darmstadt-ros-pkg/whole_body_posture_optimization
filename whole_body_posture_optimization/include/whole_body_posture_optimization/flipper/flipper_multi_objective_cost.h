#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_FLIPPER_MULTI_OBJECTIVE_COST_H

#include <rclcpp/rclcpp.hpp>
#include <ceres/cost_function.h>
#include <sstream>
#include <Eigen/Eigen>
#include <hector_pose_prediction_interface/pose_predictor.h>
#include <moveit/robot_state/robot_state.hpp>
#include <atomic>
#include <future>

#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost_parameters.h>
#include <whole_body_posture_optimization/util/worker_thread_pool.h>
#include <numeric>

#include <sdf_contact_estimation/prediction_diagnostics.h>

namespace whole_body_posture_optimization {

const std::string PROBABILITY_DISTRIBUTION_TABLE_NAME = "probability_distribution";

/// Predicts the settled pose from pose and returns its stability, NaN when the settled pose moved more than
/// max_position_deviation in x, y from pose. 0 disables the check.
double predictSettledPose(hector_pose_prediction_interface::PosePredictor<double>& pose_predictor,
                          const FlipperMultiObjectiveCostParameters& parameters, const Eigen::Isometry3d& pose,
                          hector_math::Pose<double>& result_pose,
                          hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                          hector_pose_prediction_interface::ContactInformation<double>& contact_information);

/// Evaluated postures a flipper solve rejected, by cause
struct FlipperRejections {
  std::atomic<int> unsettled{0}; ///< The prediction failed or the robot fell over
  std::atomic<int> drifted{0};   ///< The settled pose moved more than max_position_deviation
  std::atomic<int> transition{0}; ///< The new support polygon does not hold the previous centre of mass
  /// Unsettled by the pose predictor's status, for an SDFContactEstimation predictor
  std::atomic<int> fell_over{0};
  std::atomic<int> no_contact{0}; ///< No support polygon during or after the settling
  std::atomic<int> not_converged{0};
  /// Evaluations whose last predicted pose had sampling points in unobserved voxels, the failed ones among them
  std::atomic<int> touched_unknown{0};
  std::atomic<int> failed_touched_unknown{0};
};

class FlipperMultiObjectiveCost
{
public:
  FlipperMultiObjectiveCost(const std::vector<std::string>& free_joints,
                      const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                      const Eigen::Isometry3d& pose,
                      const moveit::core::RobotStateConstPtr& previous_state,
                      const FlipperMultiObjectiveCostParameters& parameters,
                      const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr(),
                      const std::vector<std::string>& coupled_joints={},
                      FlipperRejections* rejections=nullptr,
                      const std::vector<std::pair<size_t, size_t>>& mirrored_pairs={},
                      const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics=nullptr);

  bool operator()(double const* const* flipper_angles, double* cost) const;

  static ceres::CostFunction* Create(const std::vector<std::string>& free_joints,
                                     const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                     const Eigen::Isometry3d& pose,
                                     const moveit::core::RobotStateConstPtr& previous_state,
                                     const FlipperMultiObjectiveCostParameters& parameters,
                                     const moveit::core::RobotStateConstPtr& last_solution=moveit::core::RobotStateConstPtr(),
                                     const std::vector<std::string>& coupled_joints={},
                                     FlipperRejections* rejections=nullptr,
                                     const std::vector<std::pair<size_t, size_t>>& mirrored_pairs={},
                                     const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics=nullptr);
private:
  /// Sets the free joints and the joints coupled to them in the pose predictor
  void setJointPositions(const std::vector<double>& flipper_angles) const;

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
  FlipperRejections* rejections_;
  /// Diagnostics of the pose predictor's last prediction, null when it does not report them
  const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics_;
  /// Free joint indices of left and right flippers the symmetry cost keeps alike
  std::vector<std::pair<size_t, size_t>> mirrored_pairs_;
  hector_pose_prediction_interface::SupportPolygon<double> prev_support_polygon_;
  std::vector<size_t> param_idx_to_state_idx;
  /// (parameter index, predictor joint index) of every coupled joint
  std::vector<std::pair<size_t, size_t>> coupled_param_to_state_idx_;

  const FlipperMultiObjectiveCostParameters& parameters_;

  std::shared_ptr<WorkerThreadPool> worker_pool_;
};

}

#endif
