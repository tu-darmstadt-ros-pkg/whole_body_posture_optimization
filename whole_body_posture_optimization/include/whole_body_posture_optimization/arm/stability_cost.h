#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_STABILITY_COST_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_STABILITY_COST_H

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/arm/stability_cost_parameters.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class StabilityCost
{
public:
  StabilityCost(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<Eigen::Vector3d>& contact_points,
                 const std::vector<double>& full_state, const std::vector<double>& previous_param_angles, const Eigen::Isometry3d& robot_pose,
                 const StabilityCostParameters& parameters);

  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    // Compose full state
    std::vector<T> joint_angles(full_state_.size());
    for (unsigned int i = 0; i < joint_angles.size(); ++i) {
      joint_angles[i] = T(full_state_[i]);
    }
    for (unsigned int i = 0; i < param_idx_to_joint_idx_.size(); ++i) {
      joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
    }

    // Compute COM
    Vector3<T> com_base = robot_model_->template computeCenterOfMass(joint_angles);
    Vector3<T> com_world = robot_pose_.cast<T>() * com_base;
    // Cast contact points
    std::vector<Vector3<T>> contact_points_t;
    contact_points_t.reserve(contact_points_.size());
    for (const Eigen::Vector3d& contact_point: contact_points_) {
      contact_points_t.push_back(contact_point.cast<T>());
    }
    if (contact_points_t.empty()) {
      return false;
    }

    // Compute stabilities
    std::vector<T> stabilities = computeForceAngleStabilitiesWithGravity(contact_points_t, com_world);
//    for (unsigned int i = 0; i < stabilities.size(); i++) {
//      ROS_INFO_STREAM("Stability " << i << ": " << stabilities[i]);
//    }

    // Transform stability to cost
    std::transform(stabilities.begin(), stabilities.end(), stabilities.begin(), [this](T stability) -> T {
      return T(parameters_.exp_parameters[0]) * exp(T(parameters_.exp_parameters[1])*stability+T(parameters_.exp_parameters[2]));
    });
//    for (unsigned int i = 0; i < stabilities.size(); i++) {
//      ROS_INFO_STREAM("Stability transformed" << i << ": " << stabilities[i]);
//    }

    // Sum up and normalize by number of support vector edges
    T stability_sum = std::accumulate(stabilities.begin(), stabilities.end(), T(0));
//    ROS_INFO_STREAM("Stability sum: " << stability_sum);
    stability_sum = stability_sum / T(contact_points_.size()); // average cost per axis

    // Compute movement cost
    T movement_cost = T(0.0);
    if (!previous_param_angles_.empty()) {
      for (unsigned int idx = 0; idx < previous_param_angles_.size(); idx++) {
        T angle_diff = T(previous_param_angles_[idx]) - parameters[0][idx];
        movement_cost += lorentzian<T>(angle_diff, T(parameters_.lorentzian_sigma));
      }
      movement_cost /= T(previous_param_angles_.size()); // Normalize by joint count
    }

    T weighted_movement_cost = T(parameters_.movement_cost_weight) * movement_cost;
    cost[0] = parameters_.function_scale * (stability_sum + weighted_movement_cost);

//    ROS_INFO_STREAM("Stability cost: " << stability_sum << std::endl <<
//                    "Movement cost: " << movement_cost << "[" << weighted_movement_cost << "]");
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                                     const std::vector<double>& full_state, const std::vector<double> &previous_param_angles, const Eigen::Isometry3d &robot_pose, const StabilityCostParameters &parameters);

private:
  std::vector<double> full_state_;
  std::shared_ptr<RobotModel> robot_model_;

  std::vector<std::size_t> param_idx_to_joint_idx_;
  std::vector<Eigen::Vector3d> contact_points_;
  Eigen::Isometry3d robot_pose_;
  std::vector<double> previous_param_angles_;

  StabilityCostParameters parameters_;
};

}

#endif
