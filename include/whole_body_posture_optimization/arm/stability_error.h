#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class ComputeCenterOfMass : public ceres::CostFunction {
public:
  ComputeCenterOfMass(const std::shared_ptr<RobotModel>& robot_model, const std::vector<double>& init_angles, const std::vector<unsigned int>& param_idx_to_joint_idx)
    : robot_model_(robot_model), init_angles_(init_angles), param_idx_to_joint_idx_(param_idx_to_joint_idx) {
    this->mutable_parameter_block_sizes()->push_back(param_idx_to_joint_idx.size());
    this->set_num_residuals(3);
  }
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    std::vector<double> joint_angles = init_angles_;
    for (unsigned int i = 0; i < this->parameter_block_sizes()[0]; i++) {
      joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
    }

    Eigen::Vector3d com;
    Eigen::MatrixXd ad_com;
    robot_model_->computeCenterOfMass(joint_angles, com, ad_com);

    for (unsigned int i = 0; i < 3; i++) {
      residuals[i] = com(i);
    }

    // Compute the Jacobian if asked for.
    if (jacobians != NULL && jacobians[0] != NULL) {
      for (unsigned int r = 0; r < 3; r++) {
        for (unsigned int p = 0; p < this->parameter_block_sizes()[0]; p++) {
          jacobians[0][r * this->parameter_block_sizes()[0] + p] = ad_com(r, param_idx_to_joint_idx_[p]);
        }
      }
    }
    return true;
  }
private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<double> init_angles_;
  std::vector<unsigned int> param_idx_to_joint_idx_;

};

struct StabilityErrorParameters {

  static StabilityErrorParameters createFromServer(const ros::NodeHandle& nh) {
    StabilityErrorParameters parameters;

    std::vector<double> exp_parameters = {1.6, -1.7, 0.0};
    nh.param("exp_parameters", exp_parameters, exp_parameters);
    if (exp_parameters.size() != 3) {
      ROS_ERROR_STREAM("Invalid size for parameter 'exp_parameter'. Got " << exp_parameters.size() << " expected 3");
      parameters.exp_parameters[0] = parameters.exp_parameters[1] = parameters.exp_parameters[2] = 0;
    } else {
      std::copy(begin(exp_parameters), end(exp_parameters), parameters.exp_parameters);
    }
    nh.param("lorentzian_sigma", parameters.lorentzian_sigma, 0.1);
    nh.param("regularization_weight", parameters.regularization_weight, 0.0);
    nh.param("function_scale", parameters.function_scale, 1.0);
    return parameters;
  }

  std::string toString() {
    std::stringstream ss;
    ss << "Stability cost parameters:" << std::endl;
    ss << "lorentzian_sigma = " << lorentzian_sigma << std::endl;
    ss << "regularization_weight = " << regularization_weight << std::endl;
    ss << "function_scale = " << function_scale << std::endl;
    ss << "exp_parameters = [" << exp_parameters[0] << ", " << exp_parameters[1] << ", " << exp_parameters[2] << "]" << std::endl;
    return ss.str();
  }

  double exp_parameters[3];
  double regularization_weight;
  double lorentzian_sigma;
  double function_scale;
};

class StabilityError {
public:
  StabilityError(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<Eigen::Vector3d>& contact_points,
                 const std::vector<double>& initial_joint_angles, const std::vector<double>& previous_param_angles, const Eigen::Affine3d& robot_pose,
                 const StabilityErrorParameters& parameters);

  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    // Compute COM
    T com[3];
    (*compute_com_)(parameters, &com[0]);
    Vector3<T> com_base(com[0], com[1], com[2]); // TODO use eigen map to eliminate copy
//    ROS_INFO_STREAM("com_base: " << com_base.x() << std::endl << com_base.y() << std::endl << com_base.z());
    Vector3<T> com_world = robot_pose_.cast<T>() * com_base;
    // Cast contact points
    std::vector<Vector3<T>> contact_points_t;
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

    // Compute regularization
    T regularization_cost = T(0.0);
    if (!previous_param_angles_.empty()) {
      for (unsigned int idx = 0; idx < previous_param_angles_.size(); idx++) {
        T angle_diff = T(previous_param_angles_[idx]) - parameters[0][idx];
        regularization_cost += lorentzian<T>(angle_diff, T(parameters_.lorentzian_sigma));
      }
    }
//    ROS_INFO_STREAM("regularization_cost: " << regularization_cost);

    cost[0] = parameters_.function_scale * (stability_sum + T(parameters_.regularization_weight) * regularization_cost);
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                                     const std::vector<double>& initial_joint_angles, const std::vector<double> &previous_param_angles, const Eigen::Affine3d &robot_pose, const StabilityErrorParameters &parameters);

private:
  std::shared_ptr<ceres::DynamicCostFunctionToFunctor> compute_com_;
  std::shared_ptr<RobotModel> robot_model_;

  std::vector<unsigned int> param_idx_to_joint_idx_;
  std::vector<Eigen::Vector3d> contact_points_;
  Eigen::Affine3d robot_pose_;
  std::vector<double> initial_joint_angles;
  std::vector<double> previous_param_angles_;

  StabilityErrorParameters parameters_;
};

}

#endif
