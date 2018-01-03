#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H

#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <algorithm>
//#undef LOG
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
//    std::vector<double> joint_positions(robot_model_->getNumJoints());
//    for (unsigned int i = 0; i < robot_model_->getNumJoints(); i++) {
//      joint_positions[i] = parameters[0][i];
//    }

//    std::vector<double> joint_positions(parameters[0][0], parameters[0][0] + robot_model_->getNumJoints());
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
          jacobians[0][r * this->num_residuals() + p] = ad_com(r, param_idx_to_joint_idx_[p]);
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

class StabilityError {
public:
  StabilityError(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<Eigen::Vector3d>& contact_points,
                 const std::vector<double>& initial_joint_angles, const std::vector<double>& previous_param_angles, const Eigen::Affine3d& robot_pose);

  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    // Compute COM
    T* com = new T[3];
    (*compute_com_)(parameters, com);
    Vector3<T> com_base(com[0], com[1], com[2]); // TODO use eigen map to eliminate copy
    Vector3<T> com_world = robot_pose_.cast<T>() * com_base;

    // Cast contact points
    std::vector<Vector3<T>> contact_points_t;
    for (const Eigen::Vector3d& contact_point: contact_points_) {
      contact_points_t.push_back(contact_point.cast<T>());
    }

    // Compute stabilities
    std::vector<T> stabilities = computeForceAngleStabilitiesWithGravity(contact_points_t, com_world);
    std::transform(stabilities.begin(), stabilities.end(), stabilities.begin(), [](T stability) -> T { return 1.0/(pow(stability, 2)); });
    T stability_inverted_sum = std::accumulate(stabilities.begin(), stabilities.end(), T(0));
//    T stability_sum = std::accumulate(stabilities.begin(), stabilities.end(), T(0));
//    T stability_min = *std::min_element(stabilities.begin(), stabilities.end());

    // Compute regularization
    T regularization_weight = T(0.1);
    T regularization_cost = T(0.0);
    for (unsigned int idx = 0; idx < previous_param_angles_.size(); idx++) {
      T angle_diff = T(previous_param_angles_[idx]) - parameters[0][idx];
      regularization_cost += lorentzian<T>(angle_diff, T(0.1));
    }

    cost[0] = stability_inverted_sum + regularization_weight * regularization_cost;
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel>& robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points,
                                     const std::vector<double>& initial_joint_angles, const std::vector<double> &previous_param_angles, const Eigen::Affine3d &robot_pose);

private:
  std::shared_ptr<ceres::DynamicCostFunctionToFunctor> compute_com_;
  std::shared_ptr<RobotModel> robot_model_;

  std::vector<unsigned int> param_idx_to_joint_idx_;
  std::vector<Eigen::Vector3d> contact_points_;
  Eigen::Affine3d robot_pose_;
  std::vector<double> initial_joint_angles;
  std::vector<double> previous_param_angles_;
};

}

#endif
