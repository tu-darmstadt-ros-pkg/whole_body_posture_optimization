#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H

#include <whole_body_posture_optimization/robot_model.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3T = Eigen::Transform<T, 3, Eigen::Affine>;

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


template <typename T> std::vector<T> computeForceAngleStabilities(const std::vector<Vector3T<T>>& contact_points, const Vector3T<T>& external_force, const Vector3T<T>& com) {
  std::vector<T> stabilities;
  for (unsigned int i = 0; i < contact_points.size(); i++) {
    Vector3T<T> contact_point_p1 = contact_points[(i+1) % contact_points.size()];

    Vector3T<T> axis = contact_point_p1 - contact_points[i]; // a_i
    Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity(); // I
    Vector3T<T> axis_norm = axis / axis.norm(); // \hat{a}_i
    Eigen::Matrix<T, 3, 3> p1 = identity - axis_norm * axis_norm.transpose(); // I - \hat{a}_i * \hat{a}_i^T
    Vector3T<T> axis_normal = p1 * (contact_point_p1 - com); // l_i
    Vector3T<T> axis_normal_norm = axis_normal / axis_normal.norm();
    Vector3T<T> force_component = p1 * external_force; // f_i -> can be precomputed
    Vector3T<T> force_component_norm = force_component / force_component.norm(); // \hat{f}_i
    Vector3T<T> distance = -axis_normal + (axis_normal.dot(force_component_norm)) * force_component_norm; // d_i


    // TODO: differentiable signed angle: https://stackoverflow.com/a/33920320
    T t1 = force_component_norm.cross(axis_normal_norm).dot(axis_norm);
    T t2 = axis_normal_norm.dot(force_component_norm);
    T signed_angle = atan2(t1, t2);
    T angle = acos(axis_normal_norm.dot(force_component_norm));
    T stability = signed_angle * distance.norm() * force_component.norm();
    stabilities.push_back(stability);
  }

  return stabilities;
}

template <typename T> std::vector<T> computeForceAngleStabilitiesWithGravity(const std::vector<Vector3T<T>>& contact_points, const Vector3T<T>& com) {
  return computeForceAngleStabilities(contact_points, Vector3T<T>(T(0), T(0), T(-9.81)), com);
}


class StabilityError {
public:
  StabilityError(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints,
                 const std::vector<Eigen::Vector3d>& contact_points, const std::vector<double>& initial_pose, const Eigen::Affine3d& robot_pose);

  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    // Convert complete state to T
//    T* joint_angles = new T[param_idx_to_joint_idx_.size()];
//    for (unsigned int i = 0; i < param_idx_to_joint_idx_.size(); i++) {
//      joint_angles[i] = parameters[0][i];
//    }
    // Override free joints with parameters
//    for (unsigned int i = 0; i < param_idx_to_joint_idx_.size(); i++) {
//      joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
//    }

//    T** param_vec = &joint_angles;
    T* com = new T[3];
    (*compute_com_)(parameters, com);
    Vector3T<T> com_base(com[0], com[1], com[2]); // TODO use eigen map to eliminate copy
    Vector3T<T> com_world = robot_pose_.cast<T>() * com_base;
    std::vector<Vector3T<T>> contact_points_t;
    for (const Eigen::Vector3d& contact_point: contact_points_) {
      contact_points_t.push_back(contact_point.cast<T>());
    }

    std::vector<T> stabilities = computeForceAngleStabilitiesWithGravity(contact_points_t, com_world);
    std::vector<T> s(stabilities.size());
    std::transform(stabilities.begin(), stabilities.end(), s.begin(), [](T stability) -> T { return 1.0/(pow(stability, 2)); });
    T stability_sum2 = std::accumulate(s.begin(), s.end(), T(0));
//    T stability_sum = std::accumulate(stabilities.begin(), stabilities.end(), T(0));
//    T stability_min = *std::min_element(stabilities.begin(), stabilities.end());
    cost[0] = stability_sum2;
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string> &free_joints,
                                     const std::vector<Eigen::Vector3d> &contact_points, const std::vector<double>& initial_pose, const Eigen::Affine3d &robot_pose);

private:
  std::shared_ptr<ceres::DynamicCostFunctionToFunctor> compute_com_;
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_joint_idx_;
  std::vector<Eigen::Vector3d> contact_points_;
  Eigen::Affine3d robot_pose_;
  std::vector<double> initial_angles_;
};

}

#endif
