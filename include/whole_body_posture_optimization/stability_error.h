#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__STABILITY_ERROR_H

#include <whole_body_posture_optimization/robot_model.h>
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
  ComputeCenterOfMass(const std::shared_ptr<RobotModel>& robot_model)
    : robot_model_(robot_model) {
    joint_positions_.resize(robot_model->getNumJoints());
    this->mutable_parameter_block_sizes()->push_back(robot_model->getNumJoints());
    this->set_num_residuals(3);
  }
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    std::vector<double> joint_positions;
    for (unsigned int i = 0; i < robot_model_->getNumJoints(); i++) {
      joint_positions[i] = parameters[0][i];
    }

    Eigen::Vector3d com;
    Eigen::MatrixXd jacobian;
    robot_model_->computeCenterOfMass(joint_positions, com, jacobian);

    for (unsigned int i = 0; i < 3; i++) {
      residuals[i] = com(i);
    }

    // Compute the Jacobian if asked for.
    if (jacobians != NULL) {
      for (unsigned int i = 0; i < 3; i++) {
        if (jacobians[i] != NULL) {
          for (unsigned int j = 0; j < joint_positions.size(); j++) {
            jacobians[i][j] = jacobian(i, j); // TODO check!!!
          }
        }
      }
    }
    return true;
  }
private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<double> joint_positions_;
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
    Vector3T<T> distance = -axis_normal + (axis_normal.transpose() * force_component_norm) * force_component_norm; // d_i

    T sign = T(1);
//    if (force_component_norm.cross(axis_normal_norm) * axis_norm > 0) { // not differentiable
//      sign = 1;
//    } else {
//      sign = -1;
//    }
    T angle = sign * acos(force_component_norm.dot(axis_normal_norm));
    T stability = angle * distance.norm() * force_component.norm();
    stabilities.push_back(stability);
  }

  return stabilities;
}


class StabilityError {
public:
  StabilityError(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<Eigen::Vector3d>& contact_points);
  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    T* com = new T[3];
    compute_com_(parameters, com);
    Vector3T<T> com_vec(com[0], com[1], com[2]); // TODO use eigen map to eliminate copy
    std::vector<Vector3T<T>> contact_points_t;
    for (const Eigen::Vector3d& contact_point: contact_points_) {
      contact_points_t.push_back(contact_point.cast<T>());
    }
    Vector3T<T> external_force(T(0), T(0), T(9.81)); // only gravity
    std::vector<T> stabilities = computeForceAngleStabilities(contact_points_t, external_force, com_vec);
    T sum = T(0);
    for (const T& stability: stabilities) {
      sum -= stability;
    }
    cost[0] = sum;
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string> &free_joints, const std::vector<Eigen::Vector3d> &contact_points);

private:
  ceres::DynamicCostFunctionToFunctor compute_com_;
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_joint_idx_;
  std::vector<Eigen::Vector3d> contact_points_;
};

}

#endif
