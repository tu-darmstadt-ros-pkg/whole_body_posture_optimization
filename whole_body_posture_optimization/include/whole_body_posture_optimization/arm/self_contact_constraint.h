#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_SELF_CONTACT_CONSTRAINT_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_SELF_CONTACT_CONSTRAINT_H

#include "whole_body_posture_optimization/util/common.h"

#include <whole_body_posture_optimization/robot_model.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class SelfContactConstraint {
public:
  SelfContactConstraint(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<std::size_t>& param_idx_to_chain_idx,
                    const std::vector<double>& default_chain_angles, const CollisionObject& obj1, const CollisionObject& obj2)
    : robot_model_(robot_model), free_joints_(free_joints), param_idx_to_chain_idx_(param_idx_to_chain_idx), default_chain_angles_(default_chain_angles),
      obj1_(obj1), obj2_(obj2) {
    if (!robot_model_) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_posture_optimization"), "Robot model is null!");
    }
  }

  template <typename T> bool operator()(T const* const* parameters, T* cost) const {
    // Copy default chain angles to T vector
    std::vector<T> chain_angles(default_chain_angles_.size());
    for (unsigned int chain_idx = 0; chain_idx < chain_angles.size(); chain_idx++) {
      chain_angles[chain_idx] = T(default_chain_angles_[chain_idx]);
    }
    // Override free joints with parameters
    for (unsigned int p_idx = 0; p_idx < free_joints_.size(); p_idx++) {
      chain_angles[param_idx_to_chain_idx_[p_idx]] = parameters[0][p_idx];
    }

    // Compute sphere positions in world
    Vector3<T> sphere_center1 = obj1_.collision_sphere.relative_vec_.cast<T>();
    Affine3<T> base_to_link1 = robot_model_->getLinkTransform<T>(obj1_.link_name, chain_angles);
    Vector3<T> sphere_center_base1 = base_to_link1 * sphere_center1;

    Vector3<T> sphere_center2 = obj2_.collision_sphere.relative_vec_.cast<T>();
    Affine3<T> base_to_link2 = robot_model_->getLinkTransform<T>(obj2_.link_name, chain_angles);
    Vector3<T> sphere_center_base2 = base_to_link2 * sphere_center2;

    T distance = (sphere_center_base1 - sphere_center_base2).norm();

    // d - r1 - p1 - r2 - p2 > 0
    cost[0] = -(distance - T(obj1_.collision_sphere.radius_)
                - T(obj1_.padding) - T(obj2_.collision_sphere.radius_) - T(obj2_.padding)); // inequality constraints in nlopt have to be < 0
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<std::size_t>& param_idx_to_chain_idx,
                                     const std::vector<double>& default_chain_angles, const CollisionObject& obj1, const CollisionObject& obj2) {
    ceres::DynamicAutoDiffCostFunction<SelfContactConstraint>* cost_function = new ceres::DynamicAutoDiffCostFunction<SelfContactConstraint, 4>(
          new SelfContactConstraint(robot_model, free_joints, param_idx_to_chain_idx, default_chain_angles, obj1, obj2));

    cost_function->AddParameterBlock(free_joints.size());
    cost_function->SetNumResiduals(1);

    return static_cast<ceres::CostFunction*>(cost_function);
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_chain_idx_; // Transforms parameter index to chain index
  std::vector<std::string> free_joints_; // name of joint associated with parameter of same idx
  std::vector<double> default_chain_angles_; // default values for all chain joints

  CollisionObject obj1_;
  CollisionObject obj2_;
};

}

#endif
