#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_SDF_CONTACT_CONSTRAINT_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_SDF_CONTACT_CONSTRAINT_H

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <ceres/ceres.h>

#include <Eigen/Eigen>

namespace  whole_body_posture_optimization {

class SdfContactConstraint {
public:
  SdfContactConstraint(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<int>& param_idx_to_chain_idx,
                    const std::vector<double>& default_chain_angles, const Eigen::Isometry3d& robot_pose, const sdf_contact_estimation::SdfModelPtr& sdf_model,
                    const CollisionObject& obj)
    : robot_model_(robot_model), free_joints_(free_joints), param_idx_to_chain_idx_(param_idx_to_chain_idx), default_chain_angles_(default_chain_angles), robot_pose_(robot_pose),
  sdf_model_(sdf_model), obj_(obj) {
    if (!robot_model_) {
      ROS_ERROR_STREAM("Robot model is null!");
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

    // Compute sphere position in world
    Vector3<T> sphere_center = obj_.collision_sphere.relative_vec_.cast<T>();
    Affine3<T> base_to_link = robot_model_->computeChainFK<T>(obj_.link_name, chain_angles);
    Vector3<T> sphere_center_world = robot_pose_.cast<T>() * base_to_link * sphere_center;

    // Evaluate sdf at sphere position
    T sdf = sdf_model_->getSdf<T>(sphere_center_world.x(), sphere_center_world.y(), sphere_center_world.z());

    // sdf - r > 0
    cost[0] = -(sdf - T(obj_.collision_sphere.radius_) - T(obj_.padding)); // inequality constraints in nlopt have to be < 0
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<int>& param_idx_to_chain_idx,
                                     const std::vector<double>& default_chain_angles, const Eigen::Isometry3d& robot_pose, const sdf_contact_estimation::SdfModelPtr& sdf_model,
                                     const CollisionObject& obj) {
    auto* cost_function = new ceres::DynamicAutoDiffCostFunction<SdfContactConstraint, 4>(
          new SdfContactConstraint(robot_model, free_joints, param_idx_to_chain_idx, default_chain_angles, robot_pose, sdf_model, obj));

    cost_function->AddParameterBlock(free_joints.size());
    cost_function->SetNumResiduals(1);

    return static_cast<ceres::CostFunction*>(cost_function);
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<int> param_idx_to_chain_idx_; // Transforms parameter index to chain index
  std::vector<std::string> free_joints_; // name of joint associated with parameter of same idx
  std::vector<double> default_chain_angles_; // default values for all chain joints
  Eigen::Isometry3d robot_pose_; // current robot pose in world
  sdf_contact_estimation::SdfModelPtr sdf_model_;

  CollisionObject obj_;
};

}

#endif
