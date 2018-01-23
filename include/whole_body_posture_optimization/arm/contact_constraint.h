#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__CONTACT_CONSTRAINT_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__CONTACT_CONSTRAINT_H

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <sdf_contact_estimation/sdf/sdf_wrapper.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class ContactConstraint {
public:
  ContactConstraint(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<int>& param_idx_to_chain_idx,
                    const std::vector<double>& default_chain_angles, const Eigen::Affine3d& robot_pose, const sdf_contact_estimation::SdfWrapper& sdf,
                    const collision_detection::CollisionSphere& sphere, std::string link_name, double padding)
    : robot_model_(robot_model), free_joints_(free_joints), param_idx_to_chain_idx_(param_idx_to_chain_idx), default_chain_angles_(default_chain_angles), robot_pose_(robot_pose),
  sdf_(&sdf), sphere_(sphere), link_name_(link_name), padding_(padding) {
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
    Vector3<T> sphere_center = sphere_.relative_vec_.cast<T>();
    Affine3<T> base_to_link = robot_model_->computeChainFK<T>(link_name_, chain_angles);
    Vector3<T> sphere_center_world = robot_pose_.cast<T>() * base_to_link * sphere_center;

    // Evaluate sdf at sphere position
    T sdf = sdf_->getSdf<T>(sphere_center_world.x(), sphere_center_world.y(), sphere_center_world.z());

    // sdf - r > 0
    cost[0] = -(sdf - T(sphere_.radius_) - T(padding_)); // inequality constraints in nlopt have to be < 0
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::string>& free_joints, const std::vector<int>& param_idx_to_chain_idx,
                                     const std::vector<double>& default_chain_angles, const Eigen::Affine3d& robot_pose, const sdf_contact_estimation::SdfWrapper& sdf,
                                     const collision_detection::CollisionSphere& sphere, std::string link_name, double padding) {
    ceres::DynamicAutoDiffCostFunction<ContactConstraint>* cost_function = new ceres::DynamicAutoDiffCostFunction<ContactConstraint, 4>(
          new ContactConstraint(robot_model, free_joints, param_idx_to_chain_idx, default_chain_angles, robot_pose, sdf, sphere, link_name, padding));

    cost_function->AddParameterBlock(free_joints.size());
    cost_function->SetNumResiduals(1);

    return static_cast<ceres::CostFunction*>(cost_function);
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<int> param_idx_to_chain_idx_; // Transforms parameter index to chain index
  std::vector<std::string> free_joints_; // name of joint associated with parameter of same idx
  std::vector<double> default_chain_angles_; // default values for all chain joints
  Eigen::Affine3d robot_pose_; // current robot pose in world
  const sdf_contact_estimation::SdfWrapper* sdf_;

  collision_detection::CollisionSphere sphere_; // collision sphere
  std::string link_name_; // link where sphere is attached to
  double padding_;
};

}

#endif
