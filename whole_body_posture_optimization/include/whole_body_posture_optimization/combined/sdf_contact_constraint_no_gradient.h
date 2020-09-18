#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_SDF_CONTACT_CONSTRAINT_NO_GRADIENT_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_SDF_CONTACT_CONSTRAINT_NO_GRADIENT_H

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <sdf_contact_estimation/sdf/sdf_wrapper.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class SdfContactConstraintNoGradient {
public:
  SdfContactConstraintNoGradient(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::size_t>& param_idx_to_joint_idx, const std::vector<double>& initial_full_state,
                                 const Eigen::Affine3d* robot_pose, const sdf_contact_estimation::SdfWrapper& sdf, const CollisionObject& obj)
    : robot_model_(robot_model), param_idx_to_joint_idx_(param_idx_to_joint_idx), initial_full_state_(initial_full_state), robot_pose_(robot_pose), sdf_(&sdf), obj_(obj) {
    if (!robot_model_) {
      ROS_ERROR_STREAM("Robot model is null!");
    }
  }

  bool operator()(double const* const* parameters, double* cost) const {
    // Copy parameters into full joint state
    std::vector<double> joint_angles = initial_full_state_;
    for (unsigned int i = 0; i < param_idx_to_joint_idx_.size(); i++) {
      joint_angles[param_idx_to_joint_idx_[i]] = parameters[0][i];
    }

//    ROS_INFO_STREAM("Current robot_pose:" << robot_pose_->affine());
    robot_model_->updateRobotState(*robot_pose_, joint_angles);

    // Compute sphere position in world
    const Eigen::Vector3d& sphere_center = obj_.collision_sphere.relative_vec_;
    const Eigen::Affine3d& world_to_link = robot_model_->getMoveitRobotState().getGlobalLinkTransform(obj_.link_name);
    Eigen::Vector3d sphere_center_world = world_to_link * sphere_center;

    // Evaluate sdf at sphere position
    double sdf = sdf_->getSdf<double>(sphere_center_world.x(), sphere_center_world.y(), sphere_center_world.z());

    // sdf - r > 0
    cost[0] = -(sdf - obj_.collision_sphere.radius_ - obj_.padding); // inequality constraints in nlopt have to be < 0
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<std::size_t>& param_idx_to_joint_idx,
                                     const std::vector<double>& initial_full_state, const Eigen::Affine3d* robot_pose, const sdf_contact_estimation::SdfWrapper& sdf,
                                     const CollisionObject& obj) {
    ceres::DynamicNumericDiffCostFunction<SdfContactConstraintNoGradient>* cost_function = new ceres::DynamicNumericDiffCostFunction<SdfContactConstraintNoGradient>(
          new SdfContactConstraintNoGradient(robot_model, param_idx_to_joint_idx, initial_full_state, robot_pose, sdf, obj));

    cost_function->AddParameterBlock(param_idx_to_joint_idx.size());
    cost_function->SetNumResiduals(1);

    return static_cast<ceres::CostFunction*>(cost_function);
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_joint_idx_; // Map parameter index to joint index
  std::vector<double> initial_full_state_; // default values for all joints
  const Eigen::Affine3d* robot_pose_; // current robot pose in world
  const sdf_contact_estimation::SdfWrapper* sdf_;

  CollisionObject obj_;
};

}

#endif
