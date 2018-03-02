#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__SELF_CONTACT_CONSTRAINT_NO_GRADIENT
#define WHOLE_BODY_POSTURE_OPTIMIZATION__SELF_CONTACT_CONSTRAINT_NO_GRADIENT

#include <whole_body_posture_optimization/common.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <algorithm>
#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <math.h>

namespace  whole_body_posture_optimization {

class SelfContactConstraintNoGradient {
public:
  SelfContactConstraintNoGradient(const std::shared_ptr<RobotModel> &robot_model, const std::vector<unsigned int>& param_idx_to_joint_idx,
                                  const std::vector<double>& initial_full_state, const Eigen::Affine3d& robot_pose, const CollisionObject& obj1, const CollisionObject& obj2)
    : robot_model_(robot_model), param_idx_to_joint_idx_(param_idx_to_joint_idx), initial_full_state_(initial_full_state), robot_pose_(robot_pose), obj1_(obj1), obj2_(obj2)
  {
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

    robot_model_->updateJointAngles(joint_angles);

    // Compute sphere position in world
    const Eigen::Vector3d& sphere_center1 = obj1_.collision_sphere.relative_vec_;
    const Eigen::Affine3d& base_to_link1 = robot_model_->getMoveitRobotState().getGlobalLinkTransform(obj1_.link_name);
    Eigen::Vector3d sphere_center_base1 = base_to_link1 * sphere_center1;

    // Compute sphere position in world
    const Eigen::Vector3d& sphere_center2 = obj2_.collision_sphere.relative_vec_;
    const Eigen::Affine3d& base_to_link2 = robot_model_->getMoveitRobotState().getGlobalLinkTransform(obj2_.link_name);
    Eigen::Vector3d sphere_center_base2 =  base_to_link2 * sphere_center2;



    double distance = (sphere_center_base1 - sphere_center_base2).norm();

    // d - r1 - p1 - r2 - p2 > 0
    cost[0] = -(distance - obj1_.collision_sphere.radius_ - obj1_.padding - obj2_.collision_sphere.radius_ - obj2_.padding); // inequality constraints in nlopt have to be < 0
    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<RobotModel> &robot_model, const std::vector<unsigned int>& param_idx_to_joint_idx,
                                     const std::vector<double>& initial_full_state, const Eigen::Affine3d& robot_pose, const CollisionObject& obj1, const CollisionObject& obj2) {
    ceres::DynamicNumericDiffCostFunction<SelfContactConstraintNoGradient>* cost_function = new ceres::DynamicNumericDiffCostFunction<SelfContactConstraintNoGradient>(
          new SelfContactConstraintNoGradient(robot_model, param_idx_to_joint_idx, initial_full_state, robot_pose, obj1, obj2));

    cost_function->AddParameterBlock(param_idx_to_joint_idx.size());
    cost_function->SetNumResiduals(1);

    return static_cast<ceres::CostFunction*>(cost_function);
  }

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_joint_idx_; // Transforms parameter index to chain index
  std::vector<double> initial_full_state_; // default values for all chain joints
  Eigen::Affine3d robot_pose_; // current robot pose in world

  CollisionObject obj1_;
  CollisionObject obj2_;
};

}

#endif
