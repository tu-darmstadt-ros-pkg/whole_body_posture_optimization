#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_ROBOT_MODEL_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_ROBOT_MODEL_H

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <Eigen/Eigen>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/collision_distance_field/collision_distance_field_types.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>

#include <ad_kinematics/tree.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>

#include "whole_body_posture_optimization/util/types.h"

namespace whole_body_posture_optimization {

class RobotModel {
public:
  RobotModel(const rclcpp::Node::SharedPtr& node);

  template <typename T>
  Vector3<T> computeCenterOfMass(const std::vector<T> &joint_angles) {
    T total_mass = T(0.0);
    Vector3<T> com = computeCenterOfMassRecursive(urdf_model_->root_link_, joint_angles, Affine3<T>::Identity(), total_mass);
    com /= total_mass;
    return com;
  }

  template<typename T> Affine3<T> getLinkTransform(const std::string& link_name, const std::vector<T>& actuated_joint_positions) {
    return ad_tree_->computeTransform<T>(link_name, actuated_joint_positions).toAffine();
  }


  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance, float voxel_size,
                                                              const std::vector<double>& joint_state=std::vector<double>());
  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance,
                                                              const std::shared_ptr<voxblox::TsdfMap>& init_tsdf, const std::vector<double>& joint_state=std::vector<double>());

  void updateJointAngles(const std::vector<double>& joint_angles);
  void updatePose(const Eigen::Isometry3d& robot_pose);
  void updateRobotState(const Eigen::Isometry3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState(const Eigen::Isometry3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState();

  std::vector<std::string> getActiveJointNames() const;
  std::vector<std::size_t> getActiveJointIndices(const std::vector<std::string>& joints) const;
  std::string getParentJointName(const std::string& link_name);
  bool getJointLimits(const std::string& joint_name, double& lower_limit, double& upper_limit);
  unsigned int getNumJoints() const;

  std::vector<std::string> getCollisionLinkNames();
  std::vector<std::string> getCollisionGroupNames();
  const std::vector<collision_detection::CollisionSphere>& getCollisionSpheres(const std::string& link_name);

  moveit::core::RobotState getMoveitRobotState() const;
  moveit::core::RobotModelPtr getMoveitRobotModel() const;
private:
  template <typename T>
  Vector3<T> computeCenterOfMassRecursive(const urdf::LinkSharedPtr& link, const std::vector<T> &joint_angles, const Affine3<T>& transform, T& total_mass) {
    Vector3<T> com = Vector3<T>::Zero();
    if (link->inertial && link->inertial->mass > 0) {
      T mass = T(link->inertial->mass);
      Vector3<T> link_com;
      link_com.x() = T(link->inertial->origin.position.x);
      link_com.y() = T(link->inertial->origin.position.y);
      link_com.z() = T(link->inertial->origin.position.z);
      com = transform * link_com * mass;
      total_mass += mass;
    }
    for (auto const& child_link: link->child_links) {
      Affine3<T> child_transform = transform * ad_tree_->getLink(child_link->name)->pose(joint_angles).toAffine();
      com += computeCenterOfMassRecursive(child_link, joint_angles, child_transform, total_mass);
    }
    return com;
  }
  void generateLinkBodyDecompositions(double resolution);

  rclcpp::Node::SharedPtr node_;
  std::string param_prefix_;

  std::vector<collision_detection::BodyDecompositionConstPtr> body_decompositions_;
  std::map<std::string, unsigned int> body_decomposition_index_map_;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  moveit::core::RobotModelPtr moveit_robot_model_;

  /// Contains all joints (revolute, prismatic, fixed, continuous, mimic)
  std::vector<std::string> joint_names_;
  /// Only active joints (no fixed, no mimic)
  std::vector<std::string> active_joint_names_;


  moveit::core::RobotStatePtr moveit_robot_state_;
  Eigen::Isometry3d robot_pose_;
  std::vector<double> joint_angles_;

  std::shared_ptr<ad_kinematics::Tree> ad_tree_;
  std::string collision_group_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
};

}

#endif
