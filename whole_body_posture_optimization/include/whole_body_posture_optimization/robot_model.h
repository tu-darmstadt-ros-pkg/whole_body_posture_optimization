#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_ROBOT_MODEL_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_ROBOT_MODEL_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <Eigen/Eigen>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>

#include <ad_kinematics/tree.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/conversions.h>

#include <whole_body_posture_optimization/types.h>

namespace whole_body_posture_optimization {

struct JointMimic {
  JointMimic(unsigned int _joint_index, double _offset, double _multiplier)
    : joint_index(_joint_index), offset(_offset), multiplier(_multiplier) {}
  unsigned int joint_index; // target index
  double offset;
  double multiplier;
};

class RobotModel {
public:
  RobotModel(ros::NodeHandle& nh);

  template <typename T>
  Vector3<T> computeCenterOfMass(const std::vector<T> &joint_angles) {
    std::vector<T> full_joint_state = getFullStateFromActuated(joint_angles);

    T total_mass = T(0.0);
    Vector3<T> com = computeCenterOfMassRecursive(urdf_model_->root_link_, full_joint_state, Affine3<T>::Identity(), total_mass);
    com /= total_mass;
    return com;
  }

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


  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance, float voxel_size,
                                                              const std::vector<double>& joint_state=std::vector<double>());
  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance,
                                                              const std::shared_ptr<voxblox::TsdfMap>& init_tsdf, const std::vector<double>& joint_state=std::vector<double>());

  void updateJointAngles(const std::vector<double>& joint_angles);
  void updatePose(const Eigen::Isometry3d& robot_pose);
  void updateRobotState(const Eigen::Isometry3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState(const Eigen::Isometry3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState();
  void publishCollisionSpheres(const Eigen::Isometry3d& robot_pose, const std::vector<double> joint_angles);
  void publishCollisionSpheres();

  std::vector<std::size_t> getActuatedJointIndices(const std::vector<std::string>& joints) const;
  std::string getParentJointName(const std::string& link_name);
  bool getJointLimits(const std::string& joint_name, double& lower_limit, double& upper_limit);
  unsigned int getNumJoints() const;

  std::vector<std::string> getCollisionLinkNames();
  std::vector<std::string> getCollisionGroupNames();
  const std::vector<collision_detection::CollisionSphere>& getCollisionSpheres(const std::string& link_name);

  template<typename T> Affine3<T> computeChainFK(const std::string& link_name, const std::vector<T>& chain_angles) {
    return ad_tree_->computeTransform<T>(link_name, chain_angles).toAffine();
  }

  std::shared_ptr<ad_kinematics::Tree> getADTree() const;

  std::vector<std::size_t> getChainToJointIndexMapping();
  std::vector<int> getParamToChainIndexMapping(const std::vector<std::string>& free_joints);

  robot_state::RobotState getMoveitRobotState() const;

  robot_model::RobotModelPtr getMoveitRobotModel() const;

  template <typename T>
  void setJointValuesFromActuated(unsigned int actuated_idx, T value, std::vector<T>& joint_state) const {
    joint_state[actuated_idx_to_joint_idx_[actuated_idx]] = value;
    for (const JointMimic& mimic: actuated_joint_mimics_[actuated_idx]) {
      joint_state[mimic.joint_index] = value * T(mimic.multiplier) + T(mimic.offset);
    }
  }
  template <typename T>
  std::vector<T> getFullStateFromActuated(const std::vector<T>& actuated_joint_state) const {
    std::vector<T> full_joint_state(num_joints_, T(0.0));
    for (unsigned int i = 0; i < actuated_joint_state.size(); i++) {
      setJointValuesFromActuated<T>(i, actuated_joint_state[i], full_joint_state);
    }
    return full_joint_state;
  }

  double getTotalMass() const;

  std::vector<std::string> getActuatedJointNames() const;

private:
  void generateLinkBodyDecompositions(double resolution);
  void updateActuatedJointMapping(const std::string& joint_name, unsigned int joint_index);
  void updateJointMimicMapping(const std::string& joint_name, unsigned int joint_index);

  ros::NodeHandle nh_;
  std::string robot_description_;

  std::vector<collision_detection::BodyDecompositionConstPtr> body_decompositions_;
  std::map<std::string, unsigned int> body_decomposition_index_map_;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  robot_model::RobotModelPtr moveit_robot_model_;

  /// Contains all joints (revolute, prismatic, fixed, continuous, mimic)
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;
  /// Only actuated joints (no fixed, no mimic)
  std::vector<std::string> actuated_joint_names_;
  /// Maps actuated joints to the full joint index of rbdl (contains fixed joints, mimics..)
  std::vector<unsigned int> actuated_idx_to_joint_idx_;
  /// One entry for each actuated joint, multiple entries for ever joint that mimics the actuated joint
  std::vector<std::vector<JointMimic>> actuated_joint_mimics_;

  robot_state::RobotStatePtr moveit_robot_state_;
  Eigen::Isometry3d robot_pose_;
  std::vector<double> joint_angles_;

  std::shared_ptr<ad_kinematics::Tree> ad_tree_;
  std::string collision_group_;

  ros::Publisher robot_state_pub_;
  ros::Publisher collision_spheres_pub_;

  double total_mass_;
};

}

#endif
