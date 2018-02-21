#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__ROBOT_MODEL_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__ROBOT_MODEL_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>

#include <ad_kinematics/chain.h>

#undef LOG // of course everyone names their logging macro the same
#include <differentiation/rbdl_utilsAD.h>
#include <urdfreader/urdfreader.h>

#undef LOG
#include <voxblox/core/tsdf_map.h>

#include <whole_body_posture_optimization/common.h>
#include <sdf_contact_estimation/visualisation.h>

namespace rbdl = RigidBodyDynamics;

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
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com, Eigen::MatrixXd &jacobian);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com);

  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance);
  std::shared_ptr<voxblox::TsdfMap> computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance, const std::shared_ptr<voxblox::TsdfMap>& init_tsdf);

  void updateRobotState(const Eigen::Affine3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState(const Eigen::Affine3d& robot_pose, const std::vector<double> joint_angles);
  void publishRobotState();
  void publishCollisionSpheres(const Eigen::Affine3d& robot_pose, const std::vector<double> joint_angles);
  void publishCollisionSpheres();

  std::vector<unsigned int> getBodyIndices(const std::vector<std::string>& bodies);
  std::vector<unsigned int> getActuatedJointIndices(const std::vector<std::string>& joints);
  std::string getParentJointName(const std::string& link_name);
  bool getJointLimits(const std::string& joint_name, double& lower_limit, double& upper_limit);
  unsigned int getNumJoints() const;

  std::vector<std::string> getCollisionLinkNames();
  std::vector<std::string> getCollisionGroupNames();
  const std::vector<collision_detection::CollisionSphere> &getCollisionSpheres(std::string link_name);

  template<typename T> ad_kinematics::Affine3<T> computeChainFK(const std::string& link_name, const std::vector<T>& chain_angles) {
    return ad_chain_->computeTransform<T>(link_name, chain_angles).toAffine();
  }

  std::vector<unsigned int> getChainToJointIndexMapping();
  std::vector<int> getParamToChainIndexMapping(const std::vector<std::string>& free_joints);

  robot_state::RobotState getMoveitRobotState() const;

  robot_model::RobotModelPtr getMoveitRobotModel() const;

  void setJointValuesFromActuated(double actuated_idx, double value, std::vector<double>& joint_state);

private:
  void generateLinkBodyDecompositions(double resolution);
  void updateActuatedJointMapping(const std::string& joint_name, unsigned int joint_index);
  void updateJointMimicMapping(const std::string& joint_name, unsigned int joint_index);

  std::string robot_description_;

  std::vector<collision_detection::BodyDecompositionConstPtr> body_decompositions_;
  std::map<std::string, unsigned int> body_decomposition_index_map_;

  std::shared_ptr<rbdl::Model> model_ptr_;
  std::shared_ptr<ADModel> ad_model_ptr_;
  urdf::Model urdf_model_;
  robot_model::RobotModelPtr moveit_robot_model_;

  /// Contains all joints (revolute, prismatic, fixed, continuous, mimic)
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;
  /// Only actuated joints (no fixed, no mimic)
  std::vector<std::string> actuated_joint_names_;
  unsigned int num_actuated_joints_;
  /// Maps actuated joints to the full joint index of rbdl (contains fixed joints, mimics..)
  std::vector<unsigned int> actuated_idx_to_joint_idx_;
  /// One entry for each actuated joint, multiple entries for ever joint that mimics the actuated joint
  std::vector<std::vector<JointMimic>> actuated_joint_mimics_;

  robot_state::RobotStatePtr moveit_robot_state_;
  Eigen::Affine3d robot_pose_;
  std::vector<double> joint_angles_;

  std::shared_ptr<ad_kinematics::Chain> ad_chain_;
  std::string collision_group_;

  ros::Publisher robot_state_pub_;
  ros::Publisher collision_spheres_pub_;
};

}

#endif
