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

#undef LOG // of course everyone names their logging macro the same
#include <differentiation/rbdl_utilsAD.h>
#include <urdfreader/urdfreader.h>

namespace rbdl = RigidBodyDynamics;

namespace whole_body_posture_optimization {

class RobotModel {
public:
  RobotModel(const std::string& robot_description);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com, Eigen::MatrixXd &jacobian);
  void computeCenterOfMass(const std::vector<double>& joint_angles, Eigen::Vector3d& com);

  void publishRobotState(const Eigen::Affine3d& robot_pose, const std::vector<double> joint_angles);
  void publishCollisionSpheres(const Eigen::Affine3d& robot_pose, const std::vector<double> joint_angles, std::string collision_group);

  std::vector<unsigned int> getBodyIndices(const std::vector<std::string>& bodies);
  std::vector<unsigned int> getJointIndices(const std::vector<std::string>& joints);
  std::string getParentJointName(const std::string& link_name);
  bool getJointLimits(const std::string& joint_name, double& lower_limit, double& upper_limit);
  unsigned int getNumJoints() const;

private:
  void generateLinkBodyDecompositions(double resolution);

  std::vector<collision_detection::BodyDecompositionConstPtr> body_decompositions_;
  std::map<std::string, unsigned int> body_decomposition_index_map_;

  std::shared_ptr<rbdl::Model> model_ptr_;
  std::shared_ptr<ADModel> ad_model_ptr_;
  urdf::Model urdf_model_;
  unsigned int num_joints_;
  std::vector<std::string> joint_names_;

  robot_state::RobotStatePtr moveit_robot_state_;
  robot_model::RobotModelPtr moveit_robot_model_;
  ros::Publisher robot_state_pub_;
  ros::Publisher collision_spheres_pub_;

};

}

#endif
