#include <whole_body_posture_optimization/robot_model.h>

namespace whole_body_posture_optimization {

RobotModel::RobotModel(const std::string &robot_description)
  : num_joints(0)
{
  ROS_INFO_STREAM("Loading urdf model");
  if (!urdf_model_.initString(robot_description)) {
    ROS_ERROR_STREAM("Failed to parse urdf.");
  }

  model_ptr_ = std::make_shared<rbdl::Model>();
  rbdl::Addons::URDFReadFromString(robot_description.c_str(), model_ptr_.get(), true, true);
  ad_model_ptr_ = std::make_shared<ADModel>(*model_ptr_);
  ROS_INFO_STREAM("Loaded model:");
  ROS_INFO_STREAM(" -- dof: " << model_ptr_->dof_count);
  ROS_INFO_STREAM(" -- q_size: " << model_ptr_->q_size);
  num_joints = model_ptr_->dof_count;
  for (unsigned int body_id = 0; body_id < model_ptr_->mBodies.size(); body_id++) {
    std::string body_name = model_ptr_->GetBodyName(body_id);
    ROS_INFO_STREAM("Body id: " << body_id << ", name: " << body_name);
    ROS_INFO_STREAM("-- Parent joint: " << getParentJointName(body_name));
    ROS_INFO_STREAM("-- Joint type: " << model_ptr_->mJoints[body_id].mJointType);
  }
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com, Eigen::MatrixXd &jacobian)
{
  if (joint_angles.size() != num_joints) {
    ROS_ERROR_STREAM("Joint positions size doesn't match number of joints");
    return;
  }
  rbdl::Math::VectorNd q(joint_angles.size());
  rbdl::Math::VectorNd qdot(joint_angles.size());
  for (unsigned int i = 0; i < joint_angles.size(); i++) {
    q(i) = joint_angles[i];
    qdot(i) = 0;
  }
  int ndirs = num_joints;
  rbdl::Math::MatrixNd q_dirs = rbdl::Math::MatrixNd::Zero(num_joints, ndirs);
  rbdl::Math::MatrixNd qdot_dirs = rbdl::Math::MatrixNd::Zero(num_joints, ndirs);
  q_dirs.block(0, 0, num_joints, num_joints) = rbdl::Math::MatrixNd::Identity(num_joints, num_joints);
//  q_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Zero(ndof, ndof);
  qdot_dirs.block(0, 0, num_joints, num_joints) = rbdl::Math::MatrixNd::Zero(num_joints, num_joints);
//  qdot_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Identity(ndof, ndof);

  double mass;
  rbdl::Math::Vector3d com2;
  jacobian = rbdl::Math::MatrixNd(3, ndirs);
  rbdl::AD::CalcCenterOfMass(*model_ptr_, *ad_model_ptr_, q, q_dirs, qdot, qdot_dirs, mass, com2, jacobian, 0, 0, 0, 0, true);
  com = com2;
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com)
{
  Eigen::MatrixXd jacobian;
  computeCenterOfMass(joint_angles, com, jacobian);
}

std::vector<unsigned int> RobotModel::getBodyIndices(const std::vector<std::string> &bodies)
{
  std::vector<unsigned int> indices;
  for (const std::string& body: bodies) {
    int idx = model_ptr_->GetBodyId(body.c_str());
    if (idx == std::numeric_limits<unsigned int>::max()) {
      ROS_ERROR_STREAM("Body '" << body << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
}

std::string RobotModel::getParentJointName(const std::string &link_name)
{
  boost::shared_ptr<const urdf::Link> link = urdf_model_.getLink(link_name);
  if (!link) {
    ROS_ERROR_STREAM("Couldn't find link '" << link_name << "'.");
    return "NOT FOUND";
  }
  return link->parent_joint->name;
}

bool RobotModel::getJointLimits(const std::string &link_name, double &lower_limit, double &upper_limit)
{
  boost::shared_ptr<const urdf::Link> link = urdf_model_.getLink(link_name);
  if (!link) {
    ROS_ERROR_STREAM("Couldn't find link '" << link_name << "'.");
    return false;
  }
  urdf::JointSharedPtr joint = link->parent_joint;
  upper_limit = joint->limits->upper;
  lower_limit = joint->limits->lower;
  return true;
}

unsigned int RobotModel::getNumJoints() const
{
  return num_joints;
}

}
