#include <whole_body_posture_optimization/robot_model.h>

namespace whole_body_posture_optimization {

RobotModel::RobotModel(const std::string &robot_description)
  : num_joints_(0)
{
  // Loading urdf
  if (!urdf_model_.initString(robot_description)) {
    ROS_ERROR_STREAM("Failed to parse urdf.");
  }

  // Loading rbdl model
  model_ptr_ = std::make_shared<rbdl::Model>();
  rbdl::Addons::URDFReadFromString(robot_description.c_str(), model_ptr_.get(), false, false);
  ad_model_ptr_ = std::make_shared<ADModel>(*model_ptr_);
  ROS_INFO_STREAM("Loaded model:");
  ROS_INFO_STREAM(" -- dof: " << model_ptr_->dof_count);
  ROS_INFO_STREAM(" -- q_size: " << model_ptr_->q_size);
  num_joints_ = model_ptr_->dof_count;
  for (unsigned int body_id = 0; body_id < model_ptr_->mBodies.size(); body_id++) {
    std::string body_name = model_ptr_->GetBodyName(body_id);
    ROS_INFO_STREAM("Body id: " << body_id << ", name: " << body_name);
    ROS_INFO_STREAM("-- mass: " << model_ptr_->mBodies[body_id].mMass);
    if (body_id != 0) {
      ROS_INFO_STREAM("-- Parent joint: " << getParentJointName(body_name));
      ROS_INFO_STREAM("-- Joint type: " << model_ptr_->mJoints[body_id].mJointType);
      joint_names_.push_back(getParentJointName(body_name));
    }
  }

  // Loading moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit_robot_model_ = robot_model_loader.getModel();
  if (!moveit_robot_model_.get()){
    ROS_ERROR("Couldn't load robot model!");
  }
  moveit_robot_state_ = std::make_shared<robot_state::RobotState>(moveit_robot_model_);

  ros::NodeHandle pnh("~");
  robot_state_pub_ = pnh.advertise<moveit_msgs::DisplayRobotState>("robot_state", 10, true);
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com, Eigen::MatrixXd &jacobian)
{
  if (joint_angles.size() != num_joints_) {
    ROS_ERROR_STREAM("[RobotModel::computeCenterOfMass] Joint positions size (" << joint_angles.size() << ") doesn't match number of joints in robot model (" << num_joints_ << ")");
    return;
  }
  rbdl::Math::VectorNd q(joint_angles.size());
  rbdl::Math::VectorNd qdot(joint_angles.size());
  for (unsigned int i = 0; i < joint_angles.size(); i++) {
//    ROS_INFO_STREAM("Body name: " << joint_names_[i] << ", " << joint_angles[i]);
    q(i) = joint_angles[i];
    qdot(i) = 0;
  }
  int ndirs = num_joints_;
  rbdl::Math::MatrixNd q_dirs = rbdl::Math::MatrixNd::Zero(num_joints_, ndirs);
  rbdl::Math::MatrixNd qdot_dirs = rbdl::Math::MatrixNd::Zero(num_joints_, ndirs);
  q_dirs.block(0, 0, num_joints_, num_joints_) = rbdl::Math::MatrixNd::Identity(num_joints_, num_joints_);
//  q_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Zero(ndof, ndof);
  qdot_dirs.block(0, 0, num_joints_, num_joints_) = rbdl::Math::MatrixNd::Zero(num_joints_, num_joints_);
//  qdot_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Identity(ndof, ndof);

  double mass;
  rbdl::Math::Vector3d com2;
  jacobian = rbdl::Math::MatrixNd(3, ndirs);
  rbdl::AD::CalcCenterOfMass(*model_ptr_, *ad_model_ptr_, q, q_dirs, qdot, qdot_dirs, mass, com2, jacobian, 0, 0, 0, 0, true);
  com = com2;

  // Fix because rbdl ignores root body
  rbdl::Body root = model_ptr_->mBodies[0];
  com = (mass * com + root.mMass * root.mCenterOfMass)/(mass + root.mMass);
  mass += root.mMass;

//  ROS_INFO_STREAM("mass: " << mass);
//  ROS_INFO_STREAM("com: " << com);
//  ROS_INFO_STREAM("jacobian:" << std::endl << jacobian);
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com)
{
  Eigen::MatrixXd jacobian;
  computeCenterOfMass(joint_angles, com, jacobian);
}

void RobotModel::publishRobotState(const Eigen::Affine3d &robot_pose, const std::vector<double> joint_angles)
{
  if (joint_angles.size() != num_joints_) {
    ROS_ERROR_STREAM("[RobotModel::publishRobotState] joint_angles size (" << joint_angles.size() << ") doesn't match number of joints in robot model (" << num_joints_ << ")");
    return;
  }
  sensor_msgs::JointState virtual_link_joint_states;
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/trans_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_x");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_y");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_z");
  virtual_link_joint_states.name.push_back("world_virtual_joint/rot_w");

  virtual_link_joint_states.position.resize(7);

  virtual_link_joint_states.position[0] = robot_pose.translation().x();
  virtual_link_joint_states.position[1] = robot_pose.translation().y();
  virtual_link_joint_states.position[2] = robot_pose.translation().z();
  Eigen::Quaterniond quat(robot_pose.rotation());
  virtual_link_joint_states.position[3] = quat.x();
  virtual_link_joint_states.position[4] = quat.y();
  virtual_link_joint_states.position[5] = quat.z();
  virtual_link_joint_states.position[6] = quat.w();
  moveit::core::jointStateToRobotState(virtual_link_joint_states, *moveit_robot_state_);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names_;
  joint_state_msg.position = joint_angles;
  moveit::core::jointStateToRobotState(joint_state_msg, *moveit_robot_state_);

  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*moveit_robot_state_, robot_state_msg.state);
  robot_state_pub_.publish(robot_state_msg);
}

std::vector<unsigned int> RobotModel::getBodyIndices(const std::vector<std::string> &bodies)
{
  std::vector<unsigned int> indices;
  for (const std::string& body: bodies) {
    unsigned int idx = model_ptr_->GetBodyId(body.c_str());
    if (idx == std::numeric_limits<unsigned int>::max()) {
      ROS_ERROR_STREAM("Body '" << body << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
  return indices;
}

std::vector<unsigned int> RobotModel::getJointIndices(const std::vector<std::string> &joints)
{
  std::vector<unsigned int> indices;
  for (const std::string& joint: joints) {
    ptrdiff_t idx = std::find(joint_names_.begin(), joint_names_.end(), joint) - joint_names_.begin();
    if (idx >= joint_names_.size()) {
      ROS_ERROR_STREAM("Joint '" << joint << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
  return indices;
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

bool RobotModel::getJointLimits(const std::string &joint_name, double &lower_limit, double &upper_limit)
{
  urdf::JointConstSharedPtr joint = urdf_model_.getJoint(joint_name);
  if (!joint) {
    ROS_ERROR_STREAM("Couldn't find joint '" << joint_name << "'.");
    return false;
  }
  upper_limit = joint->limits->upper;
  lower_limit = joint->limits->lower;
  return true;
}

unsigned int RobotModel::getNumJoints() const
{
  return num_joints_;
}

}
