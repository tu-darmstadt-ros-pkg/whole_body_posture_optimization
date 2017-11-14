#include <whole_body_posture_optimization/center_of_mass.h>

namespace whole_body_posture_optimization {

CenterOfMass::CenterOfMass(const std::string &robot_description)
{
  rbdl::Addons::URDFReadFromString(robot_description.c_str(), model_ptr_.get(), true, false);
  ad_model_ptr_ = std::make_shared<ADModel>(*model_ptr_);
  ROS_INFO_STREAM("Loaded model:");
  ROS_INFO_STREAM(" -- dof: " << model_ptr_->dof_count);
  for (unsigned int body_id = 0; body_id < model_ptr_->mBodies.size(); body_id++) {
    ROS_INFO_STREAM("Body id: " << body_id << ", name: " << model_ptr_->GetBodyName(body_id));
  }
}

void CenterOfMass::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com, Eigen::MatrixXd &jacobian)
{
  rbdl::Math::VectorNd q(joint_angles.size());
  rbdl::Math::VectorNd qdot(joint_angles.size());
  for (unsigned int i = 0; i < joint_angles.size(); i++) {
    q(i) = joint_angles[i];
    qdot(i) = 0;
  }

  int ndof = joint_angles.size();
  int ndirs = ndof;
  rbdl::Math::MatrixNd q_dirs = rbdl::Math::MatrixNd::Zero(ndof, ndirs);
  rbdl::Math::MatrixNd qdot_dirs = rbdl::Math::MatrixNd::Zero(ndof, ndirs);
  q_dirs.block(0, 0, ndof, ndof) = rbdl::Math::MatrixNd::Identity(ndof, ndof);
//  q_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Zero(ndof, ndof);
  qdot_dirs.block(0, 0, ndof, ndof) = rbdl::Math::MatrixNd::Zero(ndof, ndof);
//  qdot_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Identity(ndof, ndof);

  double mass;
  rbdl::Math::Vector3d com2;
  jacobian = rbdl::Math::MatrixNd(3, ndirs);
  rbdl::AD::CalcCenterOfMass(*model_ptr_, *ad_model_ptr_, q, q_dirs, qdot, qdot_dirs, mass, com2, jacobian, 0, 0, 0, 0, true);
}

void CenterOfMass::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com)
{

}

}
