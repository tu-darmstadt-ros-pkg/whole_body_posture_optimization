#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const std::string &robot_description, const sdf_contact_estimation::RobotShape &robot_shape)
  : contact_estimation_(robot_shape)
{
  // load urdf
}



void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap> &tsdf)
{
  tsdf_ = tsdf;
}

void WholeBodyPostureOptimization::setDefaultJointPositions(const std::vector<double> &default_joint_positions)
{
  default_joint_positions_ = default_joint_positions;
}

std::vector<std::vector<double>> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path)
{
  std::vector<std::vector<double>> postures;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    std::vector<double> posture = findOptimalPosture(pose);
    postures.push_back(posture);
  }

  return postures;
}

std::vector<double> WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose)
{

}

}
