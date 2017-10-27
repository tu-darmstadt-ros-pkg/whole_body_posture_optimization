#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const std::string &robot_description, const sdf_contact_estimation::RobotShape &robot_shape)
  : contact_estimation_(robot_shape)
{
  // load urdf
}

std::vector<std::vector<double>> findOptimalPosture(const nav_msgs::Path& path)
{
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);
    
    
  }
}



void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap> &tsdf)
{
  tsdf_ = tsdf;
}

void WholeBodyPostureOptimization::setDefaultJointPositions(const std::vector<double> &default_joint_positions)
{
  default_joint_positions_ = default_joint_positions;
}

}
