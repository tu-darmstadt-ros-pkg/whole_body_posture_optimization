#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const std::string &robot_description, const sdf_contact_estimation::RobotShape &robot_shape)
  : contact_estimation_(robot_shape), arm_optimization_(robot_description)
{
  // load urdf
}

bool WholeBodyPostureOptimization::loadOptimizationParameters(ros::NodeHandle &nh)
{
  if (!nh.getParam("default_joint_state", default_joint_state_)) {
    ROS_ERROR_STREAM("No default joint state found.");
    return false;
  }
}



void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf)
{
  tsdf_ = tsdf;
  contact_estimation_.loadTsdf(tsdf, true);
}

void WholeBodyPostureOptimization::updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  contact_estimation_.loadCloud(cloud, true);
}

void WholeBodyPostureOptimization::setDefaultJointPositions(const std::vector<double>& default_joint_positions)
{
  default_joint_state_ = default_joint_positions;
}

std::vector<std::vector<double>> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path)
{
  std::vector<std::vector<double>> postures;
  std::vector<double> initial_joint_state = default_joint_state_;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    std::vector<double> posture = findOptimalPosture(pose, initial_joint_state);
    initial_joint_state = posture; // initialize next time step with current solution
    postures.push_back(posture);
  }

  return postures;
}

std::vector<double> WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& joint_state)
{
  Eigen::Affine3d base_to_com; //TODO compute from default arm posture
  std::vector<Eigen::Vector3d> contact_points = contact_estimation_.estimateSupportPolygon(pose, base_to_com);
  ArmOptResult result = arm_optimization_.optimize(joint_state, contact_points);

  return result.joint_angles;
}

}
