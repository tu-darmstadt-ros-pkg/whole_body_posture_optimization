#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(robot_description_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(robot_shape_gen->getShape());
//  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_);
  flipper_optimization_ = std::make_shared<FlipperOptimization>(robot_shape_gen, contact_estimation_);
}

bool WholeBodyPostureOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("/robot_description", robot_description_)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
    return false;
  }

  ROS_INFO_STREAM("Loading optimization parameters..");
  if (!nh.getParam("default_joint_state", default_joint_state_)) {
    ROS_ERROR_STREAM("No default joint state found.");
    return false;
  }

  return true;
}



void WholeBodyPostureOptimization::updateWorldModel(const std::shared_ptr<voxblox::TsdfMap>& tsdf)
{
  tsdf_ = tsdf;
  contact_estimation_->loadTsdf(tsdf, false);
}

void WholeBodyPostureOptimization::updateWorldModel(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  contact_estimation_->loadCloud(cloud, false);
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
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity(); // TODO compute com, take solution from previous time step
  base_to_com.translation() = Eigen::Vector3d(-0.15, 0, 0);
  std::vector<double> init_flipper_angles(1, 0); // TODO read from parameter space?
  return flipper_optimization_->optimize(pose, base_to_com, init_flipper_angles);
//  Eigen::Vector3d com;
//  robot_model_->computeCenterOfMass(joint_state, com);
//  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
//  base_to_com.translation() = com;
//  std::vector<Eigen::Vector3d> contact_points = contact_estimation_.estimateSupportPolygon(pose, base_to_com);
//  ArmOptResult result = arm_optimization_->optimize(joint_state, contact_points);

//  return result.joint_angles;
}

}
