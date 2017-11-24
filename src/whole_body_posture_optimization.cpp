#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(robot_description_);
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(robot_shape_gen);
  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, robot_shape_gen, contact_estimation_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_);
}

bool WholeBodyPostureOptimization::loadParameters(const ros::NodeHandle &nh)
{
  if(!nh.getParam("/robot_description", robot_description_)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
    return false;
  }

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

const std::vector<double> &WholeBodyPostureOptimization::getDefaultJointPositions()
{
  return default_joint_state_;
}

std::vector<std::vector<double>> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path)
{
  std::vector<std::vector<double>> postures;
  std::vector<double> initial_joint_state = default_joint_state_;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

//    std::vector<double> posture = findOptimalPosture(pose, initial_joint_state);
//    initial_joint_state = posture; // initialize next time step with current solution
//    postures.push_back(posture);
  }

  return postures;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_state,
                                                                           const sdf_contact_estimation::ContactEstimationResult* init_contact_result)
{
  PostureOptimizationResult result;
  result.init_joint_angles = init_joint_state;

//  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity(); // TODO compute com, take solution from previous time step
//  base_to_com.translation() = Eigen::Vector3d(-0.15, 0, 0);

  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
  robot_model_->computeCenterOfMass(init_joint_state, result.init_com);
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = result.init_com;

  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
  if (init_contact_result != NULL) {
    result.init_contact = *init_contact_result;
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_state, pose, base_to_com, result.init_contact);
  } else {
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_state, pose, base_to_com);
  }

  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(pose, base_to_com, init_joint_state, updated_init_contact);
  result.result_contact = flipper_result.contact_result;

  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points, flipper_result.contact_result.estimated_pose);
  result.result_joint_angles = arm_result.joint_angles;
  result.result_com = arm_result.center_of_mass;

  robot_model_->publishRobotState(result.result_contact.estimated_pose, result.result_joint_angles);

  return result;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose, const std::vector<double> &init_joint_state)
{
  return findOptimalPosture(pose, init_joint_state, NULL);
}

}
