#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(robot_description_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(robot_shape_gen);
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

  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity(); // TODO compute com, take solution from previous time step

  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
//  Eigen::Vector3d com;
//  robot_model_->computeCenterOfMass(init_joint_state, com);
//  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
//  base_to_com.translation() = com;
//  base_to_com.translation() = Eigen::Vector3d(-0.15, 0, 0);
  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
  if (init_contact_result != NULL) {
    result.init_contact = *init_contact_result;
    updated_init_contact = contact_estimation_->estimateSupportPolygon(pose, base_to_com, result.init_contact);
  } else {
    updated_init_contact = contact_estimation_->estimateSupportPolygon(pose, base_to_com);
  }

  std::vector<double> init_flipper_angles = init_joint_state; // TODO find corresponding index
  contact_estimation_->updateJointStates(init_flipper_angles);
  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(pose, base_to_com, init_flipper_angles, updated_init_contact);
  result.result_contact = flipper_result.contact_result;
  result.result_joint_angles = flipper_result.flipper_angles;

  //  ArmOptResult result = arm_optimization_->optimize(joint_state, flipper_result.contact_result);

  return result;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose, const std::vector<double> &init_joint_state)
{
  return findOptimalPosture(pose, init_joint_state, NULL);
}

}
