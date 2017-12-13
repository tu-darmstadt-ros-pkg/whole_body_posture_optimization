#include <whole_body_posture_optimization/whole_body_posture_optimization.h>

namespace whole_body_posture_optimization {

WholeBodyPostureOptimization::WholeBodyPostureOptimization(const ros::NodeHandle& nh, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen)
  : nh_(nh)
{
  loadParameters(nh);
  robot_model_ = std::make_shared<RobotModel>(nh_);
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(robot_shape_gen);
  contact_estimation_->loadParametersFromNamespace(nh_);
  ros::NodeHandle flipper_nh(nh, "flipper_optimization");
  flipper_optimization_ = std::make_shared<FlipperOptimization>(flipper_nh, robot_model_, robot_shape_gen, contact_estimation_);
  ros::NodeHandle arm_nh(nh, "arm_optimization");
  arm_optimization_ = std::make_shared<ArmOptimization>(arm_nh, robot_model_, contact_estimation_);
}

bool WholeBodyPostureOptimization::loadParameters(const ros::NodeHandle &nh)
{
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

std::vector<PostureOptimizationResult> WholeBodyPostureOptimization::findOptimalPostures(const nav_msgs::Path& path)
{
  std::vector<PostureOptimizationResult> results;
  PostureOptimizationResult last_result;
  ros::Rate rate(5);
  bool first = true;
  for (const geometry_msgs::PoseStamped& pose_msg: path.poses) {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg.pose, pose);

    auto start = std::chrono::high_resolution_clock::now();
    if (first) {
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), getDefaultJointPositions());
      first = false;
    } else {
//      last_result = findOptimalPosture(pose, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
      last_result = findOptimalPosture(pose, getDefaultJointPositions(), last_result.result_joint_angles, &last_result.result_contact);
    }
    results.push_back(last_result);
    std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
    ROS_INFO_STREAM("Complete optimization took " << elapsed.count()*1000.0 << " ms.");
    rate.sleep();
  }

  return results;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d& pose, const std::vector<double>& init_joint_angles,
                                                                           const std::vector<double> &previous_joint_angles,
                                                                           const sdf_contact_estimation::ContactEstimationResult* init_contact_result)
{
  PostureOptimizationResult result;
  result.init_joint_angles = init_joint_angles;

  // TODO we should be able to just take this from the previous time step. com shouldn't have changed
  robot_model_->computeCenterOfMass(init_joint_angles, result.init_com);
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = result.init_com;

  sdf_contact_estimation::ContactEstimationResult updated_init_contact;
  if (init_contact_result != NULL) {
    result.init_contact = *init_contact_result;
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com, result.init_contact);
  } else {
    updated_init_contact = flipper_optimization_->computeContactEstimation(init_joint_angles, pose, base_to_com);
  }
//  result.result_contact = updated_init_contact;

  FlipperOptimizationResult flipper_result = flipper_optimization_->optimize(updated_init_contact.estimated_pose, base_to_com, init_joint_angles, previous_joint_angles,
                                                                             updated_init_contact);
  result.result_contact = flipper_result.contact_result;
  result.result_joint_angles = flipper_result.joint_state;

  if (updated_init_contact.convex_hull_points.empty()) {
    ROS_ERROR_STREAM("Contact estimation failed.");
    result.result_joint_angles = init_joint_angles;
    return result;
  }

  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
                                                                 flipper_result.contact_result.estimated_pose, previous_joint_angles);
//  ArmOptimizationResult arm_result = arm_optimization_->optimize(flipper_result.joint_state, flipper_result.contact_result.convex_hull_points,
//                                                                 flipper_result.contact_result.estimated_pose, getDefaultJointPositions());
//  ArmOptimizationResult arm_result = arm_optimization_->optimize(init_joint_state, updated_init_contact.convex_hull_points, updated_init_contact.estimated_pose);
  result.result_joint_angles = arm_result.joint_angles;
  result.result_com = arm_result.center_of_mass;

  robot_model_->updateRobotState(result.result_contact.estimated_pose, result.result_joint_angles);
  robot_model_->publishRobotState();
  robot_model_->publishCollisionSpheres();

  return result;
}

PostureOptimizationResult WholeBodyPostureOptimization::findOptimalPosture(const Eigen::Affine3d &pose, const std::vector<double> &init_joint_state, const std::vector<double> &previous_joint_angles)
{
  return findOptimalPosture(pose, init_joint_state, previous_joint_angles, NULL);
}

}
