#include <whole_body_posture_optimization/flipper/fast_traction_cost.h>

namespace whole_body_posture_optimization {

FastTractionCost::FastTractionCost(const Eigen::Affine3d& pose, const std::vector<std::string>& free_joints, const std::vector<double>& initial_flipper_position, const std::vector<double>& previous_parameter_result,
                                   const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Parameters& parameters)
  : pose_(pose), free_joints_(free_joints), initial_flipper_position_(initial_flipper_position), previous_parameter_result_(previous_parameter_result), contact_estimation_(contact_estimation), parameters_(parameters)
{
  parameter_idx_to_flipper_idx_ = contact_estimation_->getJointIndices(free_joints);
}

bool FastTractionCost::operator()(const double* const * parameters, double* cost) const
{
  // Fuse full state with free joints
  // Parameters 0-N: Flipper joint angles
  std::vector<double> flipper_position = initial_flipper_position_;
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    flipper_position[parameter_idx_to_flipper_idx_[i]] = parameters[0][i];
  }
  // Set flipper position
  contact_estimation_->updateJointStates(flipper_position);

  // Assemble world -> base_link transform
  Eigen::Vector3d pose_rpy = sdf_contact_estimation::rotToNormalizedRpy(pose_.linear());
  // Parameters N-N+3: Z offset, Roll, Pitch coordinates of robot base in world
  Eigen::Affine3d pose(Eigen::AngleAxisd(pose_rpy(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(parameters[0][free_joints_.size() + 2], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(parameters[0][free_joints_.size() + 1], Eigen::Vector3d::UnitX()));
  pose.translation().x() = pose_.translation().x();
  pose.translation().y() = pose_.translation().y();
  pose.translation().z() = pose_.translation().z() + parameters[0][free_joints_.size() + 0];


  // Get sampled points in base link
  const sdf_contact_estimation::RobotShape& robot_shape = contact_estimation_->getRobotShape();

  double track_contact_cost = 0;
  double ground_obstacle_cost = 0;
  for (sdf_contact_estimation::ShapePtr shape: robot_shape) {
    if (!shape->isTrack()) {
      continue;
    }
    const std::vector<Eigen::Vector3d> sampling_points = shape->getSamplingPoints();
    for (const Eigen::Vector3d& contact: sampling_points) {
      // Transform to world frame
      Eigen::Vector3d contact_world = pose * contact;
      // Evaluate SDF at points
      double distance = contact_estimation_->getSdfValue(contact_world.x(), contact_world.y(), contact_world.z());
      // Apply cost function
      if (distance > 0) {
        track_contact_cost += distance;
      } else {
        ground_obstacle_cost += distance*distance;
      }
    }
  }

  double height_cost = pose.translation().z();

  ROS_INFO_STREAM(std::endl <<
                  "Track contact cost: " << track_contact_cost << std::endl <<
                  "Ground obstacle cost: " << ground_obstacle_cost
                  );

  *cost = track_contact_cost + ground_obstacle_cost + 10*height_cost;
  return true;
}

ceres::CostFunction* FastTractionCost::Create(const Eigen::Affine3d& pose, const std::vector<std::string>& free_joints, const std::vector<double>& initial_flipper_position, const std::vector<double>& previous_parameter_result,
                                              const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation, const Parameters& parameters)
{
  ceres::DynamicNumericDiffCostFunction<FastTractionCost>* cost_function = new ceres::DynamicNumericDiffCostFunction<FastTractionCost>(
        new FastTractionCost(pose, free_joints, initial_flipper_position, previous_parameter_result, contact_estimation, parameters));

  cost_function->AddParameterBlock(static_cast<int>(free_joints.size()) + 3);
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
