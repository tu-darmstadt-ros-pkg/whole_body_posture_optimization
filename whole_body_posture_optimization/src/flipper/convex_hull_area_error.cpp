#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>

#include <whole_body_posture_optimization/common.h>
#include <algorithm>
#include <ceres/dynamic_numeric_diff_cost_function.h>

namespace whole_body_posture_optimization {

ConvexHullAreaError::ConvexHullAreaError(const std::vector<std::string>& free_joints, const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                         const Eigen::Isometry3d &pose,
                                         const robot_state::RobotStateConstPtr& previous_state,
                                         const ConvexHullAreaErrorParameters &parameters)
  : free_joints_(free_joints), pose_predictor_(pose_predictor), pose_(pose), pose_math_(pose), parameters_(parameters)
{
  if (previous_state) {
    prev_flipper_param_angles_ = getVariablePositions(previous_state, free_joints_);
    prev_pose_ = previous_state->getJointTransform("world_virtual_joint");
  }
  param_idx_to_state_idx = getIndexMap<std::string>(pose_predictor_->robotModel()->jointNames(), free_joints_);
}

bool ConvexHullAreaError::operator()(double const* const* flipper_angles, double* cost) const
{
  // *** Simulation ** //
  // Update joint state
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    pose_predictor_->robotModel()->updateJointPosition(param_idx_to_state_idx[i], flipper_angles[0][i]);
  }
  // Simulate contact
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> contact_information;

  hector_math::Pose<double> result_pose = pose_math_;
  double stability = pose_predictor_->predictPoseAndContactInformation(result_pose, support_polygon, contact_information, hector_pose_prediction_interface::ContactInformationFlags::All);
  Eigen::Isometry3d result_pose_eigen = result_pose.asTransform();

  // Catch: Estimation failed or robot fell over
  if (std::isnan(stability) || stability == -std::numeric_limits<double>::max()) {
    cost[0] = std::numeric_limits<double>::max();
    return true;
  }

  // *** Cost functions ** //
  // Penalize deviation from requested position
  Eigen::Isometry3d original_to_estimated = pose_.inverse() * result_pose_eigen;
  double position_difference = original_to_estimated.translation().block<2,1>(0,0).norm();
  double position_deviation_cost = std::max(0.0, position_difference - 0.02); // 2cm leeway

  // Joint movement cost
  double movement_cost = 0.0;
  if (!prev_flipper_param_angles_.empty()) {
    for (unsigned int idx = 0; idx < prev_flipper_param_angles_.size(); idx++) {
      double angle_difference = prev_flipper_param_angles_[idx] - flipper_angles[0][idx];
      movement_cost += lorentzian<double>(angle_difference, parameters_.lorentzian_sigma);
    }
    movement_cost /= static_cast<double>(prev_flipper_param_angles_.size()); // Normalize by number of joints
  }

  // Penalize chassis roll angle
  // We want the chassis to be upright
  double chassis_roll_angle = result_pose_eigen.linear().eulerAngles(0, 1, 2)[0];
  if (chassis_roll_angle > M_PI_2) {
    chassis_roll_angle = M_PI - chassis_roll_angle;
  }


  // Use contact information
  // Use track contact to approximate traction and penalize chassis contacts
  double track_contact_area = 0;
  double body_contact_area = 0;
  Eigen::Vector3d gravity_vector = Eigen::Vector3d::UnitZ();
  for (const hector_pose_prediction_interface::ContactPointInformation<double>& point: contact_information.contact_points) {

    if (point.link_type == hector_pose_prediction_interface::LinkType::Tracks) {
      // Traction cost
      // Weight track contact by angle between contact normal and (inverse) gravity vector
      // Approximates normal force
      if (point.surface_normal == Eigen::Vector3d::Zero()) continue; // No gradient
      double angle = std::acos(gravity_vector.dot(point.surface_normal)/(point.surface_normal.norm()));
      double a = -2/M_PI;
      double b = 1;
      double contact_weight = std::max(a * angle + b, 0.0); // Value between 0 (low traction) and 1 (high traction)
      track_contact_area += contact_weight * point.surface_area;
    }

    if (point.link_type == hector_pose_prediction_interface::LinkType::Chassis ||
        point.link_type == hector_pose_prediction_interface::LinkType::Fragile) {
      body_contact_area += point.surface_area;
    }
  }

  // Approximate moment of turning resistance
  double turning_resistance = 0;
  if (!prev_flipper_param_angles_.empty()) { // Check if there is a valid previous state
    // Compute contact
    for (const hector_pose_prediction_interface::ContactPointInformation<double>& point: contact_information.contact_points) {
      Eigen::Vector3d p_base = result_pose_eigen.inverse() * point.point; // Transform from world to base link
      double distance = p_base.block<2, 1>(0, 0).norm();
      turning_resistance += distance;
    }
    // Compute turning angle
    Eigen::Quaterniond start_orientation(prev_pose_.linear());
    Eigen::Quaterniond end_orientation(result_pose_eigen.linear());
    Eigen::Quaterniond rotation_diff = start_orientation.inverse() * end_orientation;
    Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);
    double turning_angle = euler_angles(0);

    turning_resistance *= std::abs(turning_angle);
  }


  // Penalize contact normals that point against the moving direction
  // For now, the moving direction is hard-coded to x+
//  Eigen::Vector3d moving_direction = Eigen::Vector3d::UnitX();
//  Eigen::Vector3d neg_moving_direction = -moving_direction;
//  double contact_against_moving_penalty_weight = 0.0;
//  double contact_against_moving_penalty_sum = 0;
//  for (Eigen::Vector3d& contact_point: result.contact_points) {
//    Eigen::Vector3d normal;
//    contact_estimation_->getSdf().getDistanceAndGradient(contact_point, normal);
//    double angle = std::acos(neg_moving_direction.dot(normal)/(neg_moving_direction.norm() * normal.norm()));
//    double a = 1.85;
//    double c = -0.85;
//    double penalty = std::max(a * std::exp(-angle) + c, 0.0);
//    contact_against_moving_penalty_sum += penalty;
//  }

  // Compute support area TODO move to hector math
  double support_area = 0;
  for (unsigned int i = 0; i < support_polygon.contact_hull_points.size(); ++i) {
    unsigned int j = (i+1) % support_polygon.contact_hull_points.size();
    support_area += support_polygon.contact_hull_points[i].y() * support_polygon.contact_hull_points[j].x()
        - support_polygon.contact_hull_points[i].x() * support_polygon.contact_hull_points[j].y();
  }

  // Sum up cost terms
  cost[0] = - parameters_.support_area_weight * support_area
            - parameters_.track_contact_weight * track_contact_area
            + parameters_.body_contact_weight * body_contact_area
            + parameters_.turning_resistance_weight * turning_resistance
            + parameters_.movement_cost_weight * movement_cost
            + parameters_.roll_cost_weight * chassis_roll_angle * chassis_roll_angle
            + parameters_.position_deviation_weight * position_deviation_cost;

  if (parameters_.verbose) {
    ROS_INFO_STREAM("Flipper cost function (weighted value in brackets): " << std::endl <<
    "Support area: " << support_area << " (" << parameters_.support_area_weight * support_area << ")" << std::endl <<
    "Track contact: " << track_contact_area << " (" << parameters_.track_contact_weight * track_contact_area << ")" << std::endl <<
    "Body contact: " << body_contact_area << " (" << parameters_.body_contact_weight * body_contact_area << ")" << std::endl <<
    "Turning resistance: " << turning_resistance << " (" << parameters_.turning_resistance_weight * turning_resistance << ")" << std::endl <<
    "Movement penalty: " << movement_cost << " (" << parameters_.movement_cost_weight * movement_cost << ")" << std::endl <<
    "Position deviation cost: " << position_deviation_cost << " (" << parameters_.position_deviation_weight * position_deviation_cost << ")" << std::endl <<
    "Chassis roll angle: " << chassis_roll_angle << " (" << parameters_.roll_cost_weight * chassis_roll_angle << ")" << std::endl <<
    "Total cost: " << cost[0]);
  }

  return true;
}

ceres::CostFunction* ConvexHullAreaError::Create(const std::vector<std::string>& free_joints,
                                                 const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                 const Eigen::Isometry3d &pose,
                                                 const robot_state::RobotStateConstPtr& previous_state,
                                                 const ConvexHullAreaErrorParameters &parameters)
{
  auto* cost_function = new ceres::DynamicNumericDiffCostFunction<ConvexHullAreaError>(
      new ConvexHullAreaError(free_joints, pose_predictor, pose, previous_state, parameters));

  cost_function->AddParameterBlock(static_cast<int>(free_joints.size()));
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

}
