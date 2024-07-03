#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>

#include "whole_body_posture_optimization/util/common.h"
#include <algorithm>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <whole_body_planning_plotting_utils/data_log.h>

using whole_body_planning_plotting_utils::DataLog;

namespace whole_body_posture_optimization {

FlipperMultiObjectiveCost::FlipperMultiObjectiveCost(const std::vector<std::string>& free_joints, const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                     const Eigen::Isometry3d &pose,
                                                     const robot_state::RobotStateConstPtr& previous_state,
                                                     const FlipperMultiObjectiveCostParameters& parameters,
                                                     const robot_state::RobotStateConstPtr& last_solution)
  : free_joints_(free_joints), pose_predictor_(pose_predictor), pose_(pose), pose_math_(pose), parameters_(parameters)
{
  param_idx_to_state_idx = getIndexMap<std::string>(pose_predictor_->robotModel()->jointNames(), free_joints_);
  if (previous_state) {
    prev_flipper_param_angles_ = getVariablePositions(previous_state, free_joints_);
    prev_pose_ = previous_state->getJointTransform("world_virtual_joint");
    // TODO: use these values from previous iteration
    // Compute previous COM in world
    for (unsigned int i = 0; i < free_joints_.size(); i++) {
      pose_predictor_->robotModel()->updateJointPosition(param_idx_to_state_idx[i], prev_flipper_param_angles_[i]);
    }
    prev_com_ = prev_pose_ * pose_predictor->robotModel()->centerOfMass();
    // Compute previous support polygon
    pose_predictor_->estimateSupportPolygon(hector_math::Pose<double>(prev_pose_), prev_support_polygon_);
    if (prev_support_polygon_.contact_hull_points.empty()) {
      ROS_WARN_STREAM("Failed to compute previous support polygon. Cannot compute impact costs.");
    }
  }
  if (last_solution) {
    last_solution_ = getVariablePositions(last_solution, free_joints_);
  }

  // Create worker threads
  for (int i = 0; i < std::floor(parameters_.num_pose_x_samples); ++i) {
    workers_.push_back(std::make_shared<WorkerThread>());
  }

  // Set up logging
  if (DataLog::getInstance().getTable("flipper_multi_objective_cost").enabled()) {
    std::vector<std::string> data_labels = free_joints;
    data_labels.emplace_back("offset_step");
    data_labels.emplace_back("support_area_cost");
    data_labels.emplace_back("track_contact_cost");
    data_labels.emplace_back("body_contact_cost");
    data_labels.emplace_back("turning_resistance_cost");
    data_labels.emplace_back("movement_cost");
    data_labels.emplace_back("roll_angle_cost");
    data_labels.emplace_back("position_deviation_cost");
    data_labels.emplace_back("stability_cost");
    data_labels.emplace_back("impact_cost");
    data_labels.emplace_back("total_cost");

    DataLog::getInstance().getTable("flipper_multi_objective_cost").initialize(data_labels);
  }

  if (DataLog::getInstance().getTable("flipper_multi_objective_total_cost").enabled()) {
    std::vector<std::string> data_labels = free_joints;
    data_labels.emplace_back("total_cost");

    DataLog::getInstance().getTable("flipper_multi_objective_total_cost").initialize(data_labels);
  }

  if (DataLog::getInstance().getTable("_" + PROBABILITY_DISTRIBUTION_TABLE_NAME).enabled()) {
    DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).initialize({
        "pose_x",
        "pose_y",
        "pose_yaw",
        "flipper_cost",
        "rel_probability"
    });
  }
}

bool FlipperMultiObjectiveCost::operator()(double const* const* flipper_angles, double* cost) const
{
  // *** Simulation ** //
  // Update joint state
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + free_joints_.size());
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    pose_predictor_->robotModel()->updateJointPosition(param_idx_to_state_idx[i], flipper_angles_vec[i]);
  }

  cost[0] = -std::numeric_limits<double>::max();
  int num_pose_x_samples = std::floor(parameters_.num_pose_x_samples);
  num_pose_x_samples = (num_pose_x_samples < 1) ? 1 : num_pose_x_samples;

  if (num_pose_x_samples == 1) {
    cost[0] = evaluateCost(pose_, flipper_angles_vec);
  } else {
    // Spawn worker threads
    std::vector<std::promise<double>> cost_promises(num_pose_x_samples);
    for (int offset_counter = 0; offset_counter < num_pose_x_samples; ++offset_counter) {
      double offset = (offset_counter - std::floor(num_pose_x_samples/2.0)) * parameters_.sample_step_pose_x;
      Eigen::Translation3d offset_trans(offset, 0, 0);
      Eigen::Isometry3d pose_sample = pose_ * offset_trans;
      workers_[offset_counter]->addTask([this, pose_sample, flipper_angles_vec, offset_counter, &cost_promises]() {
        evaluateCostAsync(pose_sample, flipper_angles_vec, cost_promises[offset_counter]);
      });
    }

    // Collect results
    std::vector<CostSample> samples;
    samples.reserve(num_pose_x_samples);
    for (int offset_counter = 0; offset_counter < num_pose_x_samples; ++offset_counter) {
      double offset = (offset_counter - std::floor(num_pose_x_samples/2.0)) * parameters_.sample_step_pose_x;
      auto cost_future = cost_promises[offset_counter].get_future();
      double local_cost = cost_future.get(); // Blocks until available
      double rel_prob = gaussianDistribution(offset, 0.0, parameters_.pose_x_sigma);
      CostSample sample{local_cost, rel_prob};
      samples.emplace_back(sample);
    }
    cost[0] = computeCVaR(samples, parameters_.risk_level);
  }

  if (DataLog::getInstance().getTable("flipper_multi_objective_total_cost").enabled()) {
    Eigen::VectorXd data(DataLog::getInstance().getTable("flipper_multi_objective_total_cost").getDataSize());
    for (unsigned int i = 0; i < free_joints_.size(); ++i) {
      data[i] = flipper_angles[0][i];
    }
    Eigen::Index index = static_cast<Eigen::Index>(free_joints_.size()) - 1;
    data[++index] = cost[0];
    DataLog::getInstance().getTable("flipper_multi_objective_total_cost").addData(data);
  }

  return true;
}

ceres::CostFunction* FlipperMultiObjectiveCost::Create(const std::vector<std::string>& free_joints,
                                                 const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                 const Eigen::Isometry3d &pose,
                                                 const robot_state::RobotStateConstPtr& previous_state,
                                                 const FlipperMultiObjectiveCostParameters& parameters,
                                                 const robot_state::RobotStateConstPtr& last_solution)
{
  auto* cost_function = new ceres::DynamicNumericDiffCostFunction<FlipperMultiObjectiveCost>(
      new FlipperMultiObjectiveCost(free_joints, pose_predictor, pose, previous_state, parameters, last_solution));

  cost_function->AddParameterBlock(static_cast<int>(free_joints.size()));
  cost_function->SetNumResiduals(1);

  return static_cast<ceres::CostFunction*>(cost_function);
}

double FlipperMultiObjectiveCost::computeCVaR(std::vector<CostSample> samples, double risk_level) const
{
  // Sort by cost
  std::sort(samples.begin( ), samples.end( ),
            [ ]( const CostSample& lhs, const CostSample& rhs ) {
              return lhs.cost < rhs.cost;
            }
  );
  // Cumulative probability density
  std::vector<double> cum_sum_rel_prob;
  cum_sum_rel_prob.reserve(samples.size());
  cum_sum_rel_prob.push_back(samples.front().relative_probability);
  for (unsigned int i = 1; i < samples.size(); ++i) {
    cum_sum_rel_prob.push_back(cum_sum_rel_prob[i - 1] + samples[i].relative_probability);
  }
  double prob_sum = cum_sum_rel_prob.back();
  std::transform(cum_sum_rel_prob.begin(), cum_sum_rel_prob.end(), cum_sum_rel_prob.begin(),
                 [&prob_sum](double v) { return v / prob_sum; });

  // Value at risk
  auto it = std::upper_bound(cum_sum_rel_prob.begin(), cum_sum_rel_prob.end(), risk_level);
  size_t start_index = it - cum_sum_rel_prob.begin();
  // Calculate weighted mean
  double sum = 0;
  double cvar = 0;
  for (size_t i = start_index; i < samples.size(); ++i) {
    cvar += samples[i].cost * samples[i].relative_probability;
    sum += samples[i].relative_probability;
  }
  cvar /= sum;
  return cvar;
}

double FlipperMultiObjectiveCost::evaluateCost(const Eigen::Isometry3d& pose_sample,
                                               const std::vector<double>& flipper_angles) const {
  // Simulate contact
  hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> contact_information;

  hector_math::Pose<double> result_pose(pose_sample);
  double stability = pose_predictor_->predictPoseAndContactInformation(result_pose, support_polygon, contact_information, hector_pose_prediction_interface::ContactInformationFlags::All);
  Eigen::Isometry3d result_pose_eigen = result_pose.asTransform();

  // Catch: Estimation failed or robot fell over
  if (std::isnan(stability) || stability == -std::numeric_limits<double>::max()) {
    return std::numeric_limits<double>::max();
  }

  // ** Transition costs ++ //
  double turning_resistance = 0;
  double movement_cost = 0.0;
  double impact_cost = 0.0;
  if (!prev_flipper_param_angles_.empty()) {
    // Enforce valid transitions from previous to this state
    // The transition is only considered valid, if the previous (world) COM pose is still inside the new support polygon
    // This should ensure that no intermediate state between the discretized states is unstable.
    // Note: This completely prevents tipping between states, even if it would be possible
    std::vector<double> prev_com_stabilities =
        computeForceAngleStabilitiesWithGravity<double>(support_polygon.contact_hull_points, prev_com_);
    double prev_com_stability = *std::min_element(prev_com_stabilities.begin(), prev_com_stabilities.end());
    if (prev_com_stability < 0) {
      return std::numeric_limits<double>::max();
    }

    // Compute rotation difference
    Eigen::Quaterniond start_orientation(prev_pose_.linear());
    Eigen::Quaterniond end_orientation(result_pose_eigen.linear());
    Eigen::Quaterniond rotation_diff = start_orientation.inverse() * end_orientation;
    Eigen::Vector3d euler_angles = quatToEuler(rotation_diff);

    // Reduce impact force when tipping
    if (parameters_.impact_weight != 0 && !prev_support_polygon_.contact_hull_points.empty()) {
      Eigen::Vector3d com = result_pose_eigen * pose_predictor_->robotModel()->centerOfMass();
      std::vector<double> curr_com_prev_polygon_stabilities =
          computeForceAngleStabilitiesWithGravity<double>(prev_support_polygon_.contact_hull_points, com);
      double curr_com_prev_polygon_stability = *std::min_element(curr_com_prev_polygon_stabilities.begin(), curr_com_prev_polygon_stabilities.end());

      if (curr_com_prev_polygon_stability < 0) {
        // Tipping event, penalize tipping angle
        Eigen::Vector3d rel_pos = prev_pose_.inverse() * result_pose_eigen.translation();
        double backwards = rel_pos.x() > 0 ? 1.0 : -1.0;
        double rotation_angle = backwards * euler_angles(1); // only penalize falling towards ground
        impact_cost = std::max(0.0, rotation_angle);
      }
    }

    // Approximate work required to overcome the moment of turning resistance
    // Assumes to robot to turn on the spot because the real ICC is unknown
    if (parameters_.turning_resistance_weight != 0) {
      // Compute contact
      double total_contact_area = 0;
      for (const hector_pose_prediction_interface::ContactPointInformation<double>& point: contact_information.contact_points) {
        Eigen::Vector3d p_base = result_pose_eigen.inverse() * point.point; // Transform from world to base link
        double distance = p_base.block<2, 1>(0, 0).norm();
        turning_resistance += p_base.x() * point.surface_area * p_base.x() / distance;
        total_contact_area += point.surface_area;
      }
      if (total_contact_area > 0) {
        turning_resistance /= total_contact_area;
      }

      double turning_angle = euler_angles(0);
      turning_resistance *= std::abs(turning_angle); // moment of resistance * displacement = work required to overcome moment of resistance
    }

    if (parameters_.movement_cost_weight != 0) {
      // Joint movement cost
      for (unsigned int idx = 0; idx < prev_flipper_param_angles_.size(); idx++) {
        double angle_difference = prev_flipper_param_angles_[idx] - flipper_angles[idx];
        //          movement_cost += lorentzian<double>(angle_difference, parameters_.lorentzian_sigma);
        movement_cost = std::max(lorentzian<double>(angle_difference, parameters_.lorentzian_sigma), movement_cost);
      }
      //        movement_cost /= static_cast<double>(prev_flipper_param_angles_.size()); // Normalize by number of joints
    }
  }

  double regularization_cost = 0;
  if (!last_solution_.empty() && parameters_.regularization_weight > 0.0) {
    for (unsigned int idx = 0; idx < last_solution_.size(); idx++) {
      double angle_difference = last_solution_[idx] - flipper_angles[idx];
      regularization_cost = std::max(lorentzian<double>(angle_difference, parameters_.lorentzian_sigma), regularization_cost);
    }
  }

  // *** State costs ** //
  // Penalize deviation from requested position
  Eigen::Isometry3d original_to_estimated = pose_.inverse() * result_pose_eigen;
  double position_difference = original_to_estimated.translation().block<2,1>(0,0).norm();
  double position_deviation_cost = std::max(0.0, position_difference - 0.02); // 2cm leeway

  // Penalize chassis roll angle
  // We want the chassis to be upright
  double chassis_roll_angle = result_pose_eigen.linear().eulerAngles(0, 1, 2)[0];
  if (chassis_roll_angle > M_PI_2) {
    chassis_roll_angle = M_PI - chassis_roll_angle;
  }

  // Stability cost
  double stability_cost = 0;
  if (parameters_.stability_weight != 0) {
    //    Eigen::Vector3d com_pos = result_pose_eigen * pose_predictor_->robotModel()->centerOfMass();
    //    std::vector<double> stabilities = computeForceAngleStabilitiesWithGravity<double>(support_polygon.contact_hull_points, com_pos);
    std::vector<double> transformed_edge_stabilities =  support_polygon.edge_stabilities;
    if (parameters_.stability_nominal_value != 0) {
      std::transform(transformed_edge_stabilities.begin(), transformed_edge_stabilities.end(), transformed_edge_stabilities.begin(), [this](double stability) -> double {
        return std::min(stability, parameters_.stability_nominal_value);
      });
    }
    std::transform(transformed_edge_stabilities.begin(), transformed_edge_stabilities.end(), transformed_edge_stabilities.begin(), [this](double stability) -> double {
      return parameters_.stability_exp_parameters[0] * exp(parameters_.stability_exp_parameters[1] * stability+parameters_.stability_exp_parameters[2]);
    });

    stability_cost = std::accumulate(transformed_edge_stabilities.begin(), transformed_edge_stabilities.end(), 0.0);
    stability_cost /= static_cast<double>(support_polygon.edge_stabilities.size()); // average cost per axis
  }

  // Traction and fragile contacts costs
  // Use track contact to approximate traction and penalize chassis contacts
  double track_contact_area = 0;
  double body_contact_area = 0;
  Eigen::Vector3d gravity_vector = Eigen::Vector3d::UnitZ();
  for (const hector_pose_prediction_interface::ContactPointInformation<double>& point: contact_information.contact_points) {

    if (point.link_type == hector_pose_prediction_interface::LinkType::Tracks) {
      // Traction cost, Coulomb friction
      if (point.surface_normal == Eigen::Vector3d::Zero()) continue; // No gradient
      Eigen::Vector3d surface_normal = point.surface_normal.normalized();
      double normal_force = gravity_vector.dot(surface_normal); // Value between 0 (low traction) and 1 (high traction)
      track_contact_area += normal_force * point.surface_area;
    }

    if (point.link_type == hector_pose_prediction_interface::LinkType::Chassis ||
        point.link_type == hector_pose_prediction_interface::LinkType::Fragile) {
      body_contact_area += point.surface_area;
    }
  }
  if (parameters_.track_contact_nominal_value != 0) {
    track_contact_area = std::min(track_contact_area, parameters_.track_contact_nominal_value);
  }
  track_contact_area *= -1; // Negative cost (reward)

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
  support_area *= -1; // Negative cost (reward)

  // Sum up cost terms
  double local_cost = + parameters_.support_area_weight * support_area
                      + parameters_.track_contact_weight * track_contact_area
                      + parameters_.body_contact_weight * body_contact_area
                      + parameters_.turning_resistance_weight * turning_resistance
                      + parameters_.movement_cost_weight * movement_cost
                      + parameters_.roll_cost_weight * chassis_roll_angle * chassis_roll_angle
                      + parameters_.position_deviation_weight * position_deviation_cost
                      + parameters_.stability_weight * stability_cost
                      + parameters_.impact_weight * impact_cost
                      + parameters_.regularization_weight * regularization_cost;
  return local_cost;

//    if (parameters_.verbose) {
//      ROS_INFO_STREAM("Flipper cost function (weighted value in brackets): " << std::endl <<
//                      "Support area: " << support_area << " (" << parameters_.support_area_weight * support_area << ")" << std::endl <<
//                      "Track contact: " << track_contact_area << " (" << parameters_.track_contact_weight * track_contact_area << ")" << std::endl <<
//                      "Body contact: " << body_contact_area << " (" << parameters_.body_contact_weight * body_contact_area << ")" << std::endl <<
//                      "Impact: " << impact_cost << " (" << parameters_.impact_weight * impact_cost << ")" << std::endl <<
//                      "Turning resistance: " << turning_resistance << " (" << parameters_.turning_resistance_weight * turning_resistance << ")" << std::endl <<
//                      "Movement penalty: " << movement_cost << " (" << parameters_.movement_cost_weight * movement_cost << ")" << std::endl <<
//                      "Position deviation cost: " << position_deviation_cost << " (" << parameters_.position_deviation_weight * position_deviation_cost << ")" << std::endl <<
//                      "Chassis roll angle: " << chassis_roll_angle << " (" << parameters_.roll_cost_weight * chassis_roll_angle << ")" << std::endl <<
//                      "Stability cost: " << stability_cost << " (" << parameters_.stability_weight * stability_cost << ")" << std::endl <<
//                      "Total cost: " << local_cost);
//    }
//
//    if (DataLog::getInstance().getTable("flipper_multi_objective_cost").enabled()) {
//      Eigen::VectorXd data(DataLog::getInstance().getTable("flipper_multi_objective_cost").getDataSize());
//      for (unsigned int i = 0; i < free_joints_.size(); ++i) {
//        data[i] = flipper_angles[0][i];
//      }
//      Eigen::Index index = static_cast<Eigen::Index>(free_joints_.size()) - 1;
//      data[++index] = offset_counter;
//      data[++index] = parameters_.support_area_weight * support_area;
//      data[++index] = parameters_.track_contact_weight * track_contact_area;
//      data[++index] = parameters_.body_contact_weight * body_contact_area;
//      data[++index] = parameters_.turning_resistance_weight * turning_resistance;
//      data[++index] = parameters_.movement_cost_weight * movement_cost;
//      data[++index] = parameters_.roll_cost_weight * chassis_roll_angle * chassis_roll_angle;
//      data[++index] = parameters_.position_deviation_weight * position_deviation_cost;
//      data[++index] = parameters_.stability_weight * stability_cost;
//      data[++index] = parameters_.impact_weight * impact_cost;
//      data[++index] = local_cost;
//      DataLog::getInstance().getTable("flipper_multi_objective_cost").addData(data);
//    }
//
//    if (DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).enabled()) {
//
//      Eigen::VectorXd data(DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).getDataSize());
//      data <<
//          pose_sample.translation().x(),
//          pose_sample.translation().y(),
//          0.0,
//          local_cost,
//          rel_prob;
//      DataLog::getInstance().getTable(PROBABILITY_DISTRIBUTION_TABLE_NAME).addData(data);
//    }
}

void FlipperMultiObjectiveCost::evaluateCostAsync(const Eigen::Isometry3d& pose_sample,
                                                  const std::vector<double>& flipper_angles,
                                                  std::promise<double>& cost_promise) const {
  double cost = evaluateCost(pose_sample, flipper_angles);
  cost_promise.set_value(cost);
}

}
