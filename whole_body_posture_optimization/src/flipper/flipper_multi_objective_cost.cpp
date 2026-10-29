#include <whole_body_posture_optimization/flipper/flipper_multi_objective_cost.h>

#include "whole_body_posture_optimization/util/common.h"
#include <algorithm>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <whole_body_planning_plotting_utils/data_log.h>

using whole_body_planning_plotting_utils::DataLog;

namespace whole_body_posture_optimization {

FlipperMultiObjectiveCost::FlipperMultiObjectiveCost(const std::vector<std::string>& free_joints, const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                     const Eigen::Isometry3d &pose,
                                                     const moveit::core::RobotStateConstPtr& previous_state,
                                                     const FlipperMultiObjectiveCostParameters& parameters,
                                                     const moveit::core::RobotStateConstPtr& last_solution,
                                                     const std::vector<std::string>& coupled_joints,
                                                     FlipperRejections* rejections,
                                                     const std::vector<std::pair<size_t, size_t>>& mirrored_pairs,
                                                     const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics)
  : free_joints_(free_joints), pose_predictor_(pose_predictor), pose_(pose), pose_math_(pose), rejections_(rejections),
    prediction_diagnostics_(prediction_diagnostics),
    mirrored_pairs_(mirrored_pairs), parameters_(parameters)
{
  param_idx_to_state_idx = getIndexMap<std::string>(pose_predictor_->robotModel()->jointNames(), free_joints_);
  const std::vector<std::string>& state_joints = pose_predictor_->robotModel()->jointNames();
  for (size_t i = 0; i < coupled_joints.size(); ++i) {
    if (coupled_joints[i].empty()) {
      continue;
    }
    auto it = std::find(state_joints.begin(), state_joints.end(), coupled_joints[i]);
    if (it == state_joints.end()) {
      throw std::invalid_argument("Coupled joint '" + coupled_joints[i] + "' is not a joint of the pose predictor.");
    }
    coupled_param_to_state_idx_.emplace_back(i, static_cast<size_t>(it - state_joints.begin()));
  }
  if (previous_state) {
    prev_flipper_param_angles_ = getVariablePositions(previous_state, free_joints_);
    prev_pose_ = previous_state->getJointTransform("world_virtual_joint");
    // Compute previous COM in world
    setJointPositions(prev_flipper_param_angles_);
    prev_com_ = prev_pose_ * pose_predictor->robotModel()->centerOfMass();
    // Compute previous support polygon
    pose_predictor_->estimateSupportPolygon(hector_math::Pose<double>(prev_pose_), prev_support_polygon_);
    if (prev_support_polygon_.contact_hull_points.empty()) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("flipper_multi_objective_cost"), "Failed to compute previous support polygon. Cannot compute impact costs.");
    }
  }
  if (last_solution) {
    last_solution_ = getVariablePositions(last_solution, free_joints_);
  }

  // Create worker threads
  worker_pool_ = std::make_shared<WorkerThreadPool>(parameters_.num_pose_x_samples);

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

double predictSettledPose(hector_pose_prediction_interface::PosePredictor<double>& pose_predictor,
                          const FlipperMultiObjectiveCostParameters& parameters, const Eigen::Isometry3d& pose,
                          hector_math::Pose<double>& result_pose,
                          hector_pose_prediction_interface::SupportPolygon<double>& support_polygon,
                          hector_pose_prediction_interface::ContactInformation<double>& contact_information)
{
  const auto flags = hector_pose_prediction_interface::ContactInformationFlags::All;
  const auto drift = [&pose](const hector_math::Pose<double>& settled) {
    return (settled.asTransform().translation() - pose.translation()).head<2>().norm();
  };
  result_pose = hector_math::Pose<double>(pose);
  double stability = pose_predictor.predictPoseAndContactInformation(result_pose, support_polygon, contact_information, flags);
  if (parameters.max_position_deviation > 0 && drift(result_pose) > parameters.max_position_deviation) {
    return std::nan("");
  }
  return stability;
}

bool FlipperMultiObjectiveCost::operator()(double const* const* flipper_angles, double* cost) const
{
  // *** Simulation ** //
  // Update joint state
  std::vector<double> flipper_angles_vec(flipper_angles[0], flipper_angles[0] + free_joints_.size());
  setJointPositions(flipper_angles_vec);

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
      worker_pool_->addTask([this, pose_sample, flipper_angles_vec, offset_counter, &cost_promises]() {
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
    if (std::isinf(cost[0])) {
      cost[0] = std::numeric_limits<double>::max();
    }
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

void FlipperMultiObjectiveCost::setJointPositions(const std::vector<double>& flipper_angles) const
{
  for (unsigned int i = 0; i < free_joints_.size(); i++) {
    pose_predictor_->robotModel()->updateJointPosition(param_idx_to_state_idx[i], flipper_angles[i]);
  }
  for (const auto& [param_idx, state_idx] : coupled_param_to_state_idx_) {
    pose_predictor_->robotModel()->updateJointPosition(state_idx, flipper_angles[param_idx]);
  }
}

ceres::CostFunction* FlipperMultiObjectiveCost::Create(const std::vector<std::string>& free_joints,
                                                 const hector_pose_prediction_interface::PosePredictor<double>::Ptr& pose_predictor,
                                                 const Eigen::Isometry3d &pose,
                                                 const moveit::core::RobotStateConstPtr& previous_state,
                                                 const FlipperMultiObjectiveCostParameters& parameters,
                                                 const moveit::core::RobotStateConstPtr& last_solution,
                                                 const std::vector<std::string>& coupled_joints,
                                                 FlipperRejections* rejections,
                                                 const std::vector<std::pair<size_t, size_t>>& mirrored_pairs,
                                                 const sdf_contact_estimation::PredictionDiagnostics* prediction_diagnostics)
{
  auto* cost_function = new ceres::DynamicNumericDiffCostFunction<FlipperMultiObjectiveCost>(
      new FlipperMultiObjectiveCost(free_joints, pose_predictor, pose, previous_state, parameters, last_solution, coupled_joints, rejections, mirrored_pairs, prediction_diagnostics));

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

  hector_math::Pose<double> result_pose;
  double stability = predictSettledPose(*pose_predictor_, parameters_, pose_sample, result_pose, support_polygon, contact_information);
  Eigen::Isometry3d result_pose_eigen = result_pose.asTransform();

  // Catch: Estimation failed or robot fell over
  const bool failed = std::isnan(stability) || stability == -std::numeric_limits<double>::max();
  // Only with one pose sample, the samples of the CVaR path share the predictor's diagnostics
  const sdf_contact_estimation::PredictionDiagnostics* diagnostics =
      parameters_.num_pose_x_samples <= 1 ? prediction_diagnostics_ : nullptr;
  if (rejections_ && diagnostics && diagnostics->unknown_sampling_points > 0) {
    ++rejections_->touched_unknown;
    if (failed) {
      ++rejections_->failed_touched_unknown;
    }
  }
  if (failed) {
    if (rejections_) {
      const double drift = (result_pose_eigen.translation() - pose_sample.translation()).head<2>().norm();
      const bool drifted = std::isnan(stability) && parameters_.max_position_deviation > 0 && drift > parameters_.max_position_deviation;
      ++(drifted ? rejections_->drifted : rejections_->unsettled);
      if (!drifted && diagnostics) {
        using sdf_contact_estimation::PredictionStatus;
        switch (diagnostics->status) {
          case PredictionStatus::FellOver: ++rejections_->fell_over; break;
          case PredictionStatus::NoHullIteration:
          case PredictionStatus::NoHullFinal: ++rejections_->no_contact; break;
          case PredictionStatus::NotConverged: ++rejections_->not_converged; break;
          default: break;
        }
      }
    }
    return std::numeric_limits<double>::max();
  }

  // ** Transition costs ++ //
  double turning_resistance = 0;
  double transition_cost = 0.0;
  double movement_cost = 0.0;
  double impact_cost = 0.0;
  if (!prev_flipper_param_angles_.empty()) {
    // A new support polygon that holds the previous centre of mass keeps the robot from tipping between the waypoints
    std::vector<double> prev_com_stabilities =
        computeForceAngleStabilitiesWithGravity<double>(support_polygon.contact_hull_points, prev_com_);
    double prev_com_stability = *std::min_element(prev_com_stabilities.begin(), prev_com_stabilities.end());
    if (prev_com_stability < 0) {
      if (parameters_.transition_weight <= 0.0) {
        if (rejections_) {
          ++rejections_->transition;
        }
        return std::numeric_limits<double>::max();
      }
      // The robot tips between the waypoints, which costs by how far
      transition_cost = -prev_com_stability;
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
      std::vector<double> joint_costs;
      joint_costs.reserve(prev_flipper_param_angles_.size());
      for (unsigned int idx = 0; idx < prev_flipper_param_angles_.size(); idx++) {
        double angle_difference = prev_flipper_param_angles_[idx] - flipper_angles[idx];
        joint_costs.push_back(lorentzian<double>(angle_difference, parameters_.lorentzian_sigma));
      }
      movement_cost = softMax(joint_costs, parameters_.movement_temperature);
    }
  }

  double regularization_cost = 0;
  if (!last_solution_.empty() && parameters_.regularization_weight > 0.0) {
    std::vector<double> joint_costs;
    joint_costs.reserve(last_solution_.size());
    for (unsigned int idx = 0; idx < last_solution_.size(); idx++) {
      double angle_difference = last_solution_[idx] - flipper_angles[idx];
      joint_costs.push_back(lorentzian<double>(angle_difference, parameters_.lorentzian_sigma));
    }
    regularization_cost = softMax(joint_costs, parameters_.movement_temperature);
  }

  double symmetry_cost = 0;
  if (parameters_.symmetry_weight > 0.0) {
    for (const auto& [left, right] : mirrored_pairs_) {
      symmetry_cost += lorentzian<double>(flipper_angles[left] - flipper_angles[right], parameters_.lorentzian_sigma);
    }
  }

  // *** State costs ** //
  // Penalize deviation from requested position
  Eigen::Isometry3d original_to_estimated = pose_.inverse() * result_pose_eigen;
  double position_difference = original_to_estimated.translation().block<2,1>(0,0).norm();
  double position_deviation_cost = std::max(0.0, position_difference - 0.02); // 2cm leeway

  // Penalize chassis roll angle
  // We want the chassis to be upright
  double chassis_roll_angle = sdf_contact_estimation::rotToNormalizedRpy(result_pose_eigen.linear())[0];

  // Stability cost
  double stability_cost = 0;
  if (parameters_.stability_weight != 0) {
    std::vector<double> transformed_edge_stabilities =  support_polygon.edge_stabilities;
    if (parameters_.stability_nominal_value != 0) {
      std::transform(transformed_edge_stabilities.begin(), transformed_edge_stabilities.end(), transformed_edge_stabilities.begin(), [this](double stability) -> double {
        return std::min(stability, parameters_.stability_nominal_value);
      });
    }
    std::transform(transformed_edge_stabilities.begin(), transformed_edge_stabilities.end(), transformed_edge_stabilities.begin(), [this](double stability) -> double {
      return parameters_.stability_exp_parameters[0] * exp(parameters_.stability_exp_parameters[1] * stability+parameters_.stability_exp_parameters[2]);
    });

    stability_cost = softMax(transformed_edge_stabilities, parameters_.stability_temperature);
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
      // Above the track (a flipper under an open stair plate) or beside it (a block next to a flipper), clearly so for thin plates
      const double lateral = std::abs(result_pose_eigen.linear().col(1).dot(surface_normal));
      if (normal_force < -0.5 || lateral > 0.7) {
        body_contact_area += point.surface_area;
        continue;
      }
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
                      + parameters_.regularization_weight * regularization_cost
                      + parameters_.symmetry_weight * symmetry_cost
                      + parameters_.transition_weight * transition_cost;
  return local_cost;
}

void FlipperMultiObjectiveCost::evaluateCostAsync(const Eigen::Isometry3d& pose_sample,
                                                  const std::vector<double>& flipper_angles,
                                                  std::promise<double>& cost_promise) const {
  double cost = evaluateCost(pose_sample, flipper_angles);
  cost_promise.set_value(cost);
}

}
