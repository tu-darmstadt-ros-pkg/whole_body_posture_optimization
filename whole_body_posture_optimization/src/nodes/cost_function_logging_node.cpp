#include <rclcpp/rclcpp.hpp>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include "whole_body_posture_optimization/util/parameter_utils.h"
#include <whole_body_planning_plotting_utils/data_log.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>

using whole_body_planning_plotting_utils::DataLog;
using namespace whole_body_posture_optimization;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cost_function_logging_node");

  // Load environment
  auto sdf_model_node = node->create_sub_node("sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_node);
  sdf_model->loadFromServer(sdf_model_node);

  // Create pose predictor
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(node);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(node, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  WholeBodyPostureOptimization po(node, pose_predictor, sdf_model);

  // Load robot pose
  auto requested_pose_vec = param_utils::param(*node, "robot_pose", std::vector<double>(6, 0));
  if (requested_pose_vec.size() != 6) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Requested pose vector has wrong size (" << requested_pose_vec.size() << ") instead of 6. Using identity.");
    requested_pose_vec.resize(6, 0);
  }
  Eigen::Isometry3d robot_pose;
  robot_pose = Eigen::AngleAxisd(requested_pose_vec[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(requested_pose_vec[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(requested_pose_vec[3], Eigen::Vector3d::UnitX());
  robot_pose.translation() = Eigen::Vector3d(requested_pose_vec[0], requested_pose_vec[1], requested_pose_vec[2]);

  // Optimize
  DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging();
  DataLog::getInstance().getTable("flipper_multi_objective_total_cost").enableLogging();
  DataLog::getInstance().getTable("_" + whole_body_posture_optimization::PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging();
  PostureOptimizationResult result = po.findOptimalPosture(robot_pose, po.getDefaultJointPositions());
  (void)result;

  // Save results
  std::string logging_folder;
  if (param_utils::getParam(*node, "logging_folder", logging_folder)) {
    DataLog::getInstance().saveTablesToFolder(logging_folder);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
