#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_planning_plotting_utils/data_log.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>

using whole_body_planning_plotting_utils::DataLog;
using namespace whole_body_posture_optimization;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cost_function_logging_node");
  ros::NodeHandle pnh("~");

  // Load environment
  ros::NodeHandle sdf_model_nh(pnh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);

  // Create pose predictor
  // Create robot model
  ros::NodeHandle pose_predictor_nh(pnh, "pose_predictor");
  ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(false);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  WholeBodyPostureOptimization po(pnh, pose_predictor, sdf_model);

  // Load robot pose
  std::vector<double> requested_pose_vec = pnh.param<std::vector<double>>("robot_pose", std::vector<double>(6, 0));
  if (requested_pose_vec.size() != 6) {
    ROS_WARN_STREAM("Requested pose vector has wrong size (" << requested_pose_vec.size() << ") instead of 6. Using identity.");
    requested_pose_vec.resize(6, 0);
  }
  Eigen::Isometry3d robot_pose;
  robot_pose = Eigen::AngleAxisd(requested_pose_vec[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(requested_pose_vec[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(requested_pose_vec[3], Eigen::Vector3d::UnitX());
  robot_pose.translation() = Eigen::Vector3d(requested_pose_vec[0], requested_pose_vec[1], requested_pose_vec[2]);

  // Optimize
  DataLog::getInstance().getTable("flipper_multi_objective_cost").enableLogging();
  DataLog::getInstance().getTable("_" + whole_body_posture_optimization::PROBABILITY_DISTRIBUTION_TABLE_NAME).enableLogging();
  PostureOptimizationResult result = po.findOptimalPosture(robot_pose, po.getDefaultJointPositions());

  // Save results
  std::string logging_folder;
  if (pnh.getParam("logging_folder", logging_folder)) {
    DataLog::getInstance().saveTablesToFolder(logging_folder);
  }

  ros::spin();

  return 0;
}
