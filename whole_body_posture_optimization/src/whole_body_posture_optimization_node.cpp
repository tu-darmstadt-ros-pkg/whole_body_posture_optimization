#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_posture_optimization/timing.h>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

nav_msgs::Path createPath(double start_x, double stop_x, double step, const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()) {
  nav_msgs::Path path;
  double dir = start_x < stop_x ? 1 : -1;
  for (double x = start_x; dir * x < dir * stop_x; x+=dir*step) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, 0, 0) + offset;
    tf::poseEigenToMsg(pose, pose_msg.pose);

    path.poses.push_back(pose_msg);
  }
  return path;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
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

  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, pose_predictor, sdf_model);


  // Create path
//  nav_msgs::Path path;
//  if (scenario == "step" || scenario == "highstep") {
//    // step
//    path = createPath(0.14, 1.7, 0.02, Eigen::Vector3d(0, 0, -1.9));
////      path = createPath(1.7, 0.14, -0.02, Eigen::Vector3d(0, 0, -1.9));
//  } else if (scenario == "ramp") {
//    // ramp
//    path = createPath(-1, 1, 0.02, Eigen::Vector3d(0, 0, -1));
//  } else if (scenario == "sdf") {
//    double path_start, path_stop, path_step;
//    pnh.param("path_start", path_start, 0.0);
//    pnh.param("path_stop", path_stop, 1.5);
//    pnh.param("path_step", path_step, 0.02);
//    path = createPath(path_start, path_stop, path_step);
//  }

  nav_msgs::Path path;
  double path_start, path_stop, path_step;
  pnh.param("path_start", path_start, 0.0);
  pnh.param("path_stop", path_stop, 1.5);
  pnh.param("path_step", path_step, 0.02);
  path = createPath(path_start, path_stop, path_step);

  // Compute optimal postures
  auto result = po.findOptimalPostures(path);
  auto trajectory = po.computeTimedTrajectory(result);

  // Print and save results
  whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
  whole_body_posture_optimization::Timing::printTimeInfos();
//  std::string stability_info_save_folder;
//  if (pnh.getParam("stability_info_save_folder", stability_info_save_folder)) {
//    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
//    std::string save_path = stability_info_save_folder + "/stability_" + time_stamp + ".dat";
//    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectories, save_path);
//  }

  ros::spin();

  return 0;
}
