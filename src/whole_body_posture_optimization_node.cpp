#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/robot_shape_from_urdf.h>
#include <whole_body_posture_optimization/timing.h>

nav_msgs::Path createPath(double start_x, double stop_x, double step, const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()) {
  nav_msgs::Path path;
  double dir = start_x < stop_x ? 1 : -1;
  for (double x = start_x; dir * x < dir * stop_x; x+=step) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(x, 0, 0) + offset;
    tf::poseEigenToMsg(pose, pose_msg.pose);

    path.poses.push_back(pose_msg);
  }
  return path;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle pnh("~");

  // Create robot shape
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape = std::make_shared<sdf_contact_estimation::RobotShapeFromUrdf>(pnh);
  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, robot_shape);

  // Create environment
  std::string scenario = pnh.param("scenario", std::string("flat"));
  if (scenario == "sdf") {
    std::string sdf_file_path;
    if (!pnh.getParam("sdf_file_path", sdf_file_path)) {
      ROS_ERROR_STREAM("No sdf file path found in namespace '" + pnh.getNamespace() + "/sdf_file_path.");
      return 0;
    }

    if (!po.loadSdfFromFile(sdf_file_path, 0.4f)) {
      return 0;
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> cloud = sdf_contact_estimation::createScenarioFromName(scenario);
    po.updateWorldModel(cloud);
  }

  // Create path
  nav_msgs::Path path;
  if (scenario == "step" || scenario == "highstep") {
    // step
    path = createPath(0.14, 1.7, 0.02, Eigen::Vector3d(0, 0, -1.9));
//      path = createPath(1.7, 0.14, -0.02, Eigen::Vector3d(0, 0, -1.9));
  } else if (scenario == "ramp") {
    // ramp
    path = createPath(-1, 1, 0.02, Eigen::Vector3d(0, 0, -1));
  } else if (scenario == "sdf") {
    double path_start, path_stop, path_step;
    pnh.param("path_start", path_start, 0.0);
    pnh.param("path_stop", path_stop, 1.5);
    pnh.param("path_step", path_step, 0.02);
    path = createPath(path_start, path_stop, path_step);
  }

  // Compute optimal postures
  auto result = po.findOptimalPostures(path);
  auto trajectories = po.computeTimedTrajectory(result);

  // Print and save results
  whole_body_posture_optimization::WholeBodyPostureOptimization::printResultInfo(result);
  whole_body_posture_optimization::Timing::printTimeInfos();
  po.printContactEstimationStatistics();
  std::string stability_info_save_folder;
  if (pnh.getParam("stability_info_save_folder", stability_info_save_folder)) {
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = stability_info_save_folder + "/stability_" + time_stamp + ".dat";
    whole_body_posture_optimization::WholeBodyPostureOptimization::saveResultInfo(result, trajectories, save_path);
  }

  ros::spin();

  return 0;
}
