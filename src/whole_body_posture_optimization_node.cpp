#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/robot_shape_from_urdf.h>
#include <whole_body_posture_optimization/timing.h>

static bool pose_updated_;
static Eigen::Affine3d robot_pose_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

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

  ros::Subscriber robot_pose_sub = pnh.subscribe("set_robot_pose", 10, &poseCB);

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

  if (pnh.param("interactive", false)) {
    // Robot pose
    std::vector<double> requested_pose_vec = pnh.param<std::vector<double>>("robot_pose", std::vector<double>(6, 0));
    if (requested_pose_vec.size() != 6) {
      ROS_WARN_STREAM("Requested pose vector has wrong size (" << requested_pose_vec.size() << ") instead of 6. Using identity.");
      requested_pose_vec.resize(6, 0);
    }

    robot_pose_ = Eigen::AngleAxisd(requested_pose_vec[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(requested_pose_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(requested_pose_vec[3], Eigen::Vector3d::UnitX());
    robot_pose_.translation() = Eigen::Vector3d(requested_pose_vec[0], requested_pose_vec[1], requested_pose_vec[2]);

    pose_updated_ = true;
    bool reset = true;

    ros::Rate rate(10);
    whole_body_posture_optimization::PostureOptimizationResult last_result;
    while (ros::ok()) {
      ros::spinOnce();
      if (pose_updated_) {
        auto start = std::chrono::high_resolution_clock::now();
        if (reset) {
          last_result = po.findOptimalPosture(robot_pose_, po.getDefaultJointPositions(), std::vector<double>());
          reset = false;
        } else {
//          last_result = po.findOptimalPosture(robot_pose_, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
          // Always initialize optimization from default joint position
          // No movement penalty (do not use previous solution)
          last_result = po.findOptimalPosture(robot_pose_, po.getDefaultJointPositions(), std::vector<double>(), &last_result.result_contact);
        }
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
        ROS_INFO_STREAM("Complete optimization took " << elapsed.count()*1000.0 << " ms.");
        pose_updated_ = false;
      }
      rate.sleep();
    }
  } else {
    // create path
    nav_msgs::Path path;
    if (scenario == "step" || scenario == "highstep") {
      // step
      path = createPath(0.0, 1.7, 0.02, Eigen::Vector3d(0, 0, -1.9));
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
  }

  ros::spin();

  return 0;
}
