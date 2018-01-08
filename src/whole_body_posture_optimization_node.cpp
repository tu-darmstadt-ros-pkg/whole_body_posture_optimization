#include <ros/ros.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>

bool pose_updated_;
Eigen::Affine3d robot_pose_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "whole_body_posture_optimization_node");
  ros::NodeHandle pnh("~");

  ros::Subscriber robot_pose_sub = pnh.subscribe("set_robot_pose", 10, &poseCB);

  // Create robot shape
  sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape = std::make_shared<argo_tracker::ArgoTrackerShape>();
  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, tracker_shape);
  // create environment
  std::string scenario = pnh.param("scenario", std::string("flat"));
  pcl::PointCloud<pcl::PointXYZ> cloud = sdf_contact_estimation::createScenarioFromName(scenario);
  po.updateWorldModel(cloud);

  if (pnh.param("interactive", false)) {
    // Try with one pose first
    robot_pose_ = Eigen::Affine3d::Identity();
    robot_pose_.translation() = Eigen::Vector3d(0.946, 0, -1.96);
//    robot_pose_.translation() = Eigen::Vector3d(0.5, 0, -1.96);
    pose_updated_ = true;
    bool reset = true;

    ros::Rate rate(10);
    whole_body_posture_optimization::PostureOptimizationResult last_result;
    while (ros::ok()) {
      ros::spinOnce();
      if (pose_updated_) {
        auto start = std::chrono::high_resolution_clock::now();
        if (reset) {
          last_result = po.findOptimalPosture(robot_pose_, po.getDefaultJointPositions(), po.getDefaultJointPositions());
          reset = false;
        } else {
//          last_result = po.findOptimalPosture(robot_pose_, last_result.result_joint_angles, last_result.result_joint_angles, &last_result.result_contact);
          last_result = po.findOptimalPosture(robot_pose_, po.getDefaultJointPositions(), last_result.result_joint_angles, &last_result.result_contact);
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
    // step
    double direction = 1.0;
    double step = direction * 0.02;
    double x_offset = 1;
    for (double x = (direction * -0.5) + x_offset; x < (direction * 0.75) + x_offset; x+=step) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "world";

//      Eigen::Affine3d pose(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
      pose.translation() = Eigen::Vector3d(x, 0, -1.9);
      tf::poseEigenToMsg(pose, pose_msg.pose);

      path.poses.push_back(pose_msg);
    }

    // ramp
//    double step = 0.02;
//    for (double x = -1.0; x < 1.0; x+=step) {
//      geometry_msgs::PoseStamped pose_msg;
//      pose_msg.header.frame_id = "world";

////      Eigen::Affine3d pose(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
//      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
//      pose.translation() = Eigen::Vector3d(x, 0, -1);
//      tf::poseEigenToMsg(pose, pose_msg.pose);

//      path.poses.push_back(pose_msg);
//    }
    ROS_INFO_STREAM("Running optimization");
    // Compute optimal postures
    auto result = po.findOptimalPostures(path);
    po.computeTimedTrajectory(result);
  }

  ros::spin();

  return 0;
}
