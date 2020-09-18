#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include <whole_body_posture_optimization/timing.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/body_shapes/robot_shape_from_urdf.h>

static bool pose_updated_;
static Eigen::Affine3d robot_pose_;

static bool state_updated_;
static std::vector<double> previous_state_;
static std::vector<double> state_;
static std::vector<std::string> actuated_joint_names_;

static std::shared_ptr<whole_body_posture_optimization::WholeBodyPostureOptimization> po_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

void stateCb(const sensor_msgs::JointState& joint_state_msg) {
  bool state_changed = false;
  for (unsigned int joint_idx = 0; joint_idx < actuated_joint_names_.size(); ++joint_idx) {
    std::vector<std::string>::const_iterator it = std::find(joint_state_msg.name.begin(), joint_state_msg.name.end(), actuated_joint_names_[joint_idx]);
    if (it != joint_state_msg.name.end()) {
      size_t index = static_cast<size_t>(it - joint_state_msg.name.begin());
      state_[joint_idx] = joint_state_msg.position[index];

      if (std::abs(state_[joint_idx] - previous_state_[joint_idx]) > 0.01) {
        state_changed = true;
      }
    }
  }

  if (state_changed) {
    state_updated_ = true;
    previous_state_ = state_;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle pnh("~");

  // Create robot shape
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape = std::make_shared<sdf_contact_estimation::RobotShapeFromUrdf>(pnh);
  po_ = std::make_shared<whole_body_posture_optimization::WholeBodyPostureOptimization>(pnh, robot_shape);

  // Create environment
  std::string scenario = pnh.param("scenario", std::string("flat"));
  if (scenario == "sdf") {
    std::string sdf_file_path;
    if (!pnh.getParam("sdf_file_path", sdf_file_path)) {
      ROS_ERROR_STREAM("No sdf file path found in namespace '" + pnh.getNamespace() + "/sdf_file_path.");
      return 0;
    }

    if (!po_->loadSdfFromFile(sdf_file_path, 0.4f)) {
      return 0;
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> cloud = sdf_contact_estimation::createScenarioFromName(scenario);
    po_->updateWorldModel(cloud);
  }

  // Load robot pose
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

  // Set default joint state
  actuated_joint_names_ = po_->getRobotModel()->getActuatedJointNames();
  state_ = po_->getDefaultJointPositions();
  previous_state_ = state_;
  state_updated_ = true;

  // Subscribers
  ros::Subscriber robot_pose_sub = pnh.subscribe("set_robot_pose", 10, &poseCB);
  ros::Subscriber joint_state_sub = pnh.subscribe("/joint_states", 10, &stateCb);

  // Update loop
  ros::Rate rate(10);
  whole_body_posture_optimization::PostureOptimizationResult last_result;
  while (ros::ok()) {
    ros::spinOnce();
    if (pose_updated_ || state_updated_) {
      auto start = std::chrono::high_resolution_clock::now();
      if (reset) {
        last_result = po_->findOptimalPosture(robot_pose_, state_);
        reset = false;
      } else {
//          last_result = po_->findOptimalPosture(robot_pose_, state_, &last_result);
        // Always initialize optimization from default joint position
        // No movement penalty (do not use previous solution)
          last_result = po_->findOptimalPosture(robot_pose_, state_);

      }
      std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
      ROS_INFO_STREAM("Complete optimization took " << elapsed.count()*1000.0 << " ms.");
      pose_updated_ = false;
      state_updated_ = false;
    }
    rate.sleep();
  }


  ros::spin();

  return 0;
}
