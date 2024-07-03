#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include "whole_body_posture_optimization/util/timing.h"
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>

#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap/io.h>
#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <grid_map_msgs/GridMap.h>


static bool pose_updated_;
static Eigen::Isometry3d robot_pose_;

static bool state_updated_;
static std::vector<double> previous_state_;
static std::vector<double> state_;
static std::vector<std::string> active_joint_names_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

void stateCb(const sensor_msgs::JointState& joint_state_msg) {
  bool state_changed = false;
  for (unsigned int joint_idx = 0; joint_idx < active_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg.name.begin(), joint_state_msg.name.end(), active_joint_names_[joint_idx]);
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

  ros::Publisher heightmap_pub = pnh.advertise<grid_map_msgs::GridMap>("heightmap", 10, true);

  // Load environment
  ros::NodeHandle sdf_model_nh(pnh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);

  // Create pose predictor
  auto pose_predictor_name = pnh.param<std::string>("pose_predictor", "sdf_contact_estimation::SDFContactEstimation");
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor;
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    ros::NodeHandle pose_predictor_nh(pnh, "sdf_pose_predictor");
    ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pose_predictor_nh, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
  } else if (pose_predictor_name == "hector_heightmap_pose_prediction::HeightmapPosePredictor") {
    // Load heightmap
    ros::NodeHandle heightmap_nh(pnh, "heightmap");
    std::string map_path;
    if (!heightmap_nh.getParam("map_bag_file_path", map_path)) {
      ROS_ERROR_STREAM("Could not load mandatory parameter '" << heightmap_nh.getNamespace() << "/map_bag_file_path'");
      return 0;
    }
    hector_world_heightmap::MapBag<double>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<double>(map_path);
    if (!map_bag) {
      ROS_ERROR_STREAM("Failed to load map from path '" << map_path << "'");
      return 0;
    }
    heightmap_pub.publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
    urdf::Model urdf;
    if (!urdf.initParam("/robot_description")) {
      ROS_ERROR_STREAM("Failed to load urdf.");
      return 0;
    }
    auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf);
    robot_heightmap_provider->disableHeightmapCache();
    auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
  } else {
    ROS_ERROR_STREAM("Unknown pose predictor '" << pose_predictor_name << "'.");
    return 0;
  }

  whole_body_posture_optimization::WholeBodyPostureOptimization po(pnh, pose_predictor, sdf_model);

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
  active_joint_names_ = po.getRobotModel()->getActiveJointNames();
  state_ = po.getDefaultJointPositions();
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
        last_result = po.findOptimalPosture(robot_pose_, state_);
        reset = false;
      } else {
//          last_result = po_->findOptimalPosture(robot_pose_, state_, &last_result);
        // Always initialize optimization from default joint position
        // No movement penalty (do not use previous solution)
        last_result = po.findOptimalPosture(robot_pose_, state_);
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
