#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <whole_body_posture_optimization/whole_body_posture_optimization.h>
#include "whole_body_posture_optimization/util/timing.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>

#ifdef WBPO_HAVE_HEIGHTMAP
#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap/io.h>
#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#endif

namespace param_utils = whole_body_posture_optimization::param_utils;

static bool pose_updated_;
static Eigen::Isometry3d robot_pose_;

static bool state_updated_;
static std::vector<double> previous_state_;
static std::vector<double> state_;
static std::vector<std::string> active_joint_names_;

void poseCB(const geometry_msgs::msg::PoseStamped& pose_msg) {
  tf2::fromMsg(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

void stateCb(const sensor_msgs::msg::JointState& joint_state_msg) {
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("manual_control_node");

  // Load environment
  auto sdf_model_node = node->create_sub_node("sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_node);
  sdf_model->loadFromServer(sdf_model_node);

  // Create pose predictor
  auto pose_predictor_name = param_utils::param(*node, "pose_predictor", std::string("sdf_contact_estimation::SDFContactEstimation"));
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor;
  if (pose_predictor_name == "sdf_contact_estimation::SDFContactEstimation") {
    auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(node);
    auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(node, shape_model, sdf_model);
    sdf_pose_predictor->enableVisualisation(false);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
  } else if (pose_predictor_name == "hector_heightmap_pose_prediction::HeightmapPosePredictor") {
#ifdef WBPO_HAVE_HEIGHTMAP
    auto heightmap_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("heightmap", rclcpp::QoS(10).transient_local());
    // Load heightmap
    std::string map_path;
    if (!param_utils::getParam(*node, "heightmap.map_bag_file_path", map_path)) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load mandatory parameter 'heightmap.map_bag_file_path'");
      return 0;
    }
    hector_world_heightmap::MapBag<double>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<double>(map_path);
    if (!map_bag) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load map from path '" << map_path << "'");
      return 0;
    }
    heightmap_pub->publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
    urdf::Model urdf;
    std::string robot_description;
    if (!param_utils::getParam(*node, "robot_description", robot_description) || !urdf.initString(robot_description)) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load urdf.");
      return 0;
    }
    auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf);
    robot_heightmap_provider->disableHeightmapCache();
    auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
    pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
#else
    RCLCPP_ERROR_STREAM(node->get_logger(), "This package was built without heightmap pose prediction support "
                        "(build with -DWITH_HEIGHTMAP_POSE_PREDICTION=ON).");
    return 0;
#endif
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Unknown pose predictor '" << pose_predictor_name << "'.");
    return 0;
  }

  whole_body_posture_optimization::WholeBodyPostureOptimization po(node, pose_predictor, sdf_model);

  // Load robot pose
  std::vector<double> requested_pose_vec = param_utils::param(*node, "robot_pose", std::vector<double>(6, 0));
  if (requested_pose_vec.size() != 6) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Requested pose vector has wrong size (" << requested_pose_vec.size() << ") instead of 6. Using identity.");
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
  auto robot_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("set_robot_pose", 10, &poseCB);
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, &stateCb);

  // Update loop
  rclcpp::Rate rate(10);
  whole_body_posture_optimization::PostureOptimizationResult last_result;
  int loop_counter = 0;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    // Periodically re-publish the current robot state so late-joining
    // subscribers (e.g. RViz) always receive it
    if (++loop_counter % 10 == 0) {
      po.getRobotModel()->publishRobotState();
    }
    if (pose_updated_ || state_updated_) {
      auto start = std::chrono::high_resolution_clock::now();
      if (reset) {
        last_result = po.findOptimalPosture(robot_pose_, state_);
        reset = false;
      } else {
        // Always initialize optimization from default joint position
        // No movement penalty (do not use previous solution)
        last_result = po.findOptimalPosture(robot_pose_, state_);
      }
      std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
      RCLCPP_INFO_STREAM(node->get_logger(), "Complete optimization took " << elapsed.count()*1000.0 << " ms.");
      pose_updated_ = false;
      state_updated_ = false;
    }
    try {
      rate.sleep();
    } catch (const std::runtime_error&) {
      break; // context invalidated by Ctrl-C during sleep
    }
  }

  rclcpp::shutdown();
  return 0;
}
