#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <whole_body_posture_optimization/robot_model.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>


bool joint_state_received_ = false;
static std::vector<double> state_;
static std::vector<std::string> active_joint_names_;

static std::vector<double> flipper_state_;
static std::vector<std::string> flipper_joint_names_;

void jointStateCB(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg) {
  joint_state_received_ = true;
  for (unsigned int joint_idx = 0; joint_idx < active_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), active_joint_names_[joint_idx]);
    if (it != joint_state_msg->name.end()) {
      size_t index = static_cast<size_t>(it - joint_state_msg->name.begin());
      state_[joint_idx] = joint_state_msg->position[index];
    }
  }

  for (unsigned int joint_idx = 0; joint_idx < flipper_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), flipper_joint_names_[joint_idx]);
    if (it != joint_state_msg->name.end()) {
      size_t index = static_cast<size_t>(it - joint_state_msg->name.begin());
      flipper_state_[joint_idx] = joint_state_msg->position[index];
    }
  }

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("com_visualisation_node");

  auto state_sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, &jointStateCB);
  auto com_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("com", rclcpp::QoS(10).transient_local());
  auto com_pub_2 = node->create_publisher<geometry_msgs::msg::PointStamped>("com_pose_predictor", rclcpp::QoS(10).transient_local());

  whole_body_posture_optimization::RobotModel model(node);
  active_joint_names_ = model.getActiveJointNames();
  state_.resize(active_joint_names_.size(), 0.0);

  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(node);
  flipper_joint_names_ = shape_model->jointNames();
  flipper_state_.resize(flipper_joint_names_.size(), 0.0);


  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (joint_state_received_) {
      Eigen::Vector3d com = model.computeCenterOfMass(state_);
      geometry_msgs::msg::PointStamped point_msg;
      point_msg.header.frame_id = "base_link";
      point_msg.point = tf2::toMsg(com);
      com_pub->publish(point_msg);

      shape_model->updateJointPositions(flipper_state_);
      com = shape_model->centerOfMass();
      point_msg.point = tf2::toMsg(com);
      com_pub_2->publish(point_msg);

      joint_state_received_ = false;
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
