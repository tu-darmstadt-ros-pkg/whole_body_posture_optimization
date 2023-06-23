#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/robot_model.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>


bool joint_state_received_ = false;
static std::vector<double> state_;
static std::vector<std::string> actuated_joint_names_;

static std::vector<double> flipper_state_;
static std::vector<std::string> flipper_joint_names_;

void jointStateCB(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
  joint_state_received_ = true;
  for (unsigned int joint_idx = 0; joint_idx < actuated_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), actuated_joint_names_[joint_idx]);
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
  ros::init(argc, argv, "com_visualisation_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe("/joint_states", 10, &jointStateCB);
  ros::NodeHandle pnh("~");
  ros::Publisher com_pub = pnh.advertise<geometry_msgs::PointStamped>("com", 10, true);
  ros::Publisher com_pub_2 = pnh.advertise<geometry_msgs::PointStamped>("com_pose_predictor", 10, true);

  whole_body_posture_optimization::RobotModel model(pnh);
  actuated_joint_names_ = model.getActuatedJointNames();
  state_.resize(actuated_joint_names_.size(), 0.0);

  ros::NodeHandle pose_predictor_nh(pnh, "pose_predictor");
  ros::NodeHandle shape_model_nh(pose_predictor_nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  flipper_joint_names_ = shape_model->jointNames();
  flipper_state_.resize(flipper_joint_names_.size(), 0.0);


  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (joint_state_received_) {
      Eigen::Vector3d com;
      model.computeCenterOfMass(state_, com);
      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = "base_link";
      tf::pointEigenToMsg(com, point_msg.point);
      com_pub.publish(point_msg);

      shape_model->updateJointPositions(flipper_state_);
      com = shape_model->centerOfMass();
      tf::pointEigenToMsg(com, point_msg.point);
      com_pub_2.publish(point_msg);

      joint_state_received_ = false;
    }

    rate.sleep();
  }

  return 0;
}
