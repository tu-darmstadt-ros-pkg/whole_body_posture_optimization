#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <whole_body_posture_optimization/robot_model.h>


sensor_msgs::JointState last_joint_state_;
bool joint_state_received_;
std::vector<unsigned int> state_idx_to_rbdl_index;

void jointStateCB(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
  if (!joint_state_received_) {
    joint_state_received_ = true;
  }
  last_joint_state_ = *joint_state_msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "com_visualisation_node");
  ros::NodeHandle nh;

  std::string robot_description;
  if(!nh.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
    return false;
  }

  ros::Subscriber state_sub = nh.subscribe("/joint_states", 10, &jointStateCB);
  ros::NodeHandle pnh("~");
  ros::Publisher com_pub = pnh.advertise<geometry_msgs::PointStamped>("com", 10, true);

  whole_body_posture_optimization::RobotModel model(robot_description);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (joint_state_received_) {
      state_idx_to_rbdl_index = model.getJointIndices(last_joint_state_.name);
      std::vector<double> rbdl_state(state_idx_to_rbdl_index.size(), 0);
      for (unsigned int state_idx = 0; state_idx < last_joint_state_.position.size(); state_idx++) {
        rbdl_state[state_idx_to_rbdl_index[state_idx]] = last_joint_state_.position[state_idx];
      }
      Eigen::Vector3d com;
      model.computeCenterOfMass(rbdl_state, com);
      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = "base_link";
      tf::pointEigenToMsg(com, point_msg.point);
      com_pub.publish(point_msg);
    }

    rate.sleep();
  }

  return 0;
}
