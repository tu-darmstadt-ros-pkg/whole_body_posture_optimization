#include <ros/ros.h>

// system
#include <fstream>
#include <ctime>

// tf
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// msgs
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>

#include <move_base_lite_msgs/FollowPathActionGoal.h>
#include <move_base_lite_msgs/FollowPathActionFeedback.h>
#include <move_base_lite_msgs/FollowPathActionResult.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/visualisation.h>
#include <sdf_contact_estimation/body_shapes/argo_tracker_robot_shape.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/common.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox/io/layer_io.h>

enum State {
  IDLE,
  TRACKING
};

struct DataPoint {
  double time;
  double pos_x;
  double pos_y;
  double stability;
};

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
std::shared_ptr<whole_body_posture_optimization::RobotModel> robot_model_;
std::vector<std::string> joint_names_;

sensor_msgs::JointState last_joint_state_;
bool joint_state_received_;

std::shared_ptr<voxblox::EsdfMap> esdf_;
State current_state_;
nav_msgs::Path path_;
unsigned int current_waypoint_id_; // id of leg which is currently approached
                                   // corresponds to idx of path, that is already completed
ros::Time start_time_;
ros::Time end_time_;
ros::Timer recording_timer_;
// Recorded data
std::vector<DataPoint> data_;

// Parameters
std::string base_link_name_;
std::string save_folder_;
std::vector<std::string> flipper_joint_names_;

ros::Publisher stability_vis_pub_;
ros::Publisher com_pub_;
double max_recording_duration_;
double recording_frequency_;


// Predeclare functions
void startRecordingCb(const std_msgs::EmptyConstPtr&);
void recordingTimerCb(const ros::TimerEvent& event);
void goalCb(const move_base_lite_msgs::FollowPathActionGoalConstPtr& goal_msg);
void feedbackCb(const move_base_lite_msgs::FollowPathActionFeedbackConstPtr& feedback_msg);
void resultCb(const move_base_lite_msgs::FollowPathActionResultConstPtr&);
void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg);
void sdfCb(const voxblox_msgs::LayerConstPtr& layer_msg);
void recordStability(bool use_path);
void saveToDisk(std::string save_path);
std::vector<double> stateFromList(const std::vector<std::string>& joint_names);

std::vector<double> stateFromList(const std::vector<std::string>& joint_names) {
  std::vector<double> state(joint_names.size(), 0);
  for (unsigned int name_idx = 0; name_idx < joint_names.size(); name_idx++) {
    std::vector<std::string>::iterator it = std::find(last_joint_state_.name.begin(), last_joint_state_.name.end(), joint_names[name_idx]);
    if (it != last_joint_state_.name.end()) {
      unsigned int state_idx = it - last_joint_state_.name.begin();
      state[name_idx] = last_joint_state_.position[state_idx];
//      ROS_INFO_STREAM(last_joint_state_.name[state_idx] << ": " << last_joint_state_.position[state_idx]);
    } else {
      ROS_WARN_STREAM("Joint '" << joint_names[name_idx] << "' wasn't found in received joint state.");
    }
  }
  return state;
}

void startRecordingCb(const std_msgs::EmptyConstPtr&) {
  ROS_INFO_STREAM("Starting recording for " << max_recording_duration_ << " seconds.");
  start_time_ = ros::Time::now();
  ros::Duration recording_duration(max_recording_duration_);
  end_time_ = start_time_ + recording_duration;
  data_.clear();

  current_state_ = TRACKING;
  ros::NodeHandle nh;
  recording_timer_ = nh.createTimer(ros::Duration(recording_frequency_), &recordingTimerCb);
}

void recordingTimerCb(const ros::TimerEvent& event) {
  if (current_state_ = TRACKING) {
    if (event.current_expected > end_time_) {
      // Finish recording
      std::string time_stamp = whole_body_posture_optimization::getTimestamp();
      std::string save_path = save_folder_ + "/stability_benchmark_" + time_stamp + ".dat";
      ROS_INFO_STREAM("Saving results to '" << save_path << "'.");
      saveToDisk(save_path);
      current_state_ = IDLE;
      recording_timer_.stop();
    } else {
      recordStability(false);
    }
  }
}

void goalCb(const move_base_lite_msgs::FollowPathActionGoalConstPtr& goal_msg) {
  ROS_INFO_STREAM("Received path of size " << goal_msg->goal.target_path.poses.size());
  path_ = goal_msg->goal.target_path;
  current_waypoint_id_ = 0;
  start_time_ = ros::Time::now();
  data_.clear();
  current_state_ = TRACKING;
  recordStability(true);
}

void feedbackCb(const move_base_lite_msgs::FollowPathActionFeedbackConstPtr& feedback_msg) {
  if (current_state_ == TRACKING) {
    while (current_waypoint_id_ < feedback_msg->feedback.current_waypoint) {
      current_waypoint_id_++;
      recordStability(true);
    }
  }
}

void resultCb(const move_base_lite_msgs::FollowPathActionResultConstPtr&) {
  if (current_state_ == TRACKING) {
    current_waypoint_id_ = path_.poses.size()-1; // set to last waypoint
    recordStability(true);

    // Path execution finished, save to disk
    std::string time_stamp = whole_body_posture_optimization::getTimestamp();
    std::string save_path = save_folder_ + "/stability_benchmark_" + time_stamp + ".dat";
    ROS_INFO_STREAM("Saving results to '" << save_path << "'.");
    saveToDisk(save_path);
    current_state_ = IDLE;
  }
}

void sdfCb(const voxblox_msgs::LayerConstPtr& layer_msg) {
  ROS_INFO_STREAM("Received sdf update.");
  if (!esdf_) {
    voxblox::EsdfMap::Config esdf_config;
    esdf_config.esdf_voxel_size = static_cast<voxblox::FloatingPoint>(layer_msg->voxel_size);
    esdf_config.esdf_voxels_per_side = layer_msg->voxels_per_side;
    esdf_ = std::make_shared<voxblox::EsdfMap>(esdf_config);
  }
  if (!voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(*layer_msg, esdf_->getEsdfLayerPtr())) {
    ROS_ERROR_STREAM("Received invalid esdf layer msg.");
  }
  contact_estimation_->loadEsdf(esdf_, 0.4);
}

void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
  if (!joint_state_received_) {
    joint_state_received_ = true;
    last_joint_state_ = *joint_state_msg;
  } else {
    for (unsigned int joint_idx = 0; joint_idx < joint_state_msg->name.size(); joint_idx++) {
      const std::string& joint_name = joint_state_msg->name[joint_idx];
      std::vector<std::string>::iterator it = std::find(last_joint_state_.name.begin(), last_joint_state_.name.end(), joint_name);
      if (it != last_joint_state_.name.end()) {
        // we know this joint already, update position
        unsigned int idx = it - last_joint_state_.name.begin();
        last_joint_state_.position[idx] = joint_state_msg->position[joint_idx];
        last_joint_state_.velocity[idx] = joint_state_msg->velocity[joint_idx];
      } else {
        // new joint
        last_joint_state_.name.push_back(joint_name);
        last_joint_state_.position.push_back(joint_state_msg->position[joint_idx]);
        last_joint_state_.velocity.push_back(joint_state_msg->velocity[joint_idx]);
      }
    }
  }
}

void recordStability(bool use_path) {
  DataPoint dp;

  // Log Time
  ros::Time now = ros::Time::now();
  ros::Duration from_start = now - start_time_;
  dp.time = from_start.toSec();
  // world to base
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("world", base_link_name_, now, ros::Duration(1));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM("Failed to get world pose: " << ex.what());
    return;
  }
  Eigen::Affine3d world_to_base;
  tf::transformMsgToEigen(transform.transform, world_to_base);

  // Save position
  if (use_path) {
    ROS_INFO_STREAM("Reached waypoint [" << current_waypoint_id_ << "/" << path_.poses.size()-1 << "] at " <<
                    "[x=" << path_.poses[current_waypoint_id_].pose.position.x << "|y=" << path_.poses[current_waypoint_id_].pose.position.y << "]");
    dp.pos_x = path_.poses[current_waypoint_id_].pose.position.x;
    dp.pos_y = path_.poses[current_waypoint_id_].pose.position.y;
  } else {
    ROS_INFO_STREAM("Logging stability at [x=" << world_to_base.translation().x() << "|y=" << world_to_base.translation().y() << "]");
    dp.pos_x = world_to_base.translation().x();
    dp.pos_y = world_to_base.translation().y();
  }

  // Compute COM
  if (!joint_state_received_) {
    ROS_WARN_STREAM("Can't compute stability, no joint state received.");
    return;
  }
  Eigen::Vector3d com_base;
  robot_model_->computeCenterOfMass(stateFromList(joint_names_), com_base);

  // Estimate contact points and compute stability
  if (!esdf_) {
    ROS_WARN_STREAM("Didn't receive a world model yet. Can't compute contact.");
    return;
  }
  // Update flipper angles
  contact_estimation_->updateJointStates(stateFromList(flipper_joint_names_));
  Eigen::Affine3d base_to_com = Eigen::Affine3d::Identity();
  base_to_com.translation() = com_base;
  sdf_contact_estimation::ContactEstimationResult result;
  contact_estimation_->estimateContactPoints(world_to_base, base_to_com, 0.05, result);

  ROS_INFO_STREAM("Stability: " << result.min_stability);
  dp.stability = result.min_stability;
  data_.push_back(dp);

  // Publish visualization
  sdf_contact_estimation::publishPose(com_pub_, base_to_com, base_link_name_);
  sdf_contact_estimation::deleteAllMarkers(stability_vis_pub_);
  visualization_msgs::MarkerArray marker_array;
  sdf_contact_estimation::createPointMarkerArray(result.contact_points, marker_array, "world", "contact_points", Eigen::Vector3d(1, 0.5, 0), 0.03);
  sdf_contact_estimation::createPointMarkerArray(result.convex_hull_points, marker_array, "world", "convex_hull", Eigen::Vector3d(0, 1, 0), 0.04);
  sdf_contact_estimation::createConvexHullMarkerArrayWithStabilities(result.convex_hull_points, result.stabilities, marker_array);
  stability_vis_pub_.publish(marker_array);
}

void saveToDisk(std::string save_path) {
  std::ofstream file;
  file.open(save_path);

  file << "time pos_x pos_y stability" << std::endl;
  for (const DataPoint& d: data_) {
    file << d.time << " ";
    file << d.pos_x << " ";
    file << d.pos_y << " ";
    file << d.stability << std::endl;
  }

  file.close();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "stability_benchmark_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  current_state_ = IDLE;

  // Load parameters
  if (!pnh.getParam("save_folder", save_folder_)) {
    ROS_ERROR_STREAM("Parameter 'save_folder' not found. Exiting.");
    return 0;
  }
  pnh.param<std::string>("base_link_name", base_link_name_, "base_link");
  pnh.param("max_recording_duration", max_recording_duration_, 30.0);
  pnh.param("recording_frequency", recording_frequency_, 1.0);
  pnh.param<std::vector<std::string>>("flipper_joint_names", flipper_joint_names_, std::vector<std::string>());

  // Init tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  robot_model_ = std::make_shared<whole_body_posture_optimization::RobotModel>(nh);
  joint_names_ = robot_model_->getActuatedJointNames();
  sdf_contact_estimation::RobotShapeGeneratorBasePtr tracker_shape = std::make_shared<argo_tracker::ArgoTrackerShape>();
  contact_estimation_ = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(nh, tracker_shape);

  // Subscribers
  stability_vis_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("stability", 10, true);
  com_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("com", 10, true);
  ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, &jointStateCb);
  ros::Subscriber goal_sub = nh.subscribe("goal", 10, &goalCb);
  ros::Subscriber feedback_sub = nh.subscribe("feedback", 10, &feedbackCb);
  ros::Subscriber result_sub = nh.subscribe("result", 10, &resultCb);
  ros::Subscriber sdf_sub = nh.subscribe("sdf", 10, &sdfCb);

  ros::Subscriber start_recording_sub = pnh.subscribe("start_recording", 10, &startRecordingCb);

  ros::spin();

  return 0;
}
