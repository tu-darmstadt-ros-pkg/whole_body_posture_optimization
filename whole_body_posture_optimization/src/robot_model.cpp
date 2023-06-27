#include <whole_body_posture_optimization/robot_model.h>

#include <whole_body_posture_optimization/common.h>
#include <sdf_contact_estimation/visualisation.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <eigen_conversions/eigen_msg.h>


namespace whole_body_posture_optimization {

RobotModel::RobotModel(ros::NodeHandle &nh)
  : nh_(nh), num_joints_(0), collision_group_(""), total_mass_(0)
{
  // Parameter loading
  if(!nh.getParam("/robot_description", robot_description_)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
  }

  if (!nh.getParam("collision_group", collision_group_)) {
    ROS_WARN_STREAM("Failed to load collision group name.");
  }

  // Loading moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  moveit_robot_model_ = robot_model_loader.getModel();
  if (!moveit_robot_model_.get()){
    ROS_ERROR("Couldn't load robot model!");
  }
  moveit_robot_state_ = std::make_shared<robot_state::RobotState>(moveit_robot_model_);

  // Loading urdf
  urdf_model_ = moveit_robot_model_->getURDF();

  // Load differentiable kinematics
  ad_tree_ = std::make_shared<ad_kinematics::Tree>(urdf_model_);
  num_joints_ = ad_tree_->getNumActuatedJoints();
  joint_names_ = ad_tree_->getActuatedJointNames();

  for (unsigned int i = 0; i < joint_names_.size(); ++i) {
    updateActuatedJointMapping(joint_names_[i], i);
  }

  // Second pass to find mimics
  for (unsigned int i = 0; i < joint_names_.size(); i++) {
    updateJointMimicMapping(joint_names_[i], i);
  }

  if (!collision_group_.empty()) {
    generateLinkBodyDecompositions(0.02); //TODO resolution?
  }

  // Publishers
  ros::NodeHandle pnh("~");
  robot_state_pub_ = pnh.advertise<moveit_msgs::DisplayRobotState>("robot_state", 10, true);
  collision_spheres_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("collision_spheres", 10, true);
}

std::shared_ptr<voxblox::TsdfMap> RobotModel::computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance, float voxel_size, const std::vector<double>& joint_state)
{
  // Create empty SDF
  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = voxel_size;
  std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(tsdf_config);

  return computeTsdfFromRobotModel(link_names, truncation_distance, tsdf, joint_state);
}

std::shared_ptr<voxblox::TsdfMap> RobotModel::computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance,
                                                                  const std::shared_ptr<voxblox::TsdfMap> &init_tsdf, const std::vector<double>& joint_state)
{
  if (!init_tsdf) {
    ROS_ERROR_STREAM("TSDF is null");
    return std::shared_ptr<voxblox::TsdfMap>();
  }

  // Create copy
  std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(init_tsdf->getTsdfLayer());

  moveit_robot_state_->setToDefaultValues();
  if (!joint_state.empty()) {
    updateJointAngles(joint_state);
  }

  // Iterate over requested links
  for (const std::string& link_name: link_names) {

    // Get the collision geometry
    unsigned int index;
    try {
      index = body_decomposition_index_map_.at(link_name);
    } catch (const std::out_of_range&) {
      ROS_WARN_STREAM("Didn't find a collision geometry for link '" << link_name << "'.");
      continue;
    }

    // Transform the link model according to current robot state
    collision_detection::PosedBodySphereDecompositionPtr sphere_representation = std::make_shared<collision_detection::PosedBodySphereDecomposition>(body_decompositions_[index]);
    sphere_representation->updatePose(moveit_robot_state_->getGlobalLinkTransform(link_name));
    // Iterate over spheres and compute sdf
    for (unsigned int j = 0; j < sphere_representation->getCollisionSpheres().size(); j++) {
      Eigen::Vector3f center = sphere_representation->getSphereCenters()[j].cast<float>();
      float radius = static_cast<float>(sphere_representation->getCollisionSpheres()[j].radius_);

      // Cube around center with size radius + truncation_distance
      float voxel_size = tsdf->getTsdfLayerPtr()->voxel_size();
      for (float x = center.x() - radius - truncation_distance; x < center.x() + radius + truncation_distance; x += voxel_size) {
        for (float y = center.y() - radius - truncation_distance; y < center.y() + radius + truncation_distance; y += voxel_size) {
          for (float z = center.z() - radius - truncation_distance; z < center.z() + radius + truncation_distance; z += voxel_size) {
            Eigen::Vector3f point(x, y, z);
            float distance = (center - point).norm() - radius;

            voxblox::Point vpoint(x, y, z);
            voxblox::Layer<voxblox::TsdfVoxel>::BlockType::Ptr block_ptr = tsdf->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(vpoint);
            voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(vpoint);

            // Check if voxel is already set
            if (voxel.weight > 0) {
              // we want to keep the distance value closest to the surface/zero
              if (std::abs(distance) < std::abs(voxel.distance)) {
                voxel.distance = distance;
              }
            } else {
              voxel.weight = 1.0;
              voxel.distance = distance;
            }

          }
        }
      }
    }
  }

  return tsdf;
}

void RobotModel::updateJointAngles(const std::vector<double>& joint_angles)
{
  if (joint_angles.size() != actuated_joint_names_.size()) {
    ROS_ERROR_STREAM("[RobotModel::updateJointAngles] joint_angles size (" << joint_angles.size() << ") doesn't match number of joints in robot model (" << actuated_joint_names_.size() << ")");
    return;
  }
  joint_angles_ = joint_angles;

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = actuated_joint_names_;
  joint_state_msg.position = joint_angles;
  moveit::core::jointStateToRobotState(joint_state_msg, *moveit_robot_state_);
}

void RobotModel::updatePose(const Eigen::Isometry3d& robot_pose)
{
  robot_pose_ = robot_pose;
  updateMoveitRobotState(*moveit_robot_state_, robot_pose);
}

void RobotModel::updateRobotState(const Eigen::Isometry3d &robot_pose, const std::vector<double> joint_angles)
{
  updateJointAngles(joint_angles);
  updatePose(robot_pose);
}

void RobotModel::publishRobotState(const Eigen::Isometry3d &robot_pose, const std::vector<double> joint_angles)
{
  updateRobotState(robot_pose, joint_angles);
  publishRobotState();
}

void RobotModel::publishRobotState()
{
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*moveit_robot_state_, robot_state_msg.state);
  robot_state_pub_.publish(robot_state_msg);
}

void RobotModel::publishCollisionSpheres(const Eigen::Isometry3d &robot_pose, const std::vector<double> joint_angles)
{
  updateRobotState(robot_pose, joint_angles);
  publishCollisionSpheres();
}

void RobotModel::publishCollisionSpheres()
{
  visualization_msgs::MarkerArray marker_array;

  float transparency = 0.5f;

  // creating colors
  std_msgs::ColorRGBA robot_color;
  robot_color.r = 0;
  robot_color.b = 0.8f;
  robot_color.g = 0;
  robot_color.a = transparency;

  std_msgs::ColorRGBA world_links_color;
  world_links_color.r = 1.0f;
  world_links_color.g = 1.0f;
  world_links_color.b = 0;
  world_links_color.a = transparency;

  // creating sphere marker
  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = "world";
  sphere_marker.header.stamp = ros::Time(0);
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::Marker::ADD;
  sphere_marker.pose.orientation.x = 0;
  sphere_marker.pose.orientation.y = 0;
  sphere_marker.pose.orientation.z = 0;
  sphere_marker.pose.orientation.w = 1;
//  sphere_marker.lifetime = ros::Duration(0);

  int id = 0;

  const moveit::core::JointModelGroup* joint_group = moveit_robot_state_->getJointModelGroup(collision_group_);
  const std::vector<std::string>& group_link_names = joint_group->getLinkModelNames();

  std::map<std::string, unsigned int>::const_iterator map_iter;
  for (map_iter = body_decomposition_index_map_.begin(); map_iter != body_decomposition_index_map_.end(); map_iter++) {
    const std::string& link_name = map_iter->first;
    unsigned int link_index = map_iter->second;

    sphere_marker.ns = link_name;

    // selecting color
    if (std::find(group_link_names.begin(), group_link_names.end(), link_name) != group_link_names.end()) {
      sphere_marker.color = robot_color;
    } else {
      sphere_marker.color = world_links_color;
    }

    collision_detection::PosedBodySphereDecompositionPtr sphere_representation(new collision_detection::PosedBodySphereDecomposition(body_decompositions_[link_index]));
    sphere_representation->updatePose(moveit_robot_state_->getGlobalLinkTransform(link_name));
    for (unsigned int j = 0; j < sphere_representation->getCollisionSpheres().size(); j++)
    {
      tf::pointEigenToMsg(sphere_representation->getSphereCenters()[j], sphere_marker.pose.position);
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 2 * sphere_representation->getCollisionSpheres()[j].radius_;
      sphere_marker.id = id;
      id++;

      marker_array.markers.push_back(sphere_marker);
    }
  }

  collision_spheres_pub_.publish(marker_array);
}

std::vector<std::size_t> RobotModel::getActuatedJointIndices(const std::vector<std::string> &joints) const
{
  std::vector<std::size_t> indices;
  for (const std::string& joint: joints) {
    std::size_t idx = static_cast<std::size_t>(std::find(actuated_joint_names_.begin(), actuated_joint_names_.end(), joint) - actuated_joint_names_.begin());
    if (idx >= actuated_joint_names_.size()) {
      ROS_ERROR_STREAM("Joint '" << joint << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
  return indices;
}

std::string RobotModel::getParentJointName(const std::string &link_name)
{
  urdf::LinkConstSharedPtr link = urdf_model_->getLink(link_name);
  if (!link) {
    ROS_ERROR_STREAM("Couldn't find link '" << link_name << "'.");
    return "NOT FOUND";
  }
  return link->parent_joint->name;
}

bool RobotModel::getJointLimits(const std::string &joint_name, double &lower_limit, double &upper_limit)
{
  // Load limits from URDF
  urdf::JointConstSharedPtr joint = urdf_model_->getJoint(joint_name);
  if (!joint) {
    ROS_ERROR_STREAM("Couldn't find joint '" << joint_name << "'.");
    return false;
  }
  double urdf_upper_limit = joint->limits->upper;
  double urdf_lower_limit = joint->limits->lower;

  // Joint limits can be overridden by a parameter
  std::vector<double> limits;
  std::string parameter_name = "joint_limits/" + joint_name;
  if (nh_.getParam(parameter_name, limits)) {
    if (limits.size() != 2) {
      ROS_ERROR_STREAM("Parameter 'joint_limits/" << joint_name << "' is not size 2.");
      return false;
    }
    if (limits[0] < limits[1]) {
      upper_limit = limits[1];
      lower_limit = limits[0];
    } else {
      upper_limit = limits[0];
      lower_limit = limits[1];
    }
    if (upper_limit > urdf_upper_limit || lower_limit < urdf_lower_limit) {
      ROS_ERROR_STREAM("Configured joint limits '" << parameter_name << " exceed bounds.");
      ROS_ERROR_STREAM(upper_limit << " > " << urdf_upper_limit << " or " << lower_limit << " < " << urdf_lower_limit);
      return false;
    }
    return true;
  } else {
    // Use URDF values
    upper_limit = urdf_upper_limit;
    lower_limit = urdf_lower_limit;
    return true;
  }
}

unsigned int RobotModel::getNumJoints() const
{
  return num_joints_;
}

std::vector<std::string> RobotModel::getCollisionLinkNames()
{
  std::vector<std::string> collision_links;
  for (const auto& kv: body_decomposition_index_map_) {
    collision_links.push_back(kv.first);
  }
  return collision_links;
}

std::vector<std::string> RobotModel::getCollisionGroupNames()
{
  const moveit::core::JointModelGroup* joint_group = moveit_robot_state_->getJointModelGroup(collision_group_);
  const std::vector<std::string>& group_link_names = joint_group->getLinkModelNames();

  std::vector<std::string> collision_links;
  for (const auto& kv: body_decomposition_index_map_) {
    std::string link_name = kv.first;
    if (std::find(group_link_names.begin(), group_link_names.end(), link_name) != group_link_names.end()) {
      collision_links.push_back(kv.first);
    }
  }
  return collision_links;
}

const std::vector<collision_detection::CollisionSphere>& RobotModel::getCollisionSpheres(const std::string& link_name)
{
  unsigned int idx = body_decomposition_index_map_.at(link_name);
  return body_decompositions_[idx]->getCollisionSpheres();
}

std::vector<std::size_t> RobotModel::getChainToJointIndexMapping()
{
  return getActuatedJointIndices(ad_tree_->getActuatedJointNames());
}

std::vector<int> RobotModel::getParamToChainIndexMapping(const std::vector<std::string> &free_joints)
{
  return ad_tree_->getJointQIndices(free_joints);
}

void RobotModel::generateLinkBodyDecompositions(double resolution)
{
  const std::vector<const moveit::core::LinkModel*>& link_models = moveit_robot_model_->getLinkModelsWithCollisionGeometry();
  for (const moveit::core::LinkModel* link_model: link_models) {
    if (link_model->getShapes().empty())
    {
      ROS_WARN("No collision geometry for link model %s though there should be", link_model->getName().c_str());
      continue;
    }

    ROS_DEBUG("Generating model for %s", link_model->getName().c_str());

    collision_detection::BodyDecompositionConstPtr bd = std::make_shared<collision_detection::BodyDecomposition>(
      link_model->getShapes(), link_model->getCollisionOriginTransforms(), resolution, 0.0);
    body_decompositions_.push_back(bd);
    body_decomposition_index_map_[link_model->getName()] = body_decompositions_.size() - 1;
  }
}

void RobotModel::updateActuatedJointMapping(const std::string& joint_name, unsigned int joint_index)
{
  urdf::JointConstSharedPtr joint_ptr = urdf_model_->getJoint(joint_name);
  if (!joint_ptr) {
    ROS_ERROR_STREAM("Couldn't find joint '" << joint_name << "'.");
  }

  bool is_fixed = joint_ptr->type != urdf::Joint::REVOLUTE && joint_ptr->type != urdf::Joint::CONTINUOUS && joint_ptr->type != urdf::Joint::PRISMATIC;
  if (is_fixed) {
    return;
  }

  // Special case for fake fixed joints
  if (joint_ptr->type == urdf::Joint::REVOLUTE && joint_ptr->limits->upper == 0.0 && joint_ptr->limits->lower == 0.0) {
    return;
  }

  if (joint_ptr->mimic) {
    return;
  }

  actuated_joint_names_.push_back(joint_name);
  actuated_idx_to_joint_idx_.push_back(joint_index);
  actuated_joint_mimics_.emplace_back();
  ROS_INFO_STREAM("Joint is actuated.");
}

void RobotModel::updateJointMimicMapping(const std::string& joint_name, unsigned int joint_index)
{
  urdf::JointConstSharedPtr joint_ptr = urdf_model_->getJoint(joint_name);
  if (!joint_ptr) {
    ROS_ERROR_STREAM("Couldn't find joint '" << joint_name << "'.");
  }
  if (joint_ptr->mimic) {
    std::vector<std::string>::const_iterator it = std::find(actuated_joint_names_.begin(), actuated_joint_names_.end(), joint_ptr->mimic->joint_name);
    if (it != actuated_joint_names_.end()) {
      ROS_INFO_STREAM("Adding mimic from " << joint_ptr->mimic->joint_name << " to " << joint_name);
      unsigned int mimic_idx = it - actuated_joint_names_.begin();
      JointMimic mimic(joint_index, joint_ptr->mimic->offset, joint_ptr->mimic->multiplier);
      actuated_joint_mimics_[mimic_idx].push_back(mimic);
    }
  }
}

std::vector<std::string> RobotModel::getActuatedJointNames() const
{
  return actuated_joint_names_;
}

double RobotModel::getTotalMass() const
{
  return total_mass_;
}

robot_model::RobotModelPtr RobotModel::getMoveitRobotModel() const
{
  return moveit_robot_model_;
}

robot_state::RobotState RobotModel::getMoveitRobotState() const
{
  return *moveit_robot_state_;
}
std::shared_ptr<ad_kinematics::Tree> RobotModel::getADTree() const
{
  return ad_tree_;
}


}
