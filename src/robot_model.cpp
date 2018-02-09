#include <whole_body_posture_optimization/robot_model.h>

namespace whole_body_posture_optimization {

RobotModel::RobotModel(ros::NodeHandle &nh)
  : num_joints_(0)
{
  // Parameter loading
  if(!nh.getParam("/robot_description", robot_description_)) {
    ROS_ERROR_STREAM("Failed to load /robot_description.");
  }

  if (!nh.getParam("collision_group", collision_group_)) {
    ROS_ERROR_STREAM("Failed to load collision group name.");
  }

  // Loading urdf
  if (!urdf_model_.initString(robot_description_)) {
    ROS_ERROR_STREAM("Failed to parse urdf.");
  }

  // Loading rbdl model
  model_ptr_ = std::make_shared<rbdl::Model>();
  rbdl::Addons::URDFReadFromString(robot_description_.c_str(), model_ptr_.get(), false, false);
  ad_model_ptr_ = std::make_shared<ADModel>(*model_ptr_);
  ROS_INFO_STREAM("Loaded model:");
  ROS_INFO_STREAM(" -- dof: " << model_ptr_->dof_count);
  ROS_INFO_STREAM(" -- q_size: " << model_ptr_->q_size);
  num_joints_ = model_ptr_->dof_count;
  for (unsigned int body_id = 0; body_id < model_ptr_->mBodies.size(); body_id++) {
    std::string body_name = model_ptr_->GetBodyName(body_id);
    ROS_INFO_STREAM("Body id: " << body_id << ", name: " << body_name);
    ROS_INFO_STREAM("-- mass: " << model_ptr_->mBodies[body_id].mMass);
    if (body_id != 0) {
      ROS_INFO_STREAM("-- Parent joint: " << getParentJointName(body_name));
      ROS_INFO_STREAM("-- Joint type: " << model_ptr_->mJoints[body_id].mJointType);
      joint_names_.push_back(getParentJointName(body_name));
    }
  }

  // Loading moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit_robot_model_ = robot_model_loader.getModel();
  if (!moveit_robot_model_.get()){
    ROS_ERROR("Couldn't load robot model!");
  }
  moveit_robot_state_ = std::make_shared<robot_state::RobotState>(moveit_robot_model_);

  // ad kinematics
  if (collision_group_ != "") {
    ad_chain_ = std::make_shared<ad_kinematics::Chain>(moveit_robot_model_->getURDF(), moveit_robot_state_->getJointModelGroup(collision_group_));
    generateLinkBodyDecompositions(0.02); //TODO resolution?
  }

  // Publishers
  ros::NodeHandle pnh("~");
  robot_state_pub_ = pnh.advertise<moveit_msgs::DisplayRobotState>("robot_state", 10, true);
  collision_spheres_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("collision_spheres", 10, true);
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com, Eigen::MatrixXd &jacobian)
{
  if (joint_angles.size() != num_joints_) {
    ROS_ERROR_STREAM("[RobotModel::computeCenterOfMass] Joint positions size (" << joint_angles.size() << ") doesn't match number of joints in robot model (" << num_joints_ << ")");
    return;
  }
  rbdl::Math::VectorNd q(joint_angles.size());
  rbdl::Math::VectorNd qdot(joint_angles.size());
  for (unsigned int i = 0; i < joint_angles.size(); i++) {
    q(i) = joint_angles[i];
    qdot(i) = 0;
  }
  int ndirs = num_joints_;
  rbdl::Math::MatrixNd q_dirs = rbdl::Math::MatrixNd::Zero(num_joints_, ndirs);
  rbdl::Math::MatrixNd qdot_dirs = rbdl::Math::MatrixNd::Zero(num_joints_, ndirs);
  q_dirs.block(0, 0, num_joints_, num_joints_) = rbdl::Math::MatrixNd::Identity(num_joints_, num_joints_);
//  q_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Zero(ndof, ndof);
  qdot_dirs.block(0, 0, num_joints_, num_joints_) = rbdl::Math::MatrixNd::Zero(num_joints_, num_joints_);
//  qdot_dirs.block(0, ndof, ndof, ndof) = MatrixNd::Identity(ndof, ndof);

  double mass;
  rbdl::Math::Vector3d com2;
  jacobian = rbdl::Math::MatrixNd(3, ndirs);
  rbdl::AD::CalcCenterOfMass(*model_ptr_, *ad_model_ptr_, q, q_dirs, qdot, qdot_dirs, mass, com2, jacobian, 0, 0, 0, 0, true);
  com = com2;

  // Work-around because rbdl_reader assigns chassis mass to world frame
  rbdl::Body root = model_ptr_->mBodies[0];
  com = (mass * com + root.mMass * root.mCenterOfMass)/(mass + root.mMass);
  mass += root.mMass;
}

void RobotModel::computeCenterOfMass(const std::vector<double> &joint_angles, Eigen::Vector3d &com)
{
  Eigen::MatrixXd jacobian;
  computeCenterOfMass(joint_angles, com, jacobian);
}

std::shared_ptr<voxblox::TsdfMap> RobotModel::computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance)
{
  // Create empty SDF
  const float voxel_size = 0.05;
  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = voxel_size; // TODO parameter
  std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(tsdf_config);

  return computeTsdfFromRobotModel(link_names, truncation_distance, tsdf);
}

std::shared_ptr<voxblox::TsdfMap> RobotModel::computeTsdfFromRobotModel(const std::vector<std::string> &link_names, float truncation_distance,
                                                                  const std::shared_ptr<voxblox::TsdfMap> &init_tsdf)
{
  if (!init_tsdf) {
    ROS_ERROR_STREAM("TSDF is null");
    return std::shared_ptr<voxblox::TsdfMap>();
  }

  // Create copy
  std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>(init_tsdf->getTsdfLayer());

  updateMoveitRobotState(*moveit_robot_state_, Eigen::Affine3d::Identity());

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

//    ROS_INFO_STREAM("-- Computing tsdf for '" << link_name << "'.");
    // Transform the link model according to current robot state
    collision_detection::PosedBodySphereDecompositionPtr sphere_representation = std::make_shared<collision_detection::PosedBodySphereDecomposition>(body_decompositions_[index]);
    sphere_representation->updatePose(moveit_robot_state_->getGlobalLinkTransform(link_name));
    // Iterate over spheres and compute sdf
    for (unsigned int j = 0; j < sphere_representation->getCollisionSpheres().size(); j++) {
      Eigen::Vector3f center = sphere_representation->getSphereCenters()[j].cast<float>();
      double radius = sphere_representation->getCollisionSpheres()[j].radius_;

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

void RobotModel::updateRobotState(const Eigen::Affine3d &robot_pose, const std::vector<double> joint_angles)
{
  if (joint_angles.size() != num_joints_) {
    ROS_ERROR_STREAM("[RobotModel::publishRobotState] joint_angles size (" << joint_angles.size() << ") doesn't match number of joints in robot model (" << num_joints_ << ")");
    return;
  }
  robot_pose_ = robot_pose;
  joint_angles_ = joint_angles;

  updateMoveitRobotState(*moveit_robot_state_, robot_pose);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names_;
  joint_state_msg.position = joint_angles;
  moveit::core::jointStateToRobotState(joint_state_msg, *moveit_robot_state_);
}

void RobotModel::publishRobotState(const Eigen::Affine3d &robot_pose, const std::vector<double> joint_angles)
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

void RobotModel::publishCollisionSpheres(const Eigen::Affine3d &robot_pose, const std::vector<double> joint_angles)
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

  unsigned int id = 0;

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

std::vector<unsigned int> RobotModel::getBodyIndices(const std::vector<std::string> &bodies)
{
  std::vector<unsigned int> indices;
  for (const std::string& body: bodies) {
    unsigned int idx = model_ptr_->GetBodyId(body.c_str());
    if (idx == std::numeric_limits<unsigned int>::max()) {
      ROS_ERROR_STREAM("Body '" << body << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
  return indices;
}

std::vector<unsigned int> RobotModel::getJointIndices(const std::vector<std::string> &joints)
{
  std::vector<unsigned int> indices;
  for (const std::string& joint: joints) {
    ptrdiff_t idx = std::find(joint_names_.begin(), joint_names_.end(), joint) - joint_names_.begin();
    if (idx >= joint_names_.size()) {
      ROS_ERROR_STREAM("Joint '" << joint << "' wasn't found in model.");
    }
    indices.push_back(idx);
  }
  return indices;
}

std::string RobotModel::getParentJointName(const std::string &link_name)
{
  boost::shared_ptr<const urdf::Link> link = urdf_model_.getLink(link_name);
  if (!link) {
    ROS_ERROR_STREAM("Couldn't find link '" << link_name << "'.");
    return "NOT FOUND";
  }
  return link->parent_joint->name;
}

bool RobotModel::getJointLimits(const std::string &joint_name, double &lower_limit, double &upper_limit)
{
  urdf::JointConstSharedPtr joint = urdf_model_.getJoint(joint_name);
  if (!joint) {
    ROS_ERROR_STREAM("Couldn't find joint '" << joint_name << "'.");
    return false;
  }
  upper_limit = joint->limits->upper;
  lower_limit = joint->limits->lower;
  return true;
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

const std::vector<collision_detection::CollisionSphere>& RobotModel::getCollisionSpheres(std::string link_name)
{
  unsigned int idx = body_decomposition_index_map_.at(link_name);
  return body_decompositions_[idx]->getCollisionSpheres();
}

std::vector<unsigned int> RobotModel::getChainToJointIndexMapping()
{
  return getJointIndices(ad_chain_->getActuatedJointNames());
}

std::vector<int> RobotModel::getParamToChainIndexMapping(const std::vector<std::string> &free_joints)
{
  return ad_chain_->getJointQIndices(free_joints);
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

robot_model::RobotModelPtr RobotModel::getMoveitRobotModel() const
{
  return moveit_robot_model_;
}

robot_state::RobotState RobotModel::getMoveitRobotState() const
{
  return *moveit_robot_state_;
}

}
