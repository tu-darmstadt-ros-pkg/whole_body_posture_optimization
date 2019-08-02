#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <moveit/planning_scene/planning_scene.h>

#include <nlopt.hpp>
#include <chrono>

#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/visualisation.h>
#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/arm/stability_error.h>
#include <whole_body_posture_optimization/arm/sdf_contact_constraint.h>
#include <whole_body_posture_optimization/arm/self_contact_constraint.h>

namespace whole_body_posture_optimization {

struct ArmOptimizationResult {
  std::vector<double> joint_angles; // optimal joint angles
  Eigen::Vector3d center_of_mass; // optimal center of mass relative to base frame
  double stability;
};

class ArmOptimization {
public:
  ArmOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);
  ArmOptimizationResult optimize(const std::vector<double>& initial_angles, const std::vector<Eigen::Vector3d>& contact_points, const Eigen::Affine3d &robot_pose,
                                 const std::vector<double> &previous_joint_angles);
  bool getEnabled() const;
  void setEnabled(bool enabled);

private:
  bool loadParameters(const ros::NodeHandle &nh);
  double getJointPadding(std::string joint_name, CollisionType collision_type);
  double getDefaultJointPadding(CollisionType collision_type);
  bool loadLimits();
  std::vector<double> jointVectorToParamVector(const std::vector<double> &joint_vec);

  ros::NodeHandle nh_;
  nlopt::opt opt_;

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_jnt_idx_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;

  std::vector<std::string> free_joints_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;

  // Optimization parameters
  bool enabled_;
  std::string algorithm_;
  StabilityErrorParameters stability_error_params_;

  std::shared_ptr<voxblox::TsdfMap> partial_robot_tsdf_;
  std::vector<std::string> semi_static_link_names_;
  std::vector<std::string> static_link_names_;

  // Collision parameters
  double environment_collision_default_padding_;
  std::map<std::string, double> environment_collision_padding_map_;
  double self_collision_default_padding_;
  std::map<std::string, double> self_collision_padding_map_;
  double self_collision_tsdf_padding_;

  double self_collision_tsdf_truncation_distance_;
  double self_collision_tsdf_voxel_size_;
  collision_detection::AllowedCollisionMatrix acm_;

  ros::Publisher result_com_pub_;
  ros::Publisher convex_hull_pub_;
  ros::Publisher robot_tsdf_pub_;

};

}

#endif
