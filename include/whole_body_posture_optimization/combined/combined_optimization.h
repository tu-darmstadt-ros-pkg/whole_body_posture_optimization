#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_COMBINED_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_COMBINED_OPTIMIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <moveit/planning_scene/planning_scene.h>

#include <nlopt.hpp>

#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/visualisation.h>
#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/robot_model.h>
#include <whole_body_posture_optimization/combined/combined_stability_cost.h>

namespace whole_body_posture_optimization {

struct CombinedOptimizationResult {
  std::vector<double> joint_angles; // optimal joint angles
  Eigen::Vector3d center_of_mass; // optimal center of mass relative to base frame
  sdf_contact_estimation::ContactEstimationResult contact_result;
};

class CombinedOptimization {
public:
  CombinedOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);

  CombinedOptimizationResult optimize(const std::vector<double>& initial_state, const Eigen::Affine3d &robot_pose, const std::vector<double>& previous_state,
                                      const sdf_contact_estimation::ContactEstimationResult &init_contact);
  sdf_contact_estimation::ContactEstimationResult computeContactEstimation(const std::vector<double>& joint_angles,const Eigen::Affine3d& pose, const Eigen::Affine3d &base_to_com);
private:
  bool loadParameters(const ros::NodeHandle &nh);
  double getJointPadding(std::string joint_name, CollisionType collision_type);
  double getDefaultJointPadding(CollisionType collision_type);
  bool loadLimits(const std::vector<std::string>& joints);
  std::vector<double> jointVectorToParamVector(const std::vector<double> &joint_vec);

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;

  std::vector<std::size_t> param_idx_to_jnt_idx_;

  // Parameters
  std::vector<std::string> free_joints_;
  std::vector<std::string> free_flipper_joints_;
  std::vector<std::string> free_arm_joints_;

  bool enabled_;
  std::string algorithm_;
  CombinedStabilityCostParameters stability_cost_parameters_;

  // Collision
  double environment_collision_default_padding_;
  std::map<std::string, double> environment_collision_padding_map_;
  double self_collision_padding_;

  std::vector<std::string> static_link_names_;
  double self_collision_tsdf_padding_;
  double self_collision_tsdf_truncation_distance_;

  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;

  std::shared_ptr<voxblox::TsdfMap> robot_tsdf_;

  collision_detection::AllowedCollisionMatrix acm_;

  // Publishers
  ros::Publisher result_com_pub_;
  ros::Publisher convex_hull_pub_;
  ros::Publisher robot_tsdf_pub_;
};

}

#endif
