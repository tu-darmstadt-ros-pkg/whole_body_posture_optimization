#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_ARM_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_ARM_OPTIMIZATION_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <nlopt.hpp>

#include <moveit/collision_detection/collision_matrix.h>
#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>
#include <ceres_nlopt_wrapper/nlopt_parameter_server.h>

#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <whole_body_posture_optimization/arm/stability_cost.h>

namespace whole_body_posture_optimization {

struct ArmOptimizationResult {
  std::vector<double> joint_angles; // optimal joint angles
  Eigen::Vector3d center_of_mass; // optimal center of mass relative to base frame
  double stability;
  int iterations;
  double optimization_time;
  double final_cost;
};

class ArmOptimization {
public:
  ArmOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model, const sdf_contact_estimation::SdfModelPtr& sdf_model);
  ArmOptimizationResult optimize(const std::vector<double>& initial_angles, const std::vector<Eigen::Vector3d>& contact_hull_points, const Eigen::Isometry3d &robot_pose,
                                 const std::vector<double> &previous_joint_angles);
  bool getEnabled() const;
  void setEnabled(bool enabled);

private:
  bool loadParameters(const ros::NodeHandle &nh);
  void loadAdditionalACMEntries(const ros::NodeHandle &nh);
  double getJointPadding(std::string joint_name, CollisionType collision_type);
  double getDefaultJointPadding(CollisionType collision_type);
  bool loadLimits();
  std::vector<double> jointVectorToParamVector(const std::vector<double> &joint_vec);

  ros::NodeHandle nh_;

  std::shared_ptr<ceres_nlopt_wrapper::NloptParameterServer> opt_server_;
  std::shared_ptr<ceres_nlopt_wrapper::NloptParameterServer> local_opt_server_;

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<std::size_t> param_idx_to_jnt_idx_;
  sdf_contact_estimation::SdfModelPtr sdf_model_;

  std::vector<std::string> free_joints_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;

  // Optimization parameters
  bool enabled_;
  std::string algorithm_;
  StabilityCostParameters stability_cost_params_;

  std::shared_ptr<voxblox::TsdfMap> partial_robot_tsdf_;
  std::vector<std::string> semi_static_link_names_;
  std::vector<std::string> static_link_names_;

  std::vector<std::string> collision_link_names_;
  std::vector<std::string> self_collision_tsdf_link_names_;

  // Collision parameters
  std::vector<std::string> self_collision_tsdf_allowed_links_;
  double environment_collision_default_padding_;
  std::map<std::string, double> environment_collision_padding_map_;
  double self_collision_default_padding_;
  std::map<std::string, double> self_collision_padding_map_;
  double self_collision_tsdf_default_padding_;
  std::map<std::string, double> self_collision_tsdf_padding_map_;

  double self_collision_tsdf_truncation_distance_;
  double self_collision_tsdf_voxel_size_;
  collision_detection::AllowedCollisionMatrix acm_;

  ros::Publisher result_com_pub_;
  ros::Publisher support_polygon_pub_;
  ros::Publisher robot_tsdf_pub_;
};

}

#endif
