#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include <nlopt.hpp>
#include <chrono>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/visualisation.h>
#include <whole_body_posture_optimization/common.h>
#include <whole_body_posture_optimization/ceres_wrapper.h>
#include <whole_body_posture_optimization/arm/stability_error.h>
#include <whole_body_posture_optimization/arm/contact_constraint.h>

namespace whole_body_posture_optimization {

struct ArmOptimizationResult {
  std::vector<double> joint_angles; // optimal joint angles
  Eigen::Vector3d center_of_mass; // optimal center of mass relative to base frame
};

class ArmOptimization {
public:
  ArmOptimization(ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);
  ArmOptimizationResult optimize(const std::vector<double>& initial_angles, const std::vector<Eigen::Vector3d>& contact_points, const Eigen::Affine3d &robot_pose);
private:
  bool loadParameters(const ros::NodeHandle &nh);
  bool loadLimits();
  std::vector<double> jointVectorToParamVector(const std::vector<double> &joint_vec);
  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_jnt_idx_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;


  std::vector<std::string> free_joints_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;

  ros::Publisher result_com_pub_;
  ros::Publisher convex_hull_pub_;
};

}

#endif
