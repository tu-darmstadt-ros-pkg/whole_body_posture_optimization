#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__ARM_OPTIMIZATION_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <nlopt.hpp>
#include <chrono>

#include <whole_body_posture_optimization/ceres_wrapper.h>
#include <whole_body_posture_optimization/stability_error.h>

namespace whole_body_posture_optimization {

struct ArmOptResult {
  std::vector<double> joint_angles; // optimal joint angles
  Eigen::Vector3d center_of_mass; // optimal center of mass relative to base frame
};

class ArmOptimization {
public:
  ArmOptimization(const ros::NodeHandle &nh, const std::shared_ptr<RobotModel>& robot_model);
  bool loadParameters(const ros::NodeHandle &nh);
  ArmOptResult optimize(const std::vector<double>& initial_pose, const std::vector<Eigen::Vector3d>& contact_points);
private:
  std::shared_ptr<RobotModel> robot_model_;

  std::vector<std::string> free_joints_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
};

}

#endif
