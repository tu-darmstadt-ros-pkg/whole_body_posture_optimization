#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__FLIPPER_OPTIMIZATION_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__FLIPPER_OPTIMIZATION_H

#include <whole_body_posture_optimization/flipper/convex_hull_area_error.h>
#include <whole_body_posture_optimization/ceres_wrapper.h>
#include <whole_body_posture_optimization/arm/stability_error.h>
#include <sdf_contact_estimation/visualisation.h>

namespace whole_body_posture_optimization {

struct FlipperOptimizationResult {
  std::vector<double> flipper_angles;
  std::vector<double> joint_state;
  sdf_contact_estimation::ContactEstimationResult contact_result;
};

class FlipperOptimization {
public:
  FlipperOptimization(ros::NodeHandle& nh, const std::shared_ptr<RobotModel>& robot_model, const sdf_contact_estimation::RobotShapeGeneratorBasePtr &robot_shape_gen,
                      const std::shared_ptr<sdf_contact_estimation::SDFContactEstimation>& contact_estimation);
  FlipperOptimizationResult optimize(const Eigen::Affine3d& robot_pose, const Eigen::Affine3d& base_to_com, const std::vector<double> &init_position,
                               const sdf_contact_estimation::ContactEstimationResult &init_contact);
  sdf_contact_estimation::ContactEstimationResult computeContactEstimation(const std::vector<double>& joint_angles,const Eigen::Affine3d& pose, const Eigen::Affine3d &base_to_com,
                                                                           const sdf_contact_estimation::ContactEstimationResult& last_result);
  sdf_contact_estimation::ContactEstimationResult computeContactEstimation(const std::vector<double>& joint_angles,const Eigen::Affine3d& pose, const Eigen::Affine3d &base_to_com);
private:
  std::vector<double> jointVectorToParamVector(const std::vector<double>& joint_vec);
  bool loadParameters(const ros::NodeHandle &nh);
  bool loadLimits();
  void createConvexHullMarkerArray(const std::vector<Eigen::Vector3d>& points, const std::vector<double>& stabilities, visualization_msgs::MarkerArray &marker_array);

  std::shared_ptr<RobotModel> robot_model_;
  std::vector<unsigned int> param_idx_to_jnt_idx_;
  std::shared_ptr<sdf_contact_estimation::SDFContactEstimation> contact_estimation_;
  sdf_contact_estimation::RobotShapeGeneratorBasePtr robot_shape_gen_;

  std::vector<std::string> free_joints_;
  std::vector<double> upper_bounds_;
  std::vector<double> lower_bounds_;

  ros::Publisher start_state_pub_;
  ros::Publisher result_state_pub_;
  ros::Publisher contact_points_pub_;
  ros::Publisher result_com_pub_;
};

}

#endif
