#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H

#include <nlopt.hpp>
#include <ceres/ceres.h>
#include <ros/ros.h>

namespace whole_body_posture_optimization {

class OptimizerBase {
public:
  OptimizerBase();
  void setMinFunction(ceres::CostFunction* cost_function);
  void loadSettingsFromNlopt(const nlopt::opt& opt);

  virtual void loadParametersFromNamespace(const ros::NodeHandle& nh) = 0;

  virtual nlopt::result optimize(std::vector<double>& parameters, double& minf) = 0;

protected:
  double evaluateCostFunction(const ceres::CostFunction* cost_function, const std::vector<double> &x, std::vector<double> &gradient);
  double evaluateCostFunction(const ceres::CostFunction* cost_function, const std::vector<double> &x);

  ceres::CostFunction* cost_function_;
  unsigned int dimensions_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  unsigned int evaluation_counter_;
};

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H
