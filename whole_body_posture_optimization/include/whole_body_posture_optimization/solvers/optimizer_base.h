#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H

#include <nlopt.hpp>
#include <ceres/ceres.h>
#include <rclcpp/rclcpp.hpp>
#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

namespace whole_body_posture_optimization {

class OptimizerBase {
public:
  OptimizerBase();
  virtual ~OptimizerBase() = default;
  void setMinFunction(std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> cost_function);
  void loadSettingsFromNlopt(const nlopt::opt& opt);

  virtual void loadParametersFromNamespace(rclcpp::Node& node, const std::string& prefix) = 0;

  virtual nlopt::result optimize(std::vector<double>& parameters, double& minf) = 0;
  virtual std::string solverName() const = 0;
protected:

  std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> cost_function_;
  unsigned int dimensions_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  unsigned int evaluation_counter_;
};

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_BASE_H
