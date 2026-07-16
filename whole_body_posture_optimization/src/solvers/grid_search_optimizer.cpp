#include <whole_body_posture_optimization/solvers/grid_search_optimizer.h>
#include "whole_body_posture_optimization/util/common.h"
#include "whole_body_posture_optimization/util/parameter_utils.h"

namespace whole_body_posture_optimization {

const std::string GridSearchOptimizer::NAME = "grid_search";

GridSearchOptimizer::GridSearchOptimizer()
 : resolution_(0.1)
{}

void GridSearchOptimizer::loadParametersFromNamespace(rclcpp::Node& node, const std::string& prefix)
{
  resolution_ = param_utils::param(node, prefix + "search_resolution", 0.1);
}

nlopt::result GridSearchOptimizer::optimize(std::vector<double>& parameters, double& minf)
{
  parameters = lower_bounds_;
  minf = std::numeric_limits<double>::max();
  gridSearch(lower_bounds_, parameters, minf, 0);
  return nlopt::FAILURE;
}

void GridSearchOptimizer::gridSearch(const std::vector<double>& search_parameters, std::vector<double>& min_parameters, double& minf, size_t dim)
{
  std::vector<double> parameters_var = search_parameters;
  for (double val = lower_bounds_[dim]; val < upper_bounds_[dim]; val += resolution_) {
    parameters_var[dim] = val;
    if (dim+1 < dimensions_) {
      gridSearch(parameters_var, min_parameters, minf, dim+1);
    } else {
      std::vector<double> gradient;
      double f = (*cost_function_)(parameters_var, gradient);
      if (!std::isnan(f) && !std::isinf(f) && f < minf) {
        minf = f;
        min_parameters = parameters_var;
      }
    }
  }
}
std::string GridSearchOptimizer::solverName() const
{
  return GridSearchOptimizer::NAME;
}

}
