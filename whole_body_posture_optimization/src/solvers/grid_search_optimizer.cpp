#include <whole_body_posture_optimization/solvers/grid_search_optimizer.h>

namespace whole_body_posture_optimization {

GridSearchOptimizer::GridSearchOptimizer()
 : resolution_(0.1)
{}

void GridSearchOptimizer::loadParametersFromNamespace(const ros::NodeHandle& nh)
{
  nh.param("search_resolution", resolution_, 0.1);
}

nlopt::result GridSearchOptimizer::optimize(std::vector<double>& parameters, double& minf)
{
  parameters = lower_bounds_;
  minf = std::numeric_limits<double>::max();
  gridSearch(parameters, minf, 0);
  return nlopt::FAILURE;
}

void GridSearchOptimizer::gridSearch(std::vector<double>& parameters, double& minf, size_t dim)
{
  std::vector<double> parameters_temp = parameters;
  for (double val = lower_bounds_[dim]; val < upper_bounds_[dim]; val += resolution_) {
    parameters_temp[dim] = val;
    if (dim+1 < dimensions_) {
      gridSearch(parameters, minf, dim+1);
    } else {
      double f = evaluateCostFunction(cost_function_, parameters);
      if (f < minf) {
        minf = f;
        parameters = parameters_temp;
      }
    }
  }
}

}
