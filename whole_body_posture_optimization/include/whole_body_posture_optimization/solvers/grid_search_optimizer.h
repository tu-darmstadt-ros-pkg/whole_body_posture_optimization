#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_GRID_SEARCH_OPTIMIZER_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_GRID_SEARCH_OPTIMIZER_H

#include <whole_body_posture_optimization/solvers/optimizer_base.h>

namespace whole_body_posture_optimization {

class GridSearchOptimizer : public OptimizerBase
{
public:
  GridSearchOptimizer();
  void loadParametersFromNamespace(const ros::NodeHandle& nh) override;
  nlopt::result optimize(std::vector<double>& parameters, double& minf) override;
  void gridSearch(const std::vector<double>& search_parameters, std::vector<double>& min_parameters, double& minf, size_t dim);
private:
  double resolution_;
};

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_GRID_SEARCH_OPTIMIZER_H
