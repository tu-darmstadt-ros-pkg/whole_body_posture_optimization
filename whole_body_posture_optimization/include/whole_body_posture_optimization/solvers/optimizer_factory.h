#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H

#include <whole_body_posture_optimization/solvers/optimizer_base.h>
#include <whole_body_posture_optimization/solvers/grid_search_optimizer.h>

namespace whole_body_posture_optimization {

std::shared_ptr<OptimizerBase> createSolverByName(const std::string& solver_name) {
  if (solver_name == GridSearchOptimizer::NAME) {
    return std::make_shared<GridSearchOptimizer>();
  }
  ROS_ERROR_STREAM("Unknown solver '" << solver_name << "'");
  return {};
}

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H
