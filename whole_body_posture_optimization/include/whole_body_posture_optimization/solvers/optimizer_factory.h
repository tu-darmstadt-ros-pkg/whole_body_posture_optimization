#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H

#include <whole_body_posture_optimization/solvers/optimizer_base.h>
#include <whole_body_posture_optimization/solvers/grid_search_optimizer.h>

namespace whole_body_posture_optimization {

inline std::shared_ptr<OptimizerBase> createSolverByName(const std::string& solver_name) {
  if (solver_name == GridSearchOptimizer::NAME) {
    return std::make_shared<GridSearchOptimizer>();
  }
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("whole_body_posture_optimization"), "Unknown solver '" << solver_name << "'");
  return {};
}

}

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZER_FACTORY_H
