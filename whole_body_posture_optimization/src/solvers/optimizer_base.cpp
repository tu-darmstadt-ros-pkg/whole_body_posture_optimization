#include <whole_body_posture_optimization/solvers/optimizer_base.h>

namespace whole_body_posture_optimization {

OptimizerBase::OptimizerBase()
 : evaluation_counter_(0)
{}

void OptimizerBase::setMinFunction(ceres::CostFunction *cost_function)
{
  cost_function_ = cost_function;
}

void OptimizerBase::loadSettingsFromNlopt(const nlopt::opt &opt)
{
  dimensions_ = opt.get_dimension();
  lower_bounds_ = opt.get_lower_bounds();
  upper_bounds_ = opt.get_upper_bounds();
}

double OptimizerBase::evaluateCostFunction(const ceres::CostFunction *cost_function, const std::vector<double> &x, std::vector<double> &gradient)
{
  evaluation_counter_++;
  double const* x_ptr = &x[0];
  double const *const *parameters_ptr = &x_ptr;
  double cost;

//  if (verbosity_level_ > 0) {
//    ROS_INFO_STREAM(evaluation_counter_ << ": x = " << vecToString(x));
//  }

  double* gradient_ptr = &gradient[0];
  double **jacobian_ptr = nullptr;

  if (!gradient.empty()) {
    jacobian_ptr = &gradient_ptr;
  }

  if (!cost_function->Evaluate(parameters_ptr, &cost, jacobian_ptr)) {
    ROS_ERROR_STREAM("Failed to evaluate cost function");
    return std::numeric_limits<double>::max();
  }

//  if (verbosity_level_ > 0) {
//    ROS_INFO_STREAM(" *** f(x) = " << cost);
//    if (verbosity_level_ > 1) {
//      if (!gradient.empty()) {
//        ROS_INFO_STREAM(" *** f'(x) = " << vecToString(gradient));
//      }
//    }
//  }
  // check cost for nan/inf
//  if (std::isnan(cost)) {
//    ROS_ERROR_STREAM("Cost '" << getName() << "'is nan. (x = " << vecToString(x) << ")");
//  }
//  if (std::isinf(cost)) {
//    ROS_WARN_STREAM("Cost '" << getName() << "'is inf. (x = " << vecToString(x) << ")");
//  }

  return cost;
}

double OptimizerBase::evaluateCostFunction(const ceres::CostFunction *cost_function, const std::vector<double> &x)
{
  std::vector<double> gradient;
  return evaluateCostFunction(cost_function, x, gradient);
}

}