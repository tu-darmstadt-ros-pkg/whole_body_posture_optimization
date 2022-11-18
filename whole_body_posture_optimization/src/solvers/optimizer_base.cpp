#include <whole_body_posture_optimization/solvers/optimizer_base.h>

namespace whole_body_posture_optimization {

OptimizerBase::OptimizerBase()
 : evaluation_counter_(0)
{}

void OptimizerBase::setMinFunction(std::shared_ptr<ceres_nlopt_wrapper::CeresCostFunctionWrapper> cost_function)
{
  cost_function_ = cost_function;
}

void OptimizerBase::loadSettingsFromNlopt(const nlopt::opt &opt)
{
  dimensions_ = opt.get_dimension();
  lower_bounds_ = opt.get_lower_bounds();
  upper_bounds_ = opt.get_upper_bounds();
}

}