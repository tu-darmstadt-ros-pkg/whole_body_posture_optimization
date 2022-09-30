#include <whole_body_posture_optimization/optimization_parameters_base.h>

namespace whole_body_posture_optimization {

OptimizationParametersBase::OptimizationParametersBase()
: verbose(false)
{}

bool OptimizationParametersBase::loadParametersFromNamespace(const ros::NodeHandle& nh)
{
  reconfigure_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh);
  bool success = loadParametersFromNamespaceImpl(nh);
  success &= loadBaseParametersFromNamespace(nh);
  reconfigure_->publishServicesTopics();
  return success;
}
void OptimizationParametersBase::registerParameter(const ros::NodeHandle& nh, const std::string& name, double& value,
                                                   double default_value, const std::string& description, double min,
                                                   double max)
{
  nh.param(name, value, default_value);
  config_[name] = &value;
  reconfigure_->registerVariable<double>(name, &value, description, min, max);
}
std::string OptimizationParametersBase::toString() const
{
  std::stringstream ss;
  for (auto const& parameter: config_) {
    ss << parameter.first << " = " << *parameter.second << std::endl;
  }
  return ss.str();
}
bool OptimizationParametersBase::loadBaseParametersFromNamespace(const ros::NodeHandle& nh)
{
  nh.param("verbose", verbose, false);
  reconfigure_->registerVariable<bool>("verbose", &verbose, std::string("Verbosity"));
  return true;
}

}