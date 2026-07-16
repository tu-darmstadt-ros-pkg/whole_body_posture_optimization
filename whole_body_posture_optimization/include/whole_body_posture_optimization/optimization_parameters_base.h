#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

namespace whole_body_posture_optimization {

/**
 * @brief Base class for optimization parameter sets.
 *
 * Parameters are declared on the given (sub-)node under the node's sub-namespace
 * prefix and can be updated at runtime via the standard ROS 2 parameter
 * interface (replaces the ROS 1 ddynamic_reconfigure implementation).
 */
class OptimizationParametersBase {
public:
  OptimizationParametersBase();
  virtual ~OptimizationParametersBase() = default;
  bool loadParametersFromNamespace(const rclcpp::Node::SharedPtr& node);
  std::string toString() const;

  bool verbose;
protected:
  virtual bool loadParametersFromNamespaceImpl(const rclcpp::Node::SharedPtr& node) = 0;
  void registerParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, double& value, double default_value, const std::string& description, double min, double max);
private:
  bool loadBaseParametersFromNamespace(const rclcpp::Node::SharedPtr& node);
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  std::string prefix_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::map<std::string, double*> config_;
  bool* verbose_ptr_ = nullptr;
  std::string verbose_param_name_;
};

}


#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H
