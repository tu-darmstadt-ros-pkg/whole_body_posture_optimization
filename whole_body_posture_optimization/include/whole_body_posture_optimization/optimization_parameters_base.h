#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace whole_body_posture_optimization {

class OptimizationParametersBase {
public:
  bool loadParametersFromNamespace(const ros::NodeHandle& nh);
  std::string toString() const;
protected:

  virtual bool loadParametersFromNamespaceImpl(const ros::NodeHandle& nh) = 0;
  void registerParameter(const ros::NodeHandle& nh, const std::string& name, double& value, double default_value, const std::string& description, double min, double max);
private:
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> reconfigure_;
  std::map<std::string, double*> config_;
};

}


#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_OPTIMIZATION_PARAMETERS_BASE_H
