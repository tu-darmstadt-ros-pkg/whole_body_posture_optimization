#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION__CERES_WRAPPER_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION__CERES_WRAPPER_H

#include <ceres/ceres.h>
#include <nlopt.hpp>
#include <ros/ros.h>

inline std::string resultToString(nlopt::result result) {
  switch (result) {
    case nlopt::FAILURE: return "FAILURE";
    case nlopt::INVALID_ARGS: return "INVALID_ARGS";
    case nlopt::OUT_OF_MEMORY: return "OUT_OF_MEMORY";
    case nlopt::ROUNDOFF_LIMITED: return "ROUNDOFF_LIMITED";
    case nlopt::FORCED_STOP: return "FORCED_STOP";
    case nlopt::SUCCESS: return "SUCCESS";
    case nlopt::STOPVAL_REACHED: return "STOPCAL_REACHED";
    case nlopt::FTOL_REACHED: return "FTOL_REACHED";
    case nlopt::XTOL_REACHED: return "XTOL_REACHED";
    case nlopt::MAXEVAL_REACHED: return "MAXEVAL_REACHED";
    case nlopt::MAXTIME_REACHED: return "MAXTIME_REACHED";
    default: return "UNKNOWN";
  }
}

template<typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

class CeresCostFunctionWrapper {
public:
  /**
   * @brief CeresCostFunctionWrapper Wraps a ceres cost function for use in nlopt.
   * Takes ownership of _cost_function
   * @param _cost_function ceres::CostFunction to wrap
   */
  CeresCostFunctionWrapper(ceres::CostFunction* _cost_function, int verbosity_level=0)
    : cost_function_(_cost_function), verbosity_level_(verbosity_level)
  {}

  ~CeresCostFunctionWrapper() {
    delete cost_function_;
  }

  double operator()(const std::vector<double> &x, std::vector<double> &gradient) const {
    double const* x_ptr = &x[0];
    double const *const *parameters_ptr = &x_ptr;
    double* cost = new double;

    if (!gradient.empty()) {
      double* gradient_ptr = &gradient[0];
      double **jacobian_ptr = &gradient_ptr;

      if (!cost_function_->Evaluate(parameters_ptr, cost, jacobian_ptr)) {
        ROS_ERROR_STREAM("Failed to evaluate cost function");
        return 0.0;
      }
    } else {
      if (!cost_function_->Evaluate(parameters_ptr, cost, NULL)) {
        ROS_ERROR_STREAM("Failed to evaluate cost function");
        return 0.0;
      }
    }

    if (verbosity_level_ > 0) {
      ROS_INFO_STREAM("x = " << vecToString(x));
      if (verbosity_level_ > 1) {
        std::stringstream ss;
        ss << " *** f(x) = " << *cost;
        if (!gradient.empty()) {
          ss << "; f'(x) = " << vecToString(gradient);
        }
        ROS_INFO_STREAM(ss.str());
      }
    }

    return *cost;
  }

  static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
      return (*static_cast<CeresCostFunctionWrapper*>(data))(x, grad);
  }
private:
  ceres::CostFunction* cost_function_;
  int verbosity_level_;
};

#endif
