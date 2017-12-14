#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

namespace ceres_nlopt_wrapper {

CeresCostFunctionWrapper::CeresCostFunctionWrapper(ceres::CostFunction *_cost_function, int verbosity_level)
  : cost_function_(_cost_function), verbosity_level_(verbosity_level), evaluation_counter_(0)
{}

CeresCostFunctionWrapper::~CeresCostFunctionWrapper() {
  delete cost_function_;
}

double CeresCostFunctionWrapper::operator()(const std::vector<double> &x, std::vector<double> &gradient) {
  evaluation_counter_++;
  double const* x_ptr = &x[0];
  double const *const *parameters_ptr = &x_ptr;
  double* cost = new double;

  if (!gradient.empty()) {
    double* gradient_ptr = &gradient[0];
    double **jacobian_ptr = &gradient_ptr;

    if (!cost_function_->Evaluate(parameters_ptr, cost, jacobian_ptr)) {
      ROS_ERROR_STREAM("Failed to evaluate cost function");
      return std::numeric_limits<double>::max();
    }
  } else {
    if (!cost_function_->Evaluate(parameters_ptr, cost, NULL)) {
      ROS_ERROR_STREAM("Failed to evaluate cost function");
      return std::numeric_limits<double>::max();
    }
  }

  if (verbosity_level_ > 0) {
    ROS_INFO_STREAM(evaluation_counter_ << ": x = " << vecToString(x));
    ROS_INFO_STREAM(" *** f(x) = " << *cost);
    if (verbosity_level_ > 1) {
      if (!gradient.empty()) {
        ROS_INFO_STREAM(" *** f'(x) = " << vecToString(gradient));
      }
    }
  }
  return *cost;
}

double CeresCostFunctionWrapper::wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
  return (*static_cast<CeresCostFunctionWrapper*>(data))(x, grad);
}

void CeresCostFunctionWrapper::setVerbosity(int level) {
  verbosity_level_ = level;
}

unsigned int CeresCostFunctionWrapper::getEvaluations() {
  return evaluation_counter_;
}

}
