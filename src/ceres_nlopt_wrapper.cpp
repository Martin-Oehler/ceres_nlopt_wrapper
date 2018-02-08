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
  double cost;

  if (verbosity_level_ > 0) {
    ROS_INFO_STREAM(evaluation_counter_ << ": x = " << vecToString(x));
  }

  if (!gradient.empty()) {
    double* gradient_ptr = &gradient[0];
    double **jacobian_ptr = &gradient_ptr;

    if (!cost_function_->Evaluate(parameters_ptr, &cost, jacobian_ptr)) {
      ROS_ERROR_STREAM("Failed to evaluate cost function");
      return std::numeric_limits<double>::max();
    }
  } else {
    if (!cost_function_->Evaluate(parameters_ptr, &cost, NULL)) {
      ROS_ERROR_STREAM("Failed to evaluate cost function");
      return std::numeric_limits<double>::max();
    }
  }

  if (verbosity_level_ > 0) {
    ROS_INFO_STREAM(" *** f(x) = " << cost);
    if (verbosity_level_ > 1) {
      if (!gradient.empty()) {
        ROS_INFO_STREAM(" *** f'(x) = " << vecToString(gradient));
      }
    }
  }
  // check cost for nan/inf
  if (std::isnan(cost)) {
    ROS_ERROR_STREAM("Cost '" << getName() << "'is nan.");
    ROS_ERROR_STREAM("x = " << vecToString(x));
  }
  if (std::isinf(cost)) {
    ROS_WARN_STREAM("Cost '" << getName() << "'is inf.");
  }
  return cost;
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

void CeresCostFunctionWrapper::setName(const std::string &name)
{
  name_ = name;
}

std::string CeresCostFunctionWrapper::getName()
{
  return name_;
}

bool checkConstraints(const std::vector<double>& parameters, const std::vector<std::shared_ptr<CeresCostFunctionWrapper>>& constraints, double tolerance)
{
  std::vector<ConstraintViolation> constraint_violations;
  return checkConstraints(parameters, constraints, tolerance, constraint_violations);
}

bool checkConstraints(const std::vector<double> &parameters, const std::vector<std::shared_ptr<CeresCostFunctionWrapper> > &constraints, double tolerance,
                      std::vector<ConstraintViolation> &constraint_violations)
{
  constraint_violations.clear();
  for (const std::shared_ptr<CeresCostFunctionWrapper>& c: constraints) {
    if (!c) {
      ROS_ERROR_STREAM("Constraint is null.");
      continue;
    }
    std::vector<double> gradient(parameters.size());
    double cost = c->operator ()(parameters, gradient);
    if (cost > tolerance) {
      constraint_violations.push_back(ConstraintViolation(c, cost));
    }
  }
  return constraint_violations.empty();
}

}
