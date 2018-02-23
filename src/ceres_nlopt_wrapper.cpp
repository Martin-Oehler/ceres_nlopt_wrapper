#include <ceres_nlopt_wrapper/ceres_nlopt_wrapper.h>

namespace ceres_nlopt_wrapper {

CeresCostFunctionWrapper::CeresCostFunctionWrapper(ceres::CostFunction *cost_function, int verbosity_level, bool use_numeric_diff)
  : cost_function_(cost_function), verbosity_level_(verbosity_level), evaluation_counter_(0), use_numeric_diff_(use_numeric_diff), nan_check_(false), inf_check_(false)
{
  if (use_numeric_diff_) {
    // Numeric Diff Wrapper
    ceres::NumericDiffOptions options;
    ceres::DynamicNumericDiffCostFunction<ceres::CostFunction, ceres::CENTRAL>* numeric_cost_function =
        new ceres::DynamicNumericDiffCostFunction<ceres::CostFunction, ceres::CENTRAL>(
          cost_function, ceres::DO_NOT_TAKE_OWNERSHIP, options);
    const std::vector<ceres::int32>& parameter_block_sizes = cost_function_->parameter_block_sizes();
    const int num_parameter_blocks = parameter_block_sizes.size();
    for (int i = 0; i < num_parameter_blocks; ++i) {
      numeric_cost_function->AddParameterBlock(parameter_block_sizes[i]);
    }
    numeric_cost_function->SetNumResiduals(cost_function_->num_residuals());
    numeric_cost_function_ = static_cast<ceres::CostFunction*>(numeric_cost_function);
  }
}

CeresCostFunctionWrapper::~CeresCostFunctionWrapper() {
  delete cost_function_;
  if (use_numeric_diff_) {
    delete numeric_cost_function_;
  }
}

double CeresCostFunctionWrapper::operator()(const std::vector<double> &x, std::vector<double> &gradient) {
  if (!use_numeric_diff_) {
    return evaluateCostFunction(cost_function_, x, gradient);
  } else {
    return evaluateCostFunction(numeric_cost_function_, x, gradient);
  }
}

double CeresCostFunctionWrapper::evaluateCostFunction(const ceres::CostFunction *cost_function, const std::vector<double> &x, std::vector<double> &gradient)
{
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

    if (!cost_function->Evaluate(parameters_ptr, &cost, jacobian_ptr)) {
      ROS_ERROR_STREAM("Failed to evaluate cost function");
      return std::numeric_limits<double>::max();
    }
  } else {
    if (!cost_function->Evaluate(parameters_ptr, &cost, NULL)) {
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
  if (nan_check_ && std::isnan(cost)) {
    ROS_ERROR_STREAM("Cost '" << getName() << "'is nan. (x = " << vecToString(x) << ")");
  }
  if (inf_check_ && std::isinf(cost)) {
    ROS_WARN_STREAM("Cost '" << getName() << "'is inf. (x = " << vecToString(x) << ")");
  }

  return cost;
}

double CeresCostFunctionWrapper::wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
  return (*static_cast<CeresCostFunctionWrapper*>(data))(x, grad);
}

bool CeresCostFunctionWrapper::checkGradient(const std::vector<double>& x)
{
  ceres::NumericDiffOptions numeric_diff_options;
  ceres::GradientChecker gradient_checker(cost_function_, nullptr, numeric_diff_options);
  ceres::GradientChecker::ProbeResults probe_results;
  double const* x_ptr = &x[0];
  double const *const *parameters_ptr = &x_ptr;
  if (!gradient_checker.Probe(parameters_ptr, 1e-5, &probe_results)) {
    ROS_ERROR_STREAM("Gradient check of '" << getName() << "' failed. Max relative error: " << probe_results.maximum_relative_error);
    ROS_ERROR_STREAM("Gradient: " << probe_results.jacobians[0]);
    ROS_ERROR_STREAM("Numeric gradient: " << probe_results.numeric_jacobians[0]);
    return false;
  }
  return true;
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

void CeresCostFunctionWrapper::enableNanCheck(bool enable)
{
  nan_check_ = enable;
}

void CeresCostFunctionWrapper::enableInfCheck(bool enable)
{
  inf_check_ = enable;
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
