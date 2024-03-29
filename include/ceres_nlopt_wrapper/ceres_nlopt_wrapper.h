#ifndef CERES_NLOPT_WRAPPER_CERES_NLOPT_WRAPPER
#define CERES_NLOPT_WRAPPER_CERES_NLOPT_WRAPPER

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <nlopt.hpp>
#include <ros/ros.h>
#include <sstream>

#include <ceres_nlopt_wrapper/utils.h>
#include <ceres_nlopt_wrapper/optimization_history.h>

namespace ceres_nlopt_wrapper {

class CeresCostFunctionWrapper {
public:
  /**
   * @brief CeresCostFunctionWrapper Wraps a ceres cost function for use in nlopt.
   * Takes ownership of _cost_function
   * @param _cost_function ceres::CostFunction to wrap
   */
  CeresCostFunctionWrapper(ceres::CostFunction* _cost_function, int verbosity_level=0, HistoryWriter* history=nullptr, bool use_numeric_diff=false,
                           ceres::Ownership ownership = ceres::TAKE_OWNERSHIP);
  ~CeresCostFunctionWrapper();

  double operator()(const std::vector<double> &x, std::vector<double> &gradient);

  /// \brief Wraps an object for use as a function pointer in nlopt
  /// Example: CeresCostFunctionWrapper support_area_cost(..);
  /// opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &support_area_cost);
  /// \param x Parameter vector
  /// \param grad Vector of size x.size() or empty. Output of the gradient
  /// \param data Pointer to CeresCostFunctionWrapper object
  /// \return Result of cost function evaluation
  static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data);

  bool checkGradient(const std::vector<double> &x, double relative_precision=1e-5, double absolute_error=1e-5);

  void setVerbosity(int level);
  unsigned int getEvaluations();
  void resetEvaluationCounter();
  void setName(const std::string& name);
  std::string getName();
  void enableNanCheck(bool enable);
  void enableInfCheck(bool enable);
private:
  double evaluateCostFunction(const ceres::CostFunction* cost_function, const std::vector<double> &x, std::vector<double> &gradient);

  ceres::CostFunction* cost_function_;
  ceres::Ownership ownership_;
  ceres::CostFunction* numeric_cost_function_;
  int verbosity_level_;
  HistoryWriter* history_;
  unsigned int evaluation_counter_;
  bool use_numeric_diff_;
  bool nan_check_;
  bool inf_check_;

  std::string name_; // Optional name for function
};

struct ConstraintViolation {
  ConstraintViolation(const std::shared_ptr<CeresCostFunctionWrapper>& _constraint, double _value)
    : constraint(_constraint), value(_value) {}

  std::shared_ptr<CeresCostFunctionWrapper> constraint;
  double value;

  std::string toString() const {
    if (constraint) {
      std::stringstream ss;
      ss << "Constraint '" << constraint->getName() << "' violated. Value: " << value;
      return ss.str();
    } else {
      return "Constraint is null.";
    }
  }
};

bool checkConstraints(const std::vector<double>& parameters, const std::vector<std::shared_ptr<CeresCostFunctionWrapper>>& constraints, double tolerance);
bool checkConstraints(const std::vector<double>& parameters, const std::vector<std::shared_ptr<CeresCostFunctionWrapper>>& constraints, double tolerance,
                      std::vector<ConstraintViolation>& constraint_violations);


}

#endif
