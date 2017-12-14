#ifndef CERES_NLOPT_WRAPPER__CERES_NLOPT_WRAPPER
#define CERES_NLOPT_WRAPPER__CERES_NLOPT_WRAPPER

#include <ceres/ceres.h>
#include <nlopt.hpp>
#include <ros/ros.h>

#include <ceres_nlopt_wrapper/utils.h>

namespace ceres_nlopt_wrapper {

class CeresCostFunctionWrapper {
public:
  /**
   * @brief CeresCostFunctionWrapper Wraps a ceres cost function for use in nlopt.
   * Takes ownership of _cost_function
   * @param _cost_function ceres::CostFunction to wrap
   */
  CeresCostFunctionWrapper(ceres::CostFunction* _cost_function, int verbosity_level=0);
  ~CeresCostFunctionWrapper();

  double operator()(const std::vector<double> &x, std::vector<double> &gradient);

  /// \brief Wraps an object for use as a function pointer in nlopt
  /// Example: CeresCostFunctionWrapper support_area_cost(..);
  /// opt.set_min_objective(ceres_nlopt_wrapper::CeresCostFunctionWrapper::wrap, &support_area_cost);
  /// \param x  Parameter vector
  /// \param grad Vector of size x.size() or empty. Output of the gradient
  /// \param data Pointer to CeresCostFunctionWrapper object
  /// \return Result of cost function evaluation
  static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data);

  void setVerbosity(int level);
  unsigned int getEvaluations();
private:
  ceres::CostFunction* cost_function_;
  int verbosity_level_;
  unsigned int evaluation_counter_;
};

}

#endif
