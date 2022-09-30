#ifndef CERES_NLOPT_WRAPPER_NLOPT_PARAMETER_SERVER_H
#define CERES_NLOPT_WRAPPER_NLOPT_PARAMETER_SERVER_H

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <nlopt.hpp>

namespace ceres_nlopt_wrapper {

class NloptParameterServer
{
public:
  explicit NloptParameterServer(const ros::NodeHandle& nh, unsigned int parameter_count);
  nlopt::opt& getOptimizer();
private:
  void loadParametersFromNamespace(const ros::NodeHandle& nh);
  nlopt::opt createOptimizer(const std::string& algorithm);
  void algorithmCallback(std::string algorithm);
  void xTolAbsCallback(double x_tol_abs);
  void xTolRelCallback(double x_tol_rel);
  void fTolAbsCallback(double f_tol_abs);
  void fTolRelCallback(double f_tol_rel);
  void maxEvalCallback(int maxeval);
  void maxTimeCallback(double maxtime);
  void stopvalCallback(double stopval);

  ros::NodeHandle nh_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> reconfigure_;
  nlopt::opt opt_;
  unsigned int parameter_count_;
  bool initialized_;
};

}


#endif  // CERES_NLOPT_WRAPPER_NLOPT_PARAMETER_SERVER_H
