#include <ceres_nlopt_wrapper/nlopt_parameter_server.h>

namespace ceres_nlopt_wrapper {

static std::map<std::string, std::string> algorithm_map = {
  {"GN_DIRECT","GN_DIRECT"},
  {"GN_DIRECT_L","GN_DIRECT_L"},
  {"GN_DIRECT_L_RAND","GN_DIRECT_L_RAND"},
  {"GN_DIRECT_NOSCAL","GN_DIRECT_NOSCAL"},
  {"GN_DIRECT_L_NOSCAL","GN_DIRECT_L_NOSCAL"},
  {"GN_DIRECT_L_RAND_NOSCAL","GN_DIRECT_L_RAND_NOSCAL"},
  {"GN_ORIG_DIRECT","GN_ORIG_DIRECT"},
  {"GN_ORIG_DIRECT_L","GN_ORIG_DIRECT_L"},
  {"GD_STOGO","GD_STOGO"},
  {"GD_STOGO_RAND","GD_STOGO_RAND"},
  {"LD_LBFGS_NOCEDAL","LD_LBFGS_NOCEDAL"},
  {"LD_LBFGS","LD_LBFGS"},
  {"LN_PRAXIS","LN_PRAXIS"},
  {"LD_VAR1","LD_VAR1"},
  {"LD_VAR2","LD_VAR2"},
  {"LD_TNEWTON","LD_TNEWTON"},
  {"LD_TNEWTON_RESTART","LD_TNEWTON_RESTART"},
  {"LD_TNEWTON_PRECOND","LD_TNEWTON_PRECOND"},
  {"LD_TNEWTON_PRECOND_RESTART","LD_TNEWTON_PRECOND_RESTART"},
  {"GN_CRS2_LM","GN_CRS2_LM"},
  {"GN_MLSL","GN_MLSL"},
  {"GD_MLSL","GD_MLSL"},
  {"GN_MLSL_LDS","GN_MLSL_LDS"},
  {"GD_MLSL_LDS","GD_MLSL_LDS"},
  {"LD_MMA","LD_MMA"},
  {"LN_NEWUOA","LN_NEWUOA"},
  {"LN_NEWUOA_BOUND","LN_NEWUOA_BOUND"},
  {"LN_NELDERMEAD","LN_NELDERMEAD"},
  {"LN_SBPLX","LN_SBPLX"},
  {"LN_AUGLAG","LN_AUGLAG"},
  {"LD_AUGLAG","LD_AUGLAG"},
  {"LN_AUGLAG_EQ","LN_AUGLAG_EQ"},
  {"LD_AUGLAG_EQ","LD_AUGLAG_EQ"},
  {"LN_BOBYQA","LN_BOBYQA"},
  {"GN_ISRES","GN_ISRES"},
  {"AUGLAG","AUGLAG"},
  {"AUGLAG_EQ","AUGLAG_EQ"},
  {"G_MLSL","G_MLSL"},
  {"G_MLSL_LDS","G_MLSL_LDS"},
  {"LD_SLSQP","LD_SLSQP"},
  {"LD_CCSAQ","LD_CCSAQ"},
  {"GN_ESCH","GN_ESCH"},
  {"GN_AGS","GN_AGS"},
};

NloptParameterServer::NloptParameterServer(const ros::NodeHandle& nh, unsigned int parameter_count)
: nh_(nh), parameter_count_(parameter_count), initialized_(false) {
  loadParametersFromNamespace(nh);
}
void NloptParameterServer::loadParametersFromNamespace(const ros::NodeHandle& nh)
{
  reconfigure_->registerEnumVariable<std::string>("algorithm", "GN_DIRECT_L" , boost::bind(&NloptParameterServer::algorithmCallback, this, _1), "Algorithm", algorithm_map);
  reconfigure_->registerVariable<double>("xtol_abs", 0.0, boost::bind(&NloptParameterServer::xTolAbsCallback, this, _1), "xtol_abs", 0, 10);
  reconfigure_->registerVariable<double>("xtol_rel", 0.0, boost::bind(&NloptParameterServer::xTolRelCallback, this, _1), "xtol_rel", 0, 10);
  reconfigure_->registerVariable<double>("ftol_abs", 0.0, boost::bind(&NloptParameterServer::fTolAbsCallback, this, _1), "ftol_abs", 0, 10);
  reconfigure_->registerVariable<double>("ftol_rel", 0.0, boost::bind(&NloptParameterServer::fTolRelCallback, this, _1), "ftol_rel", 0, 10);
  reconfigure_->registerVariable<int>("maxeval", 0, boost::bind(&NloptParameterServer::maxEvalCallback, this, _1), "maxeval", 0, std::numeric_limits<int>::max());
  reconfigure_->registerVariable<double>("maxtime", 0.0, boost::bind(&NloptParameterServer::maxTimeCallback, this, _1), "maxtime", 0, std::numeric_limits<double>::max());
  reconfigure_->registerVariable<double>("stopval", 0.0, boost::bind(&NloptParameterServer::stopvalCallback, this, _1), "stopval", 0, std::numeric_limits<double>::max());
  reconfigure_->publishServicesTopics();
}
void NloptParameterServer::algorithmCallback(std::string algorithm)
{
  opt_ = createOptimizer(algorithm);
}
void NloptParameterServer::xTolAbsCallback(double x_tol_abs)
{
  opt_.set_xtol_abs(x_tol_abs);
}
void NloptParameterServer::xTolRelCallback(double x_tol_rel)
{
  opt_.set_xtol_rel(x_tol_rel);
}
void NloptParameterServer::fTolAbsCallback(double f_tol_abs)
{
  opt_.set_ftol_abs(f_tol_abs);
}
void NloptParameterServer::fTolRelCallback(double f_tol_rel)
{
  opt_.set_ftol_rel(f_tol_rel);
}
void NloptParameterServer::maxEvalCallback(int maxeval)
{
  opt_.set_maxeval(maxeval);
}
void NloptParameterServer::maxTimeCallback(double maxtime)
{
  opt_.set_maxtime(maxtime);
}
void NloptParameterServer::stopvalCallback(double stopval)
{
  opt_.set_stopval(stopval);
}
nlopt::opt NloptParameterServer::createOptimizer(const std::string& algorithm)
{
  nlopt::opt new_opt(algorithm.c_str(), parameter_count_);
  if (initialized_) {
    new_opt.set_xtol_abs(opt_.get_xtol_abs());
    new_opt.set_xtol_rel(opt_.get_xtol_rel());
    new_opt.set_ftol_abs(opt_.get_ftol_abs());
    new_opt.set_ftol_rel(opt_.get_ftol_rel());
    new_opt.set_maxeval(opt_.get_maxeval());
    new_opt.set_maxtime(opt_.get_maxtime());
    new_opt.set_stopval(opt_.get_stopval());
  }
  initialized_ = true;
  return new_opt;
}
nlopt::opt& NloptParameterServer::getOptimizer()
{
  return opt_;
}

}
