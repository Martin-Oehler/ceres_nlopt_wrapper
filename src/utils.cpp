#include <ceres_nlopt_wrapper/utils.h>

namespace ceres_nlopt_wrapper {

std::string resultToString(const nlopt::result &result) {
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

nlopt::algorithm stringToAlgorithm(const std::string &name) {
  std::string s = name;
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "GN_DIRECT") {
    return nlopt::GN_DIRECT;
  }
  if (s == "GN_DIRECT_L") {
    return nlopt::GN_DIRECT_L;
  }
  if (s == "GN_DIRECT_L_RAND") {
    return nlopt::GN_DIRECT_L_RAND;
  }
  if (s == "GN_DIRECT_L_NOSCAL") {
    return nlopt::GN_DIRECT_L_NOSCAL;
  }
  if (s == "GN_ORIG_DIRECT") {
    return nlopt::GN_ORIG_DIRECT;
  }
  if (s == "GN_ORIG_DIRECT_L") {
    return nlopt::GN_ORIG_DIRECT_L;
  }
  if (s == "GD_STOGO") {
    return nlopt::GD_STOGO;
  }
  if (s == "GD_STOGO_RAND") {
    return nlopt::GD_STOGO_RAND;
  }
  if (s == "LD_LBFGS_NOCEDAL") {
    return nlopt::LD_LBFGS_NOCEDAL;
  }
  if (s == "LD_LBFGS") {
    return nlopt::LD_LBFGS;
  }
  if (s == "LN_PRAXIS") {
    return nlopt::LN_PRAXIS;
  }
  if (s == "LD_VAR1") {
    return nlopt::LD_VAR1;
  }
  if (s == "LD_VAR2") {
    return nlopt::LD_VAR2;
  }
  if (s == "LD_TNEWTON") {
    return nlopt::LD_TNEWTON;
  }
  if (s == "LD_TNEWTON_RESTART") {
    return nlopt::LD_TNEWTON_RESTART;
  }
  if (s == "LD_TNEWTON_PRECOND") {
    return nlopt::LD_TNEWTON_PRECOND;
  }
  if (s == "LD_TNEWTON_PRECOND_RESTART") {
    return nlopt::LD_TNEWTON_PRECOND_RESTART;
  }
  if (s == "GN_CRS2_LM") {
    return nlopt::GN_CRS2_LM;
  }
  if (s == "GN_MLSL") {
    return nlopt::GN_MLSL;
  }
  if (s == "GD_MLSL") {
    return nlopt::GD_MLSL;
  }
  if (s == "GN_MLSL_LDS") {
    return nlopt::GN_MLSL_LDS;
  }
  if (s == "GD_MLSL_LDS") {
    return nlopt::GD_MLSL_LDS;
  }
  if (s == "LD_MMA") {
    return nlopt::LD_MMA;
  }
  if (s == "LN_COBYLA") {
    return nlopt::LN_COBYLA;
  }
  if (s == "LN_NEWUOA") {
    return nlopt::LN_NEWUOA;
  }
  if (s == "LN_NEWUOA_BOUND") {
    return nlopt::LN_NEWUOA_BOUND;
  }
  if (s == "LN_NELDERMEAD") {
    return nlopt::LN_NELDERMEAD;
  }
  if (s == "LN_SBPLX") {
    return nlopt::LN_SBPLX;
  }
  if (s == "LN_AUGLAG") {
    return nlopt::LN_AUGLAG;
  }
  if (s == "LD_AUGLAG") {
    return nlopt::LD_AUGLAG;
  }
  if (s == "LN_AUGLAG_EQ") {
    return nlopt::LN_AUGLAG_EQ;
  }
  if (s == "LD_AUGLAG_EQ") {
    return nlopt::LD_AUGLAG_EQ;
  }
  if (s == "LN_BOBYQA") {
    return nlopt::LN_BOBYQA;
  }
  if (s == "GN_ISRES") {
    return nlopt::GN_ISRES;
  }
  if (s == "AUGLAG") {
    return nlopt::AUGLAG;
  }
  if (s == "AUGLAG_EQ") {
    return nlopt::AUGLAG_EQ;
  }
  if (s == "G_MLSL") {
    return nlopt::G_MLSL;
  }
  if (s == "G_MLSL_LDS") {
    return nlopt::G_MLSL_LDS;
  }
  if (s == "") {
    return nlopt::GD_MLSL;
  }
  if (s == "LD_SLSQP") {
    return nlopt::LD_SLSQP;
  }
  if (s == "LD_CCSAQ") {
    return nlopt::LD_CCSAQ;
  }
  if (s == "GN_ESCH") {
    return nlopt::GN_ESCH;
  }

  ROS_ERROR_STREAM("Unknown algorithm name '" << s << "'");
  return nlopt::NUM_ALGORITHMS;
}

void setParametersFromServer(const ros::NodeHandle &nh, nlopt::opt &opt) {
  double xtol_abs, xtol_rel;
  double ftol_abs, ftol_rel;
  int maxeval;
  double maxtime;

  std::stringstream ss;
  ss << "Loaded:" << std::endl;

  if (nh.getParam("xtol_abs", xtol_abs)) {
    ss << " xtol_abs = " << xtol_abs << std::endl;
    opt.set_xtol_abs(xtol_abs);
  }
  if (nh.getParam("xtol_rel", xtol_rel)) {
    ss << " xtol_rel = " << xtol_rel << std::endl;
    opt.set_xtol_rel(xtol_rel);
  }
  if (nh.getParam("ftol_abs", ftol_abs)) {
    ss << " ftol_abs = " << ftol_abs << std::endl;
    opt.set_ftol_abs(ftol_abs);
  }
  if (nh.getParam("ftol_rel", ftol_rel)) {
    ss << " ftol_rel = " << ftol_rel << std::endl;
    opt.set_ftol_rel(ftol_rel);
  }
  if (nh.getParam("maxeval", maxeval)) {
    ss << " maxeval = " << maxeval << std::endl;
    opt.set_maxeval(maxeval);
  }
  if (nh.getParam("maxtime", maxtime)) {
    ss << " maxtime = " << maxtime << std::endl;
    opt.set_maxtime(maxtime);
  }
  ROS_DEBUG_STREAM(ss.str());
}

}
