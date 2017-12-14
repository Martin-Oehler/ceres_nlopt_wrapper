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

}
