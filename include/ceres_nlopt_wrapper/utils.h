#ifndef CERES_NLOPT_WRAPPER__UTILS_H
#define CERES_NLOPT_WRAPPER__UTILS_H

#include <ros/ros.h>

#include <nlopt.hpp>
#include <sstream>
#include <memory>
#include <algorithm>

namespace ceres_nlopt_wrapper {

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

std::string resultToString(const nlopt::result& result);

nlopt::algorithm stringToAlgorithm(const std::string& name);

}

#endif
