#ifndef CERES_NLOPT_WRAPPER_UTILS_H
#define CERES_NLOPT_WRAPPER_UTILS_H

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

void setParametersFromServer(const ros::NodeHandle& nh, nlopt::opt& opt);

template<typename T>
bool is_equal(const std::vector<T>& v1, const std::vector<T>& v2) {
  if (v1.size() != v2.size()) {
    return false;
  }
  for (size_t i = 0; i < v1.size(); ++i) {
    if (v1[i] != v2[i]) {
      return false;
    }
  }
  return true;
}

}

#endif
