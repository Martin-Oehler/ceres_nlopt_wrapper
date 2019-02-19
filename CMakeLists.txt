cmake_minimum_required(VERSION 2.8.3)
project(ceres_nlopt_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nlopt
  roscpp
  ceres_catkin
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ceres_nlopt_wrapper
  CATKIN_DEPENDS nlopt roscpp
  DEPENDS Eigen3
)

#find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/ceres_nlopt_wrapper.h
  include/${PROJECT_NAME}/utils.h
  src/ceres_nlopt_wrapper.cpp
  src/utils.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${CERES_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ceres_nlopt_wrapper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
