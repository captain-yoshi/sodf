cmake_minimum_required(VERSION 3.1.3)
project(sodf)

set(CMAKE_CXX_STANDARD 17)

find_package(orocos_kdl)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  eigen_conversions
)

set(quarticRootsFlocke_INCLUDE_DIRS ext/quarticRootsFlocke/src)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    quarticRootsFlocke
  INCLUDE_DIRS
    include
    ${quarticRootsFlocke_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    eigen_conversions
  DEPENDS
    EIGEN3
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${quarticRootsFlocke_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/object.cpp

  src/elements/container.cpp
  src/elements/insertion.cpp

  src/geometry/volume.cpp
  src/geometry/transform.cpp
)

# add quarticRootsFlocke as an internal library
# https://github.com/ebertolazzi/quarticRootsFlocke
add_library(quarticRootsFlocke
  ext/quarticRootsFlocke/src/PolynomialRoots-1-Quadratic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-2-Cubic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-3-Quartic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-Jenkins-Traub.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-Utils.cc
)

add_subdirectory(test)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
