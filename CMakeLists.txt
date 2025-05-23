cmake_minimum_required(VERSION 3.1.3)
project(sodf)

set(CMAKE_CXX_STANDARD 17)

find_package(orocos_kdl)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  eigen_conversions
  kdl_conversions
)

set(SODF_LIBRARY ${PROJECT_NAME})


set(quarticRootsFlocke_INCLUDE_DIRS ext/quarticRootsFlocke/src)
set(better_enums_INCLUDE_DIRS ext/better-enums)
set(ginseng_INCLUDE_DIRS ext/ginseng)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    quarticRootsFlocke
  INCLUDE_DIRS
    include
    ${quarticRootsFlocke_INCLUDE_DIRS}
    ${better_enums_INCLUDE_DIRS}
    ${ginseng_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    eigen_conversions
    kdl_conversions
  DEPENDS
    EIGEN3
    orocos_kdl
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${quarticRootsFlocke_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${better_enums_INCLUDE_DIRS}
  ${ginseng_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ext/tinyxml2/tinyxml2.cpp

  src/behavior/state.cpp
  src/conversion.cpp
  src/geometry/alignment.cpp
  src/geometry/volume.cpp
  src/graph/directed_graph.cpp
  src/xml_parser.cpp
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

# add better-enums as an internal library
# https://github.com/aantron/better-enums/commit/520d8ee39037c9c94aa6e708a4fd6c0fa313ae80
add_library(better_enums INTERFACE)

# add ginseng as an internal library
# https://github.com/apples/ginseng/commit/8839ef98d0e31c64aec8f3445445367d690b742c
add_library(ginseng INTERFACE)

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
