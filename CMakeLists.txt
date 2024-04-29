cmake_minimum_required(VERSION 3.1.3)
project(sodf)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2_eigen
)

set(quarticRootsFlocke_INCLUDE_DIRS ext/quarticRootsFlocke/src)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
    ${quarticRootsFlocke_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2_eigen
  DEPENDS
    EIGEN3
)

# add quarticRootsFlocke as an internal library
# https://github.com/ebertolazzi/quarticRootsFlocke
add_subdirectory(ext/quarticRootsFlocke quarticRootsFlocke)

include_directories(
  include
  ${quarticRootsFlocke_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/object.cpp

  src/elements/container.cpp

  src/geometry/volume.cpp
  src/geometry/transform.cpp
)

add_subdirectory(test)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
