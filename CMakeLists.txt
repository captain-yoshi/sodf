cmake_minimum_required(VERSION 3.16.3)
project(sodf)

option(SODF_WITH_CURL "Enable HTTP support in SODF resolver (sodf:// via http/https roots)" ON)

set(CMAKE_CXX_STANDARD 17)

set(SODF_LIBRARY ${PROJECT_NAME})

find_package(ament_cmake QUIET)
find_package(assimp REQUIRED)

include(cmake/assimp.cmake)

# All vendored include directories (for BUILD_INTERFACE)
set(quarticRootsFlocke_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ext/quarticRootsFlocke/src)
set(better_enums_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ext/better-enums)
set(ginseng_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ext/ginseng)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "------------------------------------------")
    message(STATUS "SODF is being built using AMENT.")
    message(STATUS "------------------------------------------")
    include(cmake/ament_build.cmake)

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    add_definitions( -DUSING_ROS )
    message(STATUS "------------------------------------------")
    message(STATUS "SODF is being built using CATKIN.")
    message(STATUS "------------------------------------------")
    #include(cmake/catkin_build.cmake)
    set(catkin_FOUND TRUE)

    find_package(catkin REQUIRED COMPONENTS rostest)

    catkin_package(
      INCLUDE_DIRS
        include
        ${quarticRootsFlocke_INCLUDE_DIRS}
        ${better_enums_INCLUDE_DIRS}
        ${ginseng_INCLUDE_DIRS}
      LIBRARIES
        sodf
      DEPENDS
        Eigen3
        assimp
    )

    include_directories(
      ${catkin_INCLUDE_DIRS}
    )

    install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(DIRECTORY ext/better-enums/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/better-enums)
    install(DIRECTORY ext/quarticRootsFlocke/src/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/quarticRootsFlocke)
    install(DIRECTORY ext/ginseng/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


else()
    message(STATUS "------------------------------------------")
    message(STATUS "SODF is being built with conan.")
    message(STATUS "------------------------------------------")
    include(cmake/conan_build.cmake)
endif()

find_package(Eigen3 REQUIRED)
find_library(MUPARSER_LIB muparser REQUIRED)
find_path(MUPARSER_INCLUDE_DIR muParser.h REQUIRED)

if(SODF_WITH_CURL)
  find_package(CURL QUIET)
  if(CURL_FOUND)
    message(STATUS "SODF: HTTP support enabled (CURL::libcurl)")
  else()
    message(WARNING "SODF_WITH_CURL=ON but cURL not found → disabling HTTP support")
    set(SODF_WITH_CURL OFF)
  endif()
endif()

add_library(${PROJECT_NAME}
  ext/tinyxml2/tinyxml2.cpp

  src/assembly/constraint.cpp
  src/assembly/constraint_selector.cpp
  src/assembly/selector_context_db.cpp
  src/geometry/shape.cpp
  src/geometry/frame.cpp
  src/physics/domain_shape.cpp
  src/physics/domain_shape_base.cpp
  src/physics/domain_shape_conversion.cpp
  src/physics/fluid_domain_shape.cpp
  src/systems/fsm.cpp
  src/systems/liquid_level_joint.cpp
  src/systems/origin_constraints.cpp
  src/systems/origin_ls_solver.cpp
  src/systems/scene_graph.cpp
  src/systems/scene_graph_diff.cpp
  src/uri_resolver.cpp
  src/xml/component_parser.cpp
  src/xml/element_parser.cpp
  src/xml/entity_parser.cpp
  src/xml/expression_parser.cpp
  src/xml/for_loop_parser.cpp
  src/xml/patch_operations.cpp
  src/xml/selector.cpp
)

# Internal libraries
add_library(quarticRootsFlocke
  ext/quarticRootsFlocke/src/PolynomialRoots-1-Quadratic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-2-Cubic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-3-Quartic.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-Jenkins-Traub.cc
  ext/quarticRootsFlocke/src/PolynomialRoots-Utils.cc
)
add_library(better_enums INTERFACE)
add_library(ginseng INTERFACE)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${MUPARSER_LIB}
    Eigen3::Eigen
    quarticRootsFlocke
    assimp::assimp
)

# Public define so headers can see it
if(SODF_WITH_CURL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC SODF_WITH_CURL)
  target_link_libraries(${PROJECT_NAME} PRIVATE CURL::libcurl)
endif()

# Export include directories for dependent projects
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${quarticRootsFlocke_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${better_enums_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${ginseng_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${MUPARSER_INCLUDE_DIR}>
    # $<INSTALL_INTERFACE:include/sodf>
    $<INSTALL_INTERFACE:include>
)

# Add tests
include(CTest)
add_subdirectory(test)

# ---- Installation ----

# Install all headers (main + vendored) under include/sodf/
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/sodf)
install(DIRECTORY ext/better-enums/ DESTINATION include/sodf/better-enums)
install(DIRECTORY ext/ginseng/ DESTINATION include/sodf/ginseng)
install(DIRECTORY ext/quarticRootsFlocke/src/ DESTINATION include/sodf/quarticRootsFlocke)

# Install targets and export
install(TARGETS
  ${PROJECT_NAME}
  quarticRootsFlocke
  better_enums
  ginseng
  EXPORT sodf-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/sodf
)

# Export CMake config files (for downstream projects)
install(EXPORT sodf-targets
  FILE sodfTargets.cmake
  NAMESPACE sodf::
  DESTINATION lib/cmake/sodf
)

if(catkin_FOUND)
    install(TARGETS sodf
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # For DLLs on Windows
    )
 endif()
