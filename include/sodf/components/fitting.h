#ifndef SODF_COMPONENTS_FITTING_H_
#define SODF_COMPONENTS_FITTING_H_

#include <sodf/ecs.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct FitConstraint
{
  // Insertion or approach axis (unit vector); specifies which direction the tool/button should align with.
  Eigen::Vector3d axis_insertion;
  Eigen::Vector3d axis_reference;    //
  uint32_t rotational_symmetry = 1;  // Number of unique orientations wrt. the axis
                                     // 0 = infinite, 1 = unique, 2 = 0 and 180 degrees, etc...
  double approach_distance = 0.0;    // Pre-insertion offset [m] wrt. the axis
};

struct FitConstraintComponent
{
  ElementMap<std::string, FitConstraint> elements;
};

}  // namespace components
}  // namespace sodf

#endif
