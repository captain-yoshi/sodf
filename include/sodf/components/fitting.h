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
  Eigen::Vector3d axis_insertion;    // Insertion axis (unit vector)
  uint32_t rotational_symmetry = 1;  // Number of unique orientations wrt. the axis
                                     // 0 = infinite, 1 = unique, 2 = 0 and 180 degrees, etc...
  double approach_distance = 0.0;    // Pre-insertion offset [m] wrt. the axis
};

struct FitConstraintComponent
{
  FlatMap<std::string, FitConstraint> map;
};

}  // namespace components
}  // namespace sodf

#endif
