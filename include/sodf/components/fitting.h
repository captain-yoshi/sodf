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
  Eigen::Vector3d axis;              // Insertion axis (unit vector, e.g. -X)
  uint32_t rotational_symmetry = 1;  // Number of allowed rotations around the axis
                                     // e.g. 1 = unique orientation, 2 = 0°,180°, 4 = 0°,90°,180°,270°

  double approach_distance = 0.0;  // Distance to start insertion [m], along -axis
};

struct FitConstraintComponent
{
  FlatMap<std::string, FitConstraint> fitting_map;
};

}  // namespace components
}  // namespace sodf

#endif
