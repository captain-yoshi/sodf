#ifndef SODF_COMPONENTS_CONSTRAINT_H_
#define SODF_COMPONENTS_CONSTRAINT_H_

#include <Eigen/Geometry>

#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Constraint
{
  geometry::Transform transform;
};

struct DualConstraint
{
  std::string parent1;
  std::string child1;
  std::string parent2;
  std::string child2;
};

struct Transforms
{
  std::vector<geometry::Transform> transforms;
};

}  // namespace components
}  // namespace sodf

#endif
