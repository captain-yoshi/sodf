#ifndef SODF_COMPONENTS_DOMAIN_SHAPE_H_
#define SODF_COMPONENTS_DOMAIN_SHAPE_H_

#include <sodf/ecs.h>
#include <sodf/physics/fluid_domain_shape.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

// Sequence of fluid domain shapes representing the fillable regions in this container.
// Shapes are stacked strictly in vertical order (opposite to gravity), with no arbitrary orientation supported.
// Each shape is positioned relative to the previous using its own reference point (e.g., base, tip),
// and may be inverted at construction (such as for a tip-referenced spherical cap).
// The stacking and all fill height/volume calculations are only valid for this vertical configuration.
using DomainShapes = std::vector<physics::DomainShapePtr>;

struct DomainShapeComponent
{
  FlatMap<std::string, DomainShapes> domain_shape_map;
};

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_DOMAIN_SHAPE_H_
