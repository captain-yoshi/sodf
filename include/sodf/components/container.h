#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <vector>
#include <sodf/ecs.h>

#include <Eigen/Geometry>

#include <sodf/physics/domain_shape.h>
#include <sodf/components/shape.h>

namespace sodf {
namespace components {

struct Container
{
  double volume = 0.0;  // SI m^3
  std::string content_type;
  std::string material_type;

  Eigen::Vector3d axis_bottom;  // Unit normal vector of the bottom surface, pointing into (toward) the container bottom.

  std::string shape_ref;  // geometric
  std::string
      domain_shape_ref;  // Reference ID of the associated shape definition (e.g., fluid, solid, or generic domain shape).
                         // Allows reuse of shared geometric descriptions between containers of different content types.
};

struct ContainerComponent
{
  FlatMap<std::string, Container> container_map;
};

}  // namespace components
}  // namespace sodf

#endif
