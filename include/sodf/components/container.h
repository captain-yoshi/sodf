#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <vector>
#include <sodf/ecs.h>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct Container
{
  double volume = 0.0;  // SI m^3
  std::string content_type;
  std::string material_id;

  Eigen::Vector3d axis_bottom;  // Unit normal vector of the bottom surface, pointing into (toward) the container bottom.

  std::string domain_shape_id;  // geometric + physics shape
};

struct ContainerComponent
{
  FlatMap<std::string, Container> map;
};

}  // namespace components
}  // namespace sodf

#endif
