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
  Eigen::Vector3d axis_reference;  // Unit vector in the bottom plane

  std::string domain_shape_id;  // geometric + physics shape

  // Fully-qualified joint ID used to lookup and update the liquid surface height for this container.
  // Empty if the container does not support a dynamic liquid level.
  std::string liquid_level_joint_id;
};

struct ContainerComponent
{
  ElementMap<std::string, Container> elements;
};

}  // namespace components
}  // namespace sodf

#endif
