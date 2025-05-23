#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <vector>
#include <sodf/ecs.h>

#include <Eigen/Geometry>

#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

struct Container
{
  double volume = 0.0;  // SI m^3
  std::string content_type;
  std::string material_type;

  Eigen::Vector3d axis;  // Unit vector toward container bottom (gravity direction, e.g., -X)

  // Shapes are stacked upward from the bottom along the container -axis
  std::vector<geometry::BaseShapePtr> shapes;
};

struct ContainerComponent
{
  FlatMap<std::string, Container> container_map;
};

}  // namespace components
}  // namespace sodf

#endif
