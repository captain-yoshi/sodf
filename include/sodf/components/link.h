#ifndef SODF_COMPONENTS_LINK_H_
#define SODF_COMPONENTS_LINK_H_

#include <sodf/ecs.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

struct InertialProperties
{
  double mass = 0.0;
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_tensor = 1e-6 * Eigen::Matrix3d::Identity();  // 3×3 symmetric matrix
};

struct Link
{
  geometry::Shape visual;
  geometry::Shape collision;

  InertialProperties dynamics;
};

struct LinkComponent
{
  ElementMap<std::string, Link> elements;
};

}  // namespace components
}  // namespace sodf

#endif
