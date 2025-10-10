#ifndef SODF_COMPONENTS_LINK_H_
#define SODF_COMPONENTS_LINK_H_

#include <sodf/components/data_type.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

struct InertialProperties
{
  double mass = 0.0;
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
  // Default to a tiny positive-definite inertia (epsilon*I) instead of 0 or 1:
  // - keeps the matrix invertible for solvers
  // - clearly acts as a placeholder (negligible effect compared to real inertia)
  // - avoids hiding bugs where a real inertia tensor was never assigned
  Eigen::Matrix3d inertia_tensor = 1e-6 * Eigen::Matrix3d::Identity();
};

struct Link
{
  geometry::Shape bbox;  // bouding box, box shape
  geometry::Shape collision;
  geometry::Shape visual;

  InertialProperties dynamics;
};

struct LinkComponent
{
  ElementMap<std::string, Link> elements;
};

}  // namespace components
}  // namespace sodf

#endif
