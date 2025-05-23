#ifndef SODF_COMPONENTS_LINK_H_
#define SODF_COMPONENTS_LINK_H_

#include <Eigen/Geometry>
#include <sodf/ecs.h>

namespace sodf {
namespace components {

struct GeometryData
{
  std::string mesh_path;  // STL, DAE, OBJ, etc.
  Eigen::Vector3d scale = { 1.0, 1.0, 1.0 };
};

struct InertialProperties
{
  double mass = 0.0;
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Zero();  // 3×3 symmetric matrix
};

struct Link
{
  GeometryData visual;
  GeometryData collision;

  InertialProperties dynamics;
};

struct LinkComponent
{
  FlatMap<std::string, Link> link_map;  //
};

}  // namespace components
}  // namespace sodf

#endif
