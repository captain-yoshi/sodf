#ifndef SODF_COMPONENTS_MESH_H_
#define SODF_COMPONENTS_MESH_H_

#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Mesh
{
  std::string url;  // ressource location
  Eigen::Matrix3d inertia;
};

struct MeshCollection
{
  std::vector<std::pair<std::string, Mesh>> mesh_map;
};

}  // namespace components
}  // namespace sodf

#endif
