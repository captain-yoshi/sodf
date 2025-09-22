#ifndef SODF_COMPONENTS_TRANSFORM_H_
#define SODF_COMPONENTS_TRANSFORM_H_

#include <sodf/ecs.h>
#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct TransformComponent
{
  // Constructor ensures "root" always exists
  TransformComponent()
  {
    geometry::TransformNode origin;
    origin.parent = "";  // no parent, world-fixed
    origin.local = Eigen::Isometry3d::Identity();
    origin.global = Eigen::Isometry3d::Identity();
    origin.dirty = false;
    origin.is_static = true;

    elements.emplace_back("root", std::move(origin));
  }

  std::optional<EntityID> parent_ent_id;
  ElementMap<std::string, geometry::TransformNode> elements;
};

}  // namespace components
}  // namespace sodf

#endif
