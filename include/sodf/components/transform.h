#ifndef SODF_COMPONENTS_TRANSFORM_H_
#define SODF_COMPONENTS_TRANSFORM_H_

#include <variant>
#include <sodf/ecs.h>
#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct RootFrameTag
{
};  // or WorldFrameTag, etc.

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

    transform_map.emplace_back("root", std::move(origin));
  }

  std::optional<EntityID> parent_ent_id;
  FlatMap<std::string, geometry::TransformNode> transform_map;
};

struct OriginComponent
{
  using OriginVariant = std::variant<geometry::Transform,       // direct transform (absolute pose)
                                     geometry::AlignFrames,     // single-frame alignment
                                     geometry::AlignPairFrames  // two-frame alignment
                                     >;

  OriginVariant origin;

  // Optional: add constructors for convenience
  // OriginComponent(const geometry::Transform& tf) : origin(tf)
  // {
  // }
  // OriginComponent(const geometry::AlignFrames& af) : origin(af)
  // {
  // }
  // OriginComponent(const geometry::AlignPairFrames& apf) : origin(apf)
  // {
  // }
};

}  // namespace components
}  // namespace sodf

#endif
