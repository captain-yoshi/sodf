#ifndef SODF_COMPONENTS_TRANSFORM_H_
#define SODF_COMPONENTS_TRANSFORM_H_

#include <Eigen/Geometry>

#include <sodf/ecs.h>

namespace sodf {
namespace components {

struct RootFrameTag
{
};  // or WorldFrameTag, etc.

struct TransformFrame
{
  std::string parent;                                        // frame ID of parent
  Eigen::Isometry3d local = Eigen::Isometry3d::Identity();   // local relative to parent
  Eigen::Isometry3d global = Eigen::Isometry3d::Identity();  // world-space
  bool dirty = true;

  bool is_static = true;  // false if motion driven by a joint
};

struct TransformComponent
{
  // Constructor ensures "root" always exists
  TransformComponent()
  {
    TransformFrame origin;
    origin.parent = "";  // no parent, world-fixed
    origin.local = Eigen::Isometry3d::Identity();
    origin.global = Eigen::Isometry3d::Identity();
    origin.dirty = false;
    origin.is_static = true;

    transform_map.emplace_back("root", std::move(origin));
  }

  std::optional<EntityID> parent_ent_id;
  FlatMap<std::string, TransformFrame> transform_map;
};

struct AlignFramesComponent
{
  std::string target_id;  // e.g. "table" (from with="table")
  std::string source;     // local transform name (from <Source name="..."/>)
  std::string target;     // target entity transform name (from <Target name="..."/>)
};

struct AlignGeometricPairComponent
{
  std::string target_id;         // e.g. "table"
  double tolerance;              // maximum allowed difference [m] between the lengths
                                 // of vectors (source2 - source1) and (target2 - target1)
  std::string source1, source2;  // local transform names from this entity
  std::string target1, target2;  // local transform names from target entity (target_id)
};

}  // namespace components
}  // namespace sodf

#endif
