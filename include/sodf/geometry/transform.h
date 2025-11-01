#ifndef SODF_GEOMETRY_TRANSFORM_H_
#define SODF_GEOMETRY_TRANSFORM_H_

#include <string>
#include <Eigen/Geometry>

namespace sodf {
namespace geometry {

struct Transform
{
  std::string parent;                                    // frame ID of parent
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();  // local relative to parent
};

struct TransformNode
{
  std::string parent;                                        // frame ID of parent
  Eigen::Isometry3d local = Eigen::Isometry3d::Identity();   // local relative to parent
  Eigen::Isometry3d global = Eigen::Isometry3d::Identity();  // world-space
  bool dirty = true;

  bool is_static = true;  // false if motion driven by a joint
};

struct AlignFrames
{
  std::string target_id;  // e.g. "table" (from with="table")

  std::string source_tf;  // local transform name (from <Source name="..."/>)
  std::string target_tf;  // target entity transform name (from <Target name="..."/>)
};

struct AlignPairFrames
{
  std::string target_id;               // e.g. "table"
                                       // of vectors (source2 - source1) and (target2 - target1)
  std::string source_tf1, source_tf2;  // local transform names from this entity
  std::string target_tf1, target_tf2;  // local transform names from target entity (target_id)

  double tolerance;  // maximum allowed difference [m] between alignement
};

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_TRANSFORM_H_
