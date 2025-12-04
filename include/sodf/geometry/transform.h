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
  std::string parent;                                             // frame ID of parent
  Eigen::Isometry3d local = Eigen::Isometry3d::Identity();        // local relative to parent
  Eigen::Isometry3d global = Eigen::Isometry3d::Identity();       // world-space
  Eigen::Isometry3d rest_local{ Eigen::Isometry3d::Identity() };  // q=0 authored
  bool dirty = true;

  bool is_static = true;  // false if motion driven by a joint
};

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_TRANSFORM_H_
