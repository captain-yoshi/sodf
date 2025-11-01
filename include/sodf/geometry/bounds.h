#ifndef SODF_GEOMETRY_BOUNDS_H_
#define SODF_GEOMETRY_BOUNDS_H_

#include <Eigen/Core>
#include <vector>

namespace sodf {
namespace geometry {

struct AABB
{
  Eigen::Vector3d min{ 0, 0, 0 }, max{ 0, 0, 0 };
};

inline AABB computeAABB(const std::vector<Eigen::Vector3d>& V)
{
  if (V.empty())
    return {};
  Eigen::Vector3d mn = V[0], mx = V[0];
  for (const auto& p : V)
  {
    mn = mn.cwiseMin(p);
    mx = mx.cwiseMax(p);
  }
  return { mn, mx };
}

inline Eigen::Vector3d aabbCenter(const AABB& bb)
{
  return 0.5 * (bb.min + bb.max);
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_BOUNDS_H_
