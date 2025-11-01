#ifndef SODF_GEOMETRY_CENTROID_H_
#define SODF_GEOMETRY_CENTROID_H_

#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <limits>

namespace sodf {
namespace geometry {

struct VolumeMoments
{
  double V = 0.0;
  Eigen::Vector3d C = Eigen::Vector3d::Zero();
};

template <typename FaceT>
inline VolumeMoments accumulateVolumeMoments(const std::vector<Eigen::Vector3d>& V, const std::vector<FaceT>& F)
{
  VolumeMoments m;
  for (const auto& f : F)
  {
    const Eigen::Vector3d& a = V[f[0]];
    const Eigen::Vector3d& b = V[f[1]];
    const Eigen::Vector3d& c = V[f[2]];
    const double vol6 = a.dot(b.cross(c));
    const double vol = vol6 / 6.0;
    const Eigen::Vector3d tetc = (a + b + c) / 4.0;
    m.V += vol;
    m.C += vol * tetc;
  }
  return m;
}

template <typename FaceT>
inline bool computeVolumeCentroid(const std::vector<Eigen::Vector3d>& V, const std::vector<FaceT>& F,
                                  Eigen::Vector3d& out_centroid)
{
  auto M = accumulateVolumeMoments(V, F);
  if (!std::isfinite(M.V) || std::abs(M.V) < 1e-18)
    return false;
  out_centroid = M.C / M.V;
  return (std::isfinite(out_centroid.x()) && std::isfinite(out_centroid.y()) && std::isfinite(out_centroid.z()));
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_CENTROID_H_
