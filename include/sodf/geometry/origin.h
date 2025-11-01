#ifndef SODF_GEOMETRY_ORIGIN_H_
#define SODF_GEOMETRY_ORIGIN_H_

#include <vector>
#include <Eigen/Core>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/bounds.h>
#include <sodf/geometry/centroid.h>

namespace sodf {
namespace geometry {

// Reuse your existing helpers
inline std::vector<size_t> baseVertices(const std::vector<Eigen::Vector3d>& V, double zmin, double eps)
{
  std::vector<size_t> idx;
  idx.reserve(V.size());
  for (size_t i = 0; i < V.size(); ++i)
    if (std::abs(V[i].z() - zmin) <= eps)
      idx.push_back(i);
  return idx;
}
inline Eigen::Vector3d baseCenterXY(const std::vector<Eigen::Vector3d>& V, double zmin, double eps)
{
  const auto idx = baseVertices(V, zmin, eps);
  if (idx.empty())
  {
    auto bb = computeAABB(V);
    return { 0.5 * (bb.min.x() + bb.max.x()), 0.5 * (bb.min.y() + bb.max.y()), zmin };
  }
  Eigen::Vector2d acc(0, 0);
  for (auto i : idx)
    acc += V[i].head<2>();
  acc /= double(idx.size());
  return { acc.x(), acc.y(), zmin };
}

// 3D: apply policy to a meshified primitive (your function, just moved here)
inline void applyOriginPolicyMesh(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                                  const std::vector<geometry::TriangleMesh::Face>& F)
{
  using geometry::OriginPolicy;
  if (V.empty())
    return;
  switch (s.origin)
  {
    case OriginPolicy::Native:
      return;
    case OriginPolicy::AABBCenter:
    {
      const auto ctr = aabbCenter(computeAABB(V));
      for (auto& p : V)
        p -= ctr;
      return;
    }
    case OriginPolicy::BaseCenter:
    {
      const auto bb = computeAABB(V);
      const double zmin = bb.min.z();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());
      const auto bctr = baseCenterXY(V, zmin, eps);
      for (auto& p : V)
        p -= bctr;
      return;
    }
    case OriginPolicy::VolumeCentroid:
    {
      Eigen::Vector3d C;
      if (computeVolumeCentroid(V, F, C))
      {
        for (auto& p : V)
          p -= C;
      }
      else
      {
        const auto ctr = aabbCenter(computeAABB(V));
        for (auto& p : V)
          p -= ctr;
      }
      return;
    }
  }
}

// 2D: apply policy to planar shapes described with vertices in (u,v) coordinates.
// vertices are in the local 2D basis spanned by U (axes[1]) and V (axes[2]) around origin O (axes[0], optional).
inline void applyOriginPolicy2DVertices(const geometry::Shape& s,
                                        std::vector<Eigen::Vector3d>& uv_vertices /* z==0 in uv-frame */)
{
  using geometry::OriginPolicy;
  if (uv_vertices.empty())
    return;

  switch (s.origin)
  {
    case OriginPolicy::Native:
      return;

    case OriginPolicy::AABBCenter:
    {
      AABB bb = computeAABB(uv_vertices);
      Eigen::Vector3d ctr = aabbCenter(bb);
      for (auto& p : uv_vertices)
        p -= ctr;
      return;
    }

    case OriginPolicy::BaseCenter:
    {
      // Interpret “base” as the minimal v-edge (v = vmin)
      AABB bb = computeAABB(uv_vertices);
      const double vmin = bb.min.y();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());
      // center of the base edge in u plus v=vmin
      double u_acc = 0.0;
      int n = 0;
      for (const auto& p : uv_vertices)
        if (std::abs(p.y() - vmin) <= eps)
        {
          u_acc += p.x();
          ++n;
        }
      const double u0 = (n > 0) ? (u_acc / n) : 0.5 * (bb.min.x() + bb.max.x());
      const Eigen::Vector3d base_ctr(u0, vmin, 0);
      const Eigen::Vector3d center_shift(0, 0.5 * (bb.max.y() - bb.min.y()), 0);  // to geometric center
      for (auto& p : uv_vertices)
        p -= (base_ctr + center_shift);
      return;
    }

    case OriginPolicy::VolumeCentroid:
    {
      // 2D polygon centroid (area-weighted) in uv-plane
      double A = 0.0;
      Eigen::Vector2d C(0, 0);
      for (size_t i = 0, j = uv_vertices.size() - 1; i < uv_vertices.size(); j = i++)
      {
        const auto& pi = uv_vertices[i];
        const auto& pj = uv_vertices[j];
        const double cross = pj.x() * pi.y() - pi.x() * pj.y();
        A += cross;
        C += cross * Eigen::Vector2d(pj.x() + pi.x(), pj.y() + pi.y());
      }
      if (std::abs(A) > 1e-12)
      {
        C /= (3.0 * A);
        const Eigen::Vector3d c3(C.x(), C.y(), 0);
        for (auto& p : uv_vertices)
          p -= c3;
      }
      else
      {
        // fallback center
        const auto ctr = aabbCenter(computeAABB(uv_vertices));
        for (auto& p : uv_vertices)
          p -= ctr;
      }
      return;
    }
  }
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_ORIGIN_H_
