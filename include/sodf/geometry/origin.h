#ifndef SODF_GEOMETRY_ORIGIN_H_
#define SODF_GEOMETRY_ORIGIN_H_

#include <vector>
#include <Eigen/Core>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/bounds.h>
#include <sodf/geometry/centroid.h>

namespace sodf {
namespace geometry {

// =====================================================================================
// Helpers for primitives following the canonical convention: x ∈ [0, H] is the height.
// Base plane is x = 0. We compute the base center in the YZ plane at x = xmin.
// =====================================================================================
inline std::vector<size_t> baseVerticesX(const std::vector<Eigen::Vector3d>& V, double xmin, double eps)
{
  std::vector<size_t> idx;
  idx.reserve(V.size());
  for (size_t i = 0; i < V.size(); ++i)
    if (std::abs(V[i].x() - xmin) <= eps)
      idx.push_back(i);
  return idx;
}

inline Eigen::Vector3d baseCenterYZ(const std::vector<Eigen::Vector3d>& V, double xmin, double eps)
{
  const auto idx = baseVerticesX(V, xmin, eps);
  if (idx.empty())
  {
    // Fallback: use AABB center in Y/Z, and place point on x=xmin.
    const auto bb = computeAABB(V);
    return { xmin, 0.5 * (bb.min.y() + bb.max.y()), 0.5 * (bb.min.z() + bb.max.z()) };
  }

  // Average Y and Z over vertices lying on the base plane
  Eigen::Vector2d acc(0, 0);
  for (auto i : idx)
    acc += V[i].segment<2>(1);  // (y,z)
  acc /= double(idx.size());
  return { xmin, acc.x(), acc.y() };
}

// =====================================================================================
// 3D: apply policy to a meshified primitive. Assumes canonical X-height convention.
// - Native:      no change
// - AABBCenter:  translate so AABB center is at the origin
// - BaseCenter:  translate so base center (x=xmin, averaged YZ) is at the origin
// - VolumeCentroid: translate so volume centroid is at the origin; fallback AABBCenter
// =====================================================================================
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
      const double xmin = bb.min.x();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());
      const auto bctr = baseCenterYZ(V, xmin, eps);
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

// =====================================================================================
// 2D: apply policy to planar shapes described with vertices in a local (u,v) plane.
// NOTE: For planar/2D shapes, “BaseCenter” is ambiguous w.r.t. 3D X-height.
// Here we interpret the "base" as the minimal v-edge (v = vmin) in the (u,v) plane,
// which is a practical default for many 2D uses (e.g., rectangles in a Y–Z plane).
// If you want an X-tied base for specific 2D workflows, supply pre-shifted vertices.
// =====================================================================================
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
      // Interpret “base” as minimal-v edge
      AABB bb = computeAABB(uv_vertices);
      const double vmin = bb.min.y();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());

      // Compute center of the base edge in u at v = vmin
      double u_acc = 0.0;
      int n = 0;
      for (const auto& p : uv_vertices)
        if (std::abs(p.y() - vmin) <= eps)
        {
          u_acc += p.x();
          ++n;
        }
      const double u0 = (n > 0) ? (u_acc / n) : 0.5 * (bb.min.x() + bb.max.x());

      // Shift origin to the geometric center along v, but aligned with base u0
      const Eigen::Vector3d base_ctr(u0, vmin, 0);
      const Eigen::Vector3d center_shift(0, 0.5 * (bb.max.y() - bb.min.y()), 0);  // to v-center
      const Eigen::Vector3d target = base_ctr + center_shift;

      for (auto& p : uv_vertices)
        p -= target;
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
