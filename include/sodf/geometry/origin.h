#ifndef SODF_GEOMETRY_ORIGIN_H_
#define SODF_GEOMETRY_ORIGIN_H_

#include <vector>
#include <Eigen/Core>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/bounds.h>
#include <sodf/geometry/centroid.h>
#include <sodf/geometry/frame.h>  // computeOrthogonalAxis

namespace sodf {
namespace geometry {

// ============================================================================
// Generic plane helpers (work for any plane n·p = offset).
// n must be unit (we normalize defensively).
// U, V must span the plane and be orthonormal (we fix up defensively).
// ============================================================================

inline std::vector<size_t> verticesOnPlane(const std::vector<Eigen::Vector3d>& V, const Eigen::Vector3d& n_in,
                                           double offset, double tol)
{
  Eigen::Vector3d n = n_in;
  const double nn = n.norm();
  if (!(nn > 0.0) || !std::isfinite(nn))
    n = Eigen::Vector3d::UnitX();
  else
    n /= nn;

  std::vector<size_t> idx;
  idx.reserve(V.size());
  for (size_t i = 0; i < V.size(); ++i)
  {
    const double d = n.dot(V[i]) - offset;
    if (std::abs(d) <= tol)
      idx.push_back(i);
  }
  return idx;
}

// Average (u,v) of vertices lying on plane n·p = offset, where u = U·p, v = Vdir·p.
// Reconstruct the 3D point as p0 + u_avg*U + v_avg*Vdir with p0 any point on the plane.
// We use p0 = offset * n (valid when n is unit).
inline Eigen::Vector3d planeCenterUV(const std::vector<Eigen::Vector3d>& V, const Eigen::Vector3d& n_in, double offset,
                                     const Eigen::Vector3d& U_in, const Eigen::Vector3d& V_in, double tol)
{
  // Make a safe ONB for the plane
  Eigen::Vector3d n = n_in;
  double nn = n.norm();
  if (!(nn > 0.0) || !std::isfinite(nn))
    n = Eigen::Vector3d::UnitX();
  else
    n /= nn;

  Eigen::Vector3d U = U_in, Vdir = V_in;

  // Re-orthonormalize (U,V) against n (robust to near-degenerate inputs)
  // 1) Make U ⟂ n
  U -= n * (n.dot(U));
  double nu = U.norm();
  if (!(nu > 0.0) || !std::isfinite(nu))
    U = computeOrthogonalAxis(n);
  else
    U /= nu;

  // 2) V := n × U (guarantees right-handed plane basis)
  Vdir = n.cross(U);
  double nv = Vdir.norm();
  if (!(nv > 0.0) || !std::isfinite(nv))
  {
    Vdir = computeOrthogonalAxis(n).cross(n);
  }
  else
    Vdir /= nv;

  const std::vector<size_t> idx = verticesOnPlane(V, n, offset, tol);

  if (idx.empty())
  {
    // Fallback: AABB center projected onto plane
    const AABB bb = computeAABB(V);
    Eigen::Vector3d c = aabbCenter(bb);
    // Project c to plane and return
    const double d = n.dot(c) - offset;
    return c - d * n;
  }

  // Average u,v
  double u_acc = 0.0, v_acc = 0.0;
  for (size_t i : idx)
  {
    const Eigen::Vector3d& p = V[i];
    u_acc += U.dot(p);
    v_acc += Vdir.dot(p);
  }
  const double invN = 1.0 / static_cast<double>(idx.size());
  const double u0 = u_acc * invN;
  const double v0 = v_acc * invN;

  // Reconstruct 3D point on the plane
  const Eigen::Vector3d p0 = offset * n;  // any point satisfying n·p = offset
  return p0 + u0 * U + v0 * Vdir;
}

// Convenience: base center on the plane orthogonal to a primary axis.
// Here offset is the coordinate along that axis (e.g., xmin).
inline Eigen::Vector3d baseCenterAlong(const std::vector<Eigen::Vector3d>& V,
                                       const Eigen::Vector3d& primary_axis /* need not be unit */,
                                       double base_offset_along_axis, double tol)
{
  const Eigen::Vector3d N = (primary_axis.norm() > 0.0) ? primary_axis.normalized() : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d U = computeOrthogonalAxis(N);
  const Eigen::Vector3d Vdir = (N.cross(U)).normalized();
  return planeCenterUV(V, N, base_offset_along_axis, U, Vdir, tol);
}

// ============================================================================
// Canonical (X-height) helpers retained for convenience.
// These now delegate to the generic variants above.
// ============================================================================

// Kept for compatibility (X-plane query)
inline std::vector<size_t> baseVerticesX(const std::vector<Eigen::Vector3d>& V, double xmin, double eps)
{
  return verticesOnPlane(V, Eigen::Vector3d::UnitX(), xmin, eps);
}

// Kept for compatibility (average in YZ on x=xmin)
inline Eigen::Vector3d baseCenterYZ(const std::vector<Eigen::Vector3d>& V, double xmin, double eps)
{
  // In canonical frame, choose U=+Y, V=+Z
  return planeCenterUV(V, Eigen::Vector3d::UnitX(), xmin, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), eps);
}

// New: compute base center in **canonical X-height** convention using AABB to find xmin.
inline Eigen::Vector3d baseCenterCanonical(const std::vector<Eigen::Vector3d>& V, double tol)
{
  const AABB bb = computeAABB(V);
  const double xmin = bb.min.x();
  return baseCenterAlong(V, Eigen::Vector3d::UnitX(), xmin, tol);
}

// ============================================================================
// 3D origin policy (meshified primitives).
// Assumes input vertices are in the **canonical build frame** unless you
// pre-rotated them yourself (in which case BaseCenter will still work via the
// generic baseCenterAlong if you adjust the axis/offset accordingly).
// ============================================================================
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
      // Canonical base center (X is height): uses generic plane averaging internally.
      const AABB bb = computeAABB(V);
      const double diag = (bb.max - bb.min).norm();
      const double eps = std::max(1e-12, 1e-6 * diag);
      const Eigen::Vector3d bctr = baseCenterCanonical(V, eps);
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

// ============================================================================
// 2D origin policy (planar (u,v) vertices). Unchanged semantics.
// ============================================================================

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
      const AABB bb = computeAABB(uv_vertices);
      const Eigen::Vector3d ctr = aabbCenter(bb);
      for (auto& p : uv_vertices)
        p -= ctr;
      return;
    }

    case OriginPolicy::BaseCenter:
    {
      // “Base” = minimal-v edge in the (u,v) plane
      const AABB bb = computeAABB(uv_vertices);
      const double vmin = bb.min.y();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());

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
      const Eigen::Vector3d center_shift(0, 0.5 * (bb.max.y() - bb.min.y()), 0);
      const Eigen::Vector3d target = base_ctr + center_shift;

      for (auto& p : uv_vertices)
        p -= target;
      return;
    }

    case OriginPolicy::VolumeCentroid:
    {
      // Area-weighted 2D polygon centroid in (u,v)
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
