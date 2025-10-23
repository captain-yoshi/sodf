#ifndef SODF_GEOMETRY_MESH_H_
#define SODF_GEOMETRY_MESH_H_

#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace geometry {

// origin policy application for meshified primitives
namespace {

// AABB (min/max) and center
struct AABB
{
  Eigen::Vector3d min, max;
};
inline AABB computeAABB(const std::vector<Eigen::Vector3d>& V)
{
  if (V.empty())
    return { { 0, 0, 0 }, { 0, 0, 0 } };
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

// Collect indices of vertices that lie on the base plane z = zmin (within eps)
inline std::vector<size_t> baseVertices(const std::vector<Eigen::Vector3d>& V, double zmin, double eps)
{
  std::vector<size_t> idx;
  idx.reserve(V.size());
  for (size_t i = 0; i < V.size(); ++i)
    if (std::abs(V[i].z() - zmin) <= eps)
      idx.push_back(i);
  return idx;
}

// Base-center: center of the base polygon (use average of base vertices; robust and simple)
inline Eigen::Vector3d baseCenterXY(const std::vector<Eigen::Vector3d>& V, double zmin, double eps)
{
  const auto idx = baseVertices(V, zmin, eps);
  if (idx.empty())
  {
    // Fallback: use base AABB center in XY at z=zmin
    AABB bb = computeAABB(V);
    return { 0.5 * (bb.min.x() + bb.max.x()), 0.5 * (bb.min.y() + bb.max.y()), zmin };
  }
  Eigen::Vector2d acc(0, 0);
  for (size_t i : idx)
    acc += V[i].head<2>();
  acc /= double(idx.size());
  return { acc.x(), acc.y(), zmin };
}

// Signed volume and centroid via tetrahedra from origin (assumes closed, outward CCW)
struct VolumeMoments
{
  double V = 0.0;
  Eigen::Vector3d C = Eigen::Vector3d::Zero();  // sum of (tetra centroid * signed volume)
};

inline VolumeMoments accumulateVolumeMoments(const std::vector<Eigen::Vector3d>& V,
                                             const std::vector<geometry::TriangleMesh::Face>& F)
{
  using Face = geometry::TriangleMesh::Face;
  VolumeMoments m;
  for (const Face& f : F)
  {
    const Eigen::Vector3d& a = V[f[0]];
    const Eigen::Vector3d& b = V[f[1]];
    const Eigen::Vector3d& c = V[f[2]];
    // Signed volume of tetra (0,a,b,c) = (a · (b × c)) / 6
    double vol6 = a.dot(b.cross(c));
    double vol = vol6 / 6.0;
    Eigen::Vector3d tet_centroid = (a + b + c) / 4.0;  // centroid of tetra (0,a,b,c)
    m.V += vol;
    m.C += vol * tet_centroid;
  }
  return m;
}

inline bool computeVolumeCentroid(const std::vector<Eigen::Vector3d>& V,
                                  const std::vector<geometry::TriangleMesh::Face>& F, Eigen::Vector3d& out_centroid)
{
  auto M = accumulateVolumeMoments(V, F);
  if (!std::isfinite(M.V) || std::abs(M.V) < 1e-18)
    return false;
  out_centroid = M.C / M.V;
  return std::isfinite(out_centroid.x()) && std::isfinite(out_centroid.y()) && std::isfinite(out_centroid.z());
}

}  // anonymous namespace

inline void applyOriginPolicyToMeshifiedPrimitive(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                                                  const std::vector<geometry::TriangleMesh::Face>& F)
{
  using geometry::OriginPolicy;
  if (V.empty())
    return;

  switch (s.origin)
  {
    case OriginPolicy::Native:
      // No-op
      return;

    case OriginPolicy::AABBCenter:
    {
      AABB bb = computeAABB(V);
      const Eigen::Vector3d ctr = aabbCenter(bb);
      for (auto& p : V)
        p -= ctr;
      return;
    }

    case OriginPolicy::BaseCenter:
    {
      // Use actual base plane z = min z (robust to tiny numerical deviations)
      AABB bb = computeAABB(V);
      const double zmin = bb.min.z();
      const double eps = std::max(1e-12, 1e-6 * (bb.max - bb.min).norm());
      const Eigen::Vector3d bctr = baseCenterXY(V, zmin, eps);
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
        // Fallback to AABBCenter if centroid cannot be computed (open mesh / degenerate)
        AABB bb = computeAABB(V);
        const Eigen::Vector3d ctr = aabbCenter(bb);
        for (auto& p : V)
          p -= ctr;
      }
      return;
    }
  }
}

/*
CONVENTION
- All primitives are tessellated in a base-aligned frame with z E [0, H]
  (i.e., base at z=0 and top at z=H; +Z is the height axis).
*/

inline void meshifyCylinder(double radius, double height, int sides, std::vector<Eigen::Vector3d>& V,
                            std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  V.clear();
  F.clear();
  sides = std::max(3, sides);

  // two rings: z=0 and z=height
  for (int r = 0; r < 2; ++r)
  {
    const double z = (r == 0) ? 0.0 : height;
    for (int i = 0; i < sides; ++i)
    {
      const double a = 2.0 * M_PI * i / sides;
      V.emplace_back(radius * std::cos(a), radius * std::sin(a), z);
    }
  }

  const Index iBotC = static_cast<Index>(V.size());
  V.emplace_back(0, 0, 0);
  const Index iTopC = static_cast<Index>(V.size());
  V.emplace_back(0, 0, height);

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // side quads
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    const Index j0 = static_cast<Index>(i + sides);
    const Index j1 = static_cast<Index>(((i + 1) % sides) + sides);
    T(i0, i1, j1);
    T(i0, j1, j0);
  }
  // bottom cap (CCW when seen from +Z)
  for (int i = 1; i + 1 < sides; ++i)
    T(iBotC, static_cast<Index>(i), static_cast<Index>(i + 1));
  // top cap (CCW from +Z)
  for (int i = 1; i + 1 < sides; ++i)
    T(iTopC, static_cast<Index>(sides + i + 1), static_cast<Index>(sides + i));
}

inline void meshifyConeFrustum(double r0, double r1, double h, int sides, std::vector<Eigen::Vector3d>& V,
                               std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  V.clear();
  F.clear();
  sides = std::max(3, sides);

  for (int r = 0; r < 2; ++r)
  {
    const double z = (r == 0) ? 0.0 : h;
    const double rr = (r == 0) ? r0 : r1;
    for (int i = 0; i < sides; ++i)
    {
      const double a = 2.0 * M_PI * i / sides;
      V.emplace_back(rr * std::cos(a), rr * std::sin(a), z);
    }
  }

  const Index iBotC = static_cast<Index>(V.size());
  V.emplace_back(0, 0, 0);
  const Index iTopC = static_cast<Index>(V.size());
  V.emplace_back(0, 0, h);

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    const Index j0 = static_cast<Index>(i + sides);
    const Index j1 = static_cast<Index>(((i + 1) % sides) + sides);
    T(i0, i1, j1);
    T(i0, j1, j0);
  }
  for (int i = 1; i + 1 < sides; ++i)
    T(iBotC, static_cast<Index>(i), static_cast<Index>(i + 1));
  for (int i = 1; i + 1 < sides; ++i)
    T(iTopC, static_cast<Index>(sides + i + 1), static_cast<Index>(sides + i));
}

// Triangular prism (base in XY, extruded along +Z), with
//   X = triangle height D
//   Y = base width W
//   Z = extrusion/height H
// Apex projection offset u along the base (Y) with 0<=u<=W.
//
// Base triangle at z=0:
//   A = (0, 0, 0)
//   B = (0, W, 0)          // base along +Y
//   C = (D, u, 0)          // apex at +X, offset u along Y
inline void meshifyTriangularPrism(double W, double D, double H, double u, std::vector<Eigen::Vector3d>& V,
                                   std::vector<TriangleMesh::Face>& F)
{
  using Index = geometry::TriangleMesh::Index;
  V.clear();
  F.clear();

  // (Optional) input checks:
  // if (!(W > 0.0 && D > 0.0 && H > 0.0) || u < 0.0 || u > W)
  //   throw std::runtime_error("TriangularPrism: invalid (W>0,D>0,H>0, 0<=u<=W)");

  const double z0 = 0.0, z1 = H;

  // z = 0
  const Eigen::Vector3d A0(0.0, 0.0, z0);
  const Eigen::Vector3d B0(0.0, W, z0);
  const Eigen::Vector3d C0(D, u, z0);

  // z = H
  const Eigen::Vector3d A1(0.0, 0.0, z1);
  const Eigen::Vector3d B1(0.0, W, z1);
  const Eigen::Vector3d C1(D, u, z1);

  const Index iA0 = static_cast<Index>(V.size());
  V.push_back(A0);
  const Index iB0 = static_cast<Index>(V.size());
  V.push_back(B0);
  const Index iC0 = static_cast<Index>(V.size());
  V.push_back(C0);
  const Index iA1 = static_cast<Index>(V.size());
  V.push_back(A1);
  const Index iB1 = static_cast<Index>(V.size());
  V.push_back(B1);
  const Index iC1 = static_cast<Index>(V.size());
  V.push_back(C1);

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // Caps: bottom faces −Z (reverse), top faces +Z (CCW)
  T(iA0, iC0, iB0);  // z=0
  T(iA1, iB1, iC1);  // z=H

  // Sides
  T(iA0, iB0, iB1);
  T(iA0, iB1, iA1);  // AB
  T(iB0, iC0, iC1);
  T(iB0, iC1, iB1);  // BC
  T(iC0, iA0, iA1);
  T(iC0, iA1, iC1);  // CA
}

inline void meshifyBox(double W, double L, double H, std::vector<Eigen::Vector3d>& V, std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  const double hx = 0.5 * W;
  const double hy = 0.5 * L;

  // Centered in X/Y, base-aligned in Z: z ∈ [0, H]
  V = {
    { -hx, -hy, 0 }, { +hx, -hy, 0 }, { +hx, +hy, 0 }, { -hx, +hy, 0 },  // 0..3 bottom
    { -hx, -hy, H }, { +hx, -hy, H }, { +hx, +hy, H }, { -hx, +hy, H }   // 4..7 top
  };

  F.clear();
  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // Bottom (z=0), outward normal = -Z → reversed winding
  T(0, 2, 1);
  T(0, 3, 2);

  // Top (z=H), outward normal = +Z
  T(4, 5, 6);
  T(4, 6, 7);

  // Sides (CCW outward)
  T(0, 1, 5);
  T(0, 5, 4);  // -Y
  T(1, 2, 6);
  T(1, 6, 5);  // +X
  T(2, 3, 7);
  T(2, 7, 6);  // +Y
  T(3, 0, 4);
  T(3, 4, 7);  // -X
}

// Spherical segment (axis +Z), CCW outward
inline void meshifySphericalSegment(double a1, double a2, double H, int radial_res, int axial_res,
                                    std::vector<Eigen::Vector3d>& V, std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  V.clear();
  F.clear();
  radial_res = std::max(3, radial_res);
  axial_res = std::max(1, axial_res);
  const double eps = 1e-12;

  const double z0 = (a2 * a2 - a1 * a1 + H * H) / (2.0 * H);
  const double r = std::sqrt(std::max(0.0, a1 * a1 + z0 * z0));

  auto ring_radius = [&](double z) -> double {
    const double ah2 = r * r - (z0 - z) * (z0 - z);
    return (ah2 > 0.0) ? std::sqrt(ah2) : 0.0;
  };

  // rings
  std::vector<Index> ring_start(axial_res + 1);
  for (int k = 0; k <= axial_res; ++k)
  {
    const double z = double(k) * H / double(axial_res);
    const double rk = ring_radius(z);
    ring_start[k] = static_cast<Index>(V.size());

    for (int i = 0; i < radial_res; ++i)
    {
      const double a = 2.0 * M_PI * double(i) / double(radial_res);
      V.emplace_back(rk * std::cos(a), rk * std::sin(a), z);
    }
  }

  // cap centers
  const bool have_base_cap = (a1 > eps);
  const bool have_top_cap = (a2 > eps);

  Index base_center_idx = std::numeric_limits<Index>::max();
  Index top_center_idx = std::numeric_limits<Index>::max();
  if (have_base_cap)
  {
    base_center_idx = static_cast<Index>(V.size());
    V.emplace_back(0, 0, 0);
  }
  if (have_top_cap)
  {
    top_center_idx = static_cast<Index>(V.size());
    V.emplace_back(0, 0, H);
  }

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // side band
  for (int k = 0; k < axial_res; ++k)
  {
    const Index r0 = ring_start[k];
    const Index r1 = ring_start[k + 1];
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);

      const Index v00 = r0 + i0;
      const Index v01 = r0 + i1;
      const Index v10 = r1 + i0;
      const Index v11 = r1 + i1;

      T(v00, v10, v11);
      T(v00, v11, v01);
    }
  }

  // base cap (normal -Z)
  if (have_base_cap)
  {
    const Index rB = ring_start[0];
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      T(base_center_idx, rB + i1, rB + i0);
    }
  }

  // top cap (normal +Z)
  if (have_top_cap)
  {
    const Index rT = ring_start[axial_res];
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      T(top_center_idx, rT + i0, rT + i1);
    }
  }
}

inline bool meshifyPrimitive(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                             std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
{
  bool ok = false;

  switch (s.type)
  {
    case geometry::ShapeType::Cylinder:
      meshifyCylinder(s.dimensions.at(0), s.dimensions.at(1), radial_res, V, F);
      ok = true;
      break;

    case geometry::ShapeType::Cone:
      meshifyConeFrustum(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, V, F);
      ok = true;
      break;

    case geometry::ShapeType::TriangularPrism:
    {
      const double W = s.dimensions.at(0);
      const double D = s.dimensions.at(1);
      const double H = s.dimensions.at(2);
      const double u = (s.dimensions.size() > 3) ? s.dimensions[3] : 0.0;
      meshifyTriangularPrism(W, D, H, u, V, F);
      ok = true;
      break;
    }

    case geometry::ShapeType::SphericalSegment:
      meshifySphericalSegment(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, axial_res, V, F);
      ok = true;
      break;

    case geometry::ShapeType::Box:
      meshifyBox(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), V, F);
      ok = true;
      break;

    case geometry::ShapeType::Mesh:
    default:
      return false;  // not a primitive
  }

  if (!ok)
    return false;

  // 1) Apply origin policy (translates vertices so (0,0,0) matches the policy)
  applyOriginPolicyToMeshifiedPrimitive(s, V, F);

  // 2) Apply per-instance non-uniform scale around the origin (safe since origin now at 0)
  if (s.scale != Eigen::Vector3d(1, 1, 1))
  {
    for (auto& p : V)
      p = p.cwiseProduct(s.scale);
  }

  return true;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_H_
