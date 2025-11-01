#ifndef SODF_GEOMETRY_MESH_SHAPE_H_
#define SODF_GEOMETRY_MESH_SHAPE_H_

#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/origin.h>
#include <sodf/geometry/frame.h>

namespace sodf {
namespace geometry {

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
  /* // bottom cap (CCW when seen from +Z) */
  /* for (int i = 1; i + 1 < sides; ++i) */
  /*   T(iBotC, static_cast<Index>(i), static_cast<Index>(i + 1)); */
  /* // top cap (CCW from +Z) */
  /* for (int i = 1; i + 1 < sides; ++i) */
  /*   T(iTopC, static_cast<Index>(sides + i + 1), static_cast<Index>(sides + i)); */

  // bottom cap (outward normal -Z): reverse winding
  // for (int i = 1; i + 1 < sides; ++i)
  //   T(iBotC, static_cast<Index>(i + 1), static_cast<Index>(i));

  // // top cap (outward normal +Z): keep CCW
  // for (int i = 1; i + 1 < sides; ++i)
  //   T(iTopC, static_cast<Index>(sides + i), static_cast<Index>(sides + i + 1));

  // bottom cap (outward normal -Z): reverse winding
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iBotC, i1, i0);
  }

  // top cap (outward normal +Z): CCW when viewed from +Z
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iTopC, static_cast<Index>(sides + i0), static_cast<Index>(sides + i1));
  }
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
  /* for (int i = 1; i + 1 < sides; ++i) */
  /*   T(iBotC, static_cast<Index>(i), static_cast<Index>(i + 1)); */
  /* for (int i = 1; i + 1 < sides; ++i) */
  /*   T(iTopC, static_cast<Index>(sides + i + 1), static_cast<Index>(sides + i)); */

  // // bottom (outward normal -Z): reverse winding
  // for (int i = 1; i + 1 < sides; ++i)
  //   T(iBotC, static_cast<Index>(i + 1), static_cast<Index>(i));

  // // top (outward normal +Z): keep CCW
  // for (int i = 1; i + 1 < sides; ++i)
  //   T(iTopC, static_cast<Index>(sides + i), static_cast<Index>(sides + i + 1));

  // bottom cap (outward normal -Z): reverse winding
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iBotC, i1, i0);
  }

  // top cap (outward normal +Z): CCW when viewed from +Z
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iTopC, static_cast<Index>(sides + i0), static_cast<Index>(sides + i1));
  }
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

  const double eps = 1e-12;       // general small epsilon
  const double eps_ring = 1e-12;  // ring radius collapse threshold

  // Geometry: find sphere center offset z0 and sphere radius r
  // s.t. ring radius at z=0 is a1 and at z=H is a2.
  const double z0 = (a2 * a2 - a1 * a1 + H * H) / (2.0 * H);
  const double r = std::sqrt(std::max(0.0, a1 * a1 + z0 * z0));

  auto ring_radius = [&](double z) -> double {
    const double d2 = r * r - (z0 - z) * (z0 - z);
    return (d2 > 0.0) ? std::sqrt(d2) : 0.0;
  };

  // Build rings; collapse to a single vertex if the ring radius ~ 0
  std::vector<Index> ring_start(axial_res + 1);
  std::vector<int> ring_count(axial_res + 1);

  for (int k = 0; k <= axial_res; ++k)
  {
    const double z = (double(k) / double(axial_res)) * H;
    const double rk = ring_radius(z);

    ring_start[k] = static_cast<Index>(V.size());
    if (rk < eps_ring)
    {
      // Collapse to a pole (single vertex)
      V.emplace_back(0.0, 0.0, z);
      ring_count[k] = 1;
    }
    else
    {
      // Regular ring with radial_res samples
      for (int i = 0; i < radial_res; ++i)
      {
        const double a = 2.0 * M_PI * double(i) / double(radial_res);
        V.emplace_back(rk * std::cos(a), rk * std::sin(a), z);
      }
      ring_count[k] = radial_res;
    }
  }

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // Side bands
  for (int k = 0; k < axial_res; ++k)
  {
    const Index r0 = ring_start[k];
    const Index r1 = ring_start[k + 1];
    const int n0 = ring_count[k];
    const int n1 = ring_count[k + 1];

    // Case A: both proper rings
    if (n0 > 1 && n1 > 1)
    {
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);

        const Index v00 = r0 + i0;  // lower ring, current
        const Index v01 = r0 + i1;  // lower ring, next
        const Index v10 = r1 + i0;  // upper ring, current
        const Index v11 = r1 + i1;  // upper ring, next

        // Keep same winding as cylinder/cone bands (outward CCW)
        T(v00, v10, v11);
        T(v00, v11, v01);
      }
      continue;
    }

    // Case B: lower ring is a pole, upper is a proper ring → fan from pole up
    if (n0 == 1 && n1 > 1)
    {
      const Index pole = r0;  // single vertex
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);
        const Index b0 = r1 + i0;
        const Index b1 = r1 + i1;

        // (pole, upper[i], upper[i+1]) maintains outward normals
        T(pole, b0, b1);
      }
      continue;
    }

    // Case C: upper ring is a pole, lower is a proper ring → fan up to pole
    if (n0 > 1 && n1 == 1)
    {
      const Index pole = r1;  // single vertex
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);
        const Index a0 = r0 + i0;
        const Index a1 = r0 + i1;

        // (lower[i], pole, lower[i+1]) keeps outward normals
        T(a0, pole, a1);
      }
      continue;
    }

    // Case D: both are poles (degenerate band) → nothing to add
  }

  // Planar caps:
  // Add a bottom cap only if a1>0; add a top cap only if a2>0.
  // If an end collapsed to a pole (a*≈0), there is no planar cap to add.
  const bool have_base_cap = (a1 > eps);
  const bool have_top_cap = (a2 > eps);

  // Bottom cap (normal −Z): fan with reversed winding
  if (have_base_cap && ring_count.front() > 1)
  {
    const Index rB = ring_start.front();
    const Index c = static_cast<Index>(V.size());
    V.emplace_back(0.0, 0.0, 0.0);

    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      // Reverse (center, i1, i0) → outward −Z
      T(c, rB + i1, rB + i0);
    }
  }

  // Top cap (normal +Z): fan with CCW winding
  if (have_top_cap && ring_count.back() > 1)
  {
    const Index rT = ring_start.back();
    const Index c = static_cast<Index>(V.size());
    V.emplace_back(0.0, 0.0, H);

    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      // (center, i0, i1) → outward +Z
      T(c, rT + i0, rT + i1);
    }
  }
}

// // Spherical segment (axis +Z), CCW outward
// inline void meshifySphericalSegment(double a1, double a2, double H, int radial_res, int axial_res,
//                                     std::vector<Eigen::Vector3d>& V, std::vector<TriangleMesh::Face>& F)
// {
//   using Index = TriangleMesh::Index;

//   V.clear();
//   F.clear();
//   radial_res = std::max(3, radial_res);
//   axial_res = std::max(1, axial_res);
//   const double eps = 1e-12;

//   const double z0 = (a2 * a2 - a1 * a1 + H * H) / (2.0 * H);
//   const double r = std::sqrt(std::max(0.0, a1 * a1 + z0 * z0));

//   auto ring_radius = [&](double z) -> double {
//     const double ah2 = r * r - (z0 - z) * (z0 - z);
//     return (ah2 > 0.0) ? std::sqrt(ah2) : 0.0;
//   };

//   // rings
//   std::vector<Index> ring_start(axial_res + 1);
//   for (int k = 0; k <= axial_res; ++k)
//   {
//     const double z = double(k) * H / double(axial_res);
//     const double rk = ring_radius(z);
//     ring_start[k] = static_cast<Index>(V.size());

//     for (int i = 0; i < radial_res; ++i)
//     {
//       const double a = 2.0 * M_PI * double(i) / double(radial_res);
//       V.emplace_back(rk * std::cos(a), rk * std::sin(a), z);
//     }
//   }

//   // cap centers
//   const bool have_base_cap = (a1 > eps);
//   const bool have_top_cap = (a2 > eps);

//   Index base_center_idx = std::numeric_limits<Index>::max();
//   Index top_center_idx = std::numeric_limits<Index>::max();
//   if (have_base_cap)
//   {
//     base_center_idx = static_cast<Index>(V.size());
//     V.emplace_back(0, 0, 0);
//   }
//   if (have_top_cap)
//   {
//     top_center_idx = static_cast<Index>(V.size());
//     V.emplace_back(0, 0, H);
//   }

//   auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

//   // side band
//   for (int k = 0; k < axial_res; ++k)
//   {
//     const Index r0 = ring_start[k];
//     const Index r1 = ring_start[k + 1];
//     for (int i = 0; i < radial_res; ++i)
//     {
//       const Index i0 = static_cast<Index>(i);
//       const Index i1 = static_cast<Index>((i + 1) % radial_res);

//       const Index v00 = r0 + i0;
//       const Index v01 = r0 + i1;
//       const Index v10 = r1 + i0;
//       const Index v11 = r1 + i1;

//       T(v00, v10, v11);
//       T(v00, v11, v01);
//     }
//   }

//   // base cap (normal -Z)
//   if (have_base_cap)
//   {
//     const Index rB = ring_start[0];
//     for (int i = 0; i < radial_res; ++i)
//     {
//       const Index i0 = static_cast<Index>(i);
//       const Index i1 = static_cast<Index>((i + 1) % radial_res);
//       T(base_center_idx, rB + i1, rB + i0);
//     }
//   }

//   // top cap (normal +Z)
//   if (have_top_cap)
//   {
//     const Index rT = ring_start[axial_res];
//     for (int i = 0; i < radial_res; ++i)
//     {
//       const Index i0 = static_cast<Index>(i);
//       const Index i1 = static_cast<Index>((i + 1) % radial_res);
//       T(top_center_idx, rT + i0, rT + i1);
//     }
//   }
// }

// Canonical tessellation only: verts in base-aligned frame with z ∈ [0, H].
// No origin policy, no per-instance scale, no custom axes.
inline bool meshifyPrimitive(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                             std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
{
  using geometry::ShapeType;

  V.clear();
  F.clear();

  switch (s.type)
  {
    case ShapeType::Cylinder:
      meshifyCylinder(s.dimensions.at(0), s.dimensions.at(1), std::max(3, radial_res), V, F);
      return true;

    case ShapeType::Cone:
      meshifyConeFrustum(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), std::max(3, radial_res), V, F);
      return true;

    case ShapeType::TriangularPrism:
    {
      const double W = s.dimensions.at(0);
      const double D = s.dimensions.at(1);
      const double H = s.dimensions.at(2);
      const double u = (s.dimensions.size() > 3) ? s.dimensions[3] : 0.0;
      meshifyTriangularPrism(W, D, H, u, V, F);
      return true;
    }

    case ShapeType::SphericalSegment:
      meshifySphericalSegment(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), std::max(3, radial_res),
                              std::max(1, axial_res), V, F);
      return true;

    case ShapeType::Box:
      meshifyBox(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), V, F);
      return true;

    case ShapeType::Mesh:
    default:
      return false;  // non-primitive or unsupported here
  }
}

// Canonical tessellation + application of per-instance options:
//   1) origin policy translation
//   2) non-uniform instance scale (around origin)
//   3) user-provided axes (right-handed ONB from s.axes[0..2])
inline bool meshifyPrimitiveApplied(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                                    std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
{
  if (!meshifyPrimitive(s, V, F, radial_res, axial_res))
    return false;

  // 1) Apply origin policy (moves vertices so (0,0,0) matches the policy)
  applyOriginPolicyMesh(s, V, F);

  // 2) Apply per-instance non-uniform scale around origin
  if (s.scale != Eigen::Vector3d(1.0, 1.0, 1.0))
  {
    for (auto& p : V)
      p = p.cwiseProduct(s.scale);
  }

  // 3) Apply axes (if provided)
  if (s.axes.size() >= 3)
  {
    Eigen::Vector3d X = s.axes[0].normalized();
    Eigen::Vector3d Y = s.axes[1].normalized();
    // Gram–Schmidt to keep ONB robust, then Z = X×Y
    Y -= X * (X.dot(Y));
    if (Y.norm() > 0)
      Y.normalize();
    Eigen::Vector3d Z = X.cross(Y);
    if (Z.norm() > 0)
      Z.normalize();

    Eigen::Matrix3d R;
    R.col(0) = X;  // canonical X
    R.col(1) = Y;  // canonical Y
    R.col(2) = Z;  // canonical Z

    for (auto& p : V)
      p = R * p;
  }

  return true;
}

// inline bool meshifyPrimitive(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
//                              std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
// {
//   bool ok = false;

//   switch (s.type)
//   {
//     case geometry::ShapeType::Cylinder:
//       meshifyCylinder(s.dimensions.at(0), s.dimensions.at(1), radial_res, V, F);
//       ok = true;
//       break;

//     case geometry::ShapeType::Cone:
//       meshifyConeFrustum(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, V, F);
//       ok = true;
//       break;

//     case geometry::ShapeType::TriangularPrism:
//     {
//       const double W = s.dimensions.at(0);
//       const double D = s.dimensions.at(1);
//       const double H = s.dimensions.at(2);
//       const double u = (s.dimensions.size() > 3) ? s.dimensions[3] : 0.0;
//       meshifyTriangularPrism(W, D, H, u, V, F);
//       ok = true;
//       break;
//     }

//     case geometry::ShapeType::SphericalSegment:
//       meshifySphericalSegment(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, axial_res, V,
//       F); ok = true; break;

//     case geometry::ShapeType::Box:
//       meshifyBox(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), V, F);
//       ok = true;
//       break;

//     case geometry::ShapeType::Mesh:
//     default:
//       return false;  // not a primitive
//   }

//   if (!ok)
//     return false;

//   // 1) Apply origin policy (translates vertices so (0,0,0) matches the policy)
//   applyOriginPolicyMesh(s, V, F);

//   // 2) Apply per-instance non-uniform scale around the origin (safe since origin now at 0)
//   if (s.scale != Eigen::Vector3d(1, 1, 1))
//   {
//     for (auto& p : V)
//       p = p.cwiseProduct(s.scale);
//   }

//   // 3) Apply axes (local canonical, user axes)
//   if (s.axes.size() >= 3)
//   {
//     // Build a right-handed ONB from the provided axes (X,Y,Z) in canonical order
//     Eigen::Vector3d X = s.axes[0].normalized();
//     Eigen::Vector3d Y = s.axes[1].normalized();
//     Eigen::Vector3d Z = s.axes[2].normalized();

//     // Optional: enforce orthonormality if users supply near-but-not-perfect axes
//     // Recompute Y ⟂ X, then Z = X×Y, then renormalize (Gram–Schmidt)
//     Y -= X * (X.dot(Y));
//     if (Y.norm() > 0)
//       Y.normalize();
//     Z = X.cross(Y);
//     if (Z.norm() > 0)
//       Z.normalize();

//     Eigen::Matrix3d R;
//     R.col(0) = X;  // canonical X
//     R.col(1) = Y;  // canonical Y
//     R.col(2) = Z;  // canonical Z

//     for (auto& p : V)
//       p = R * p;
//   }

//   return true;
// }

// inline bool meshifyStackedShapePrimitives(const geometry::StackedShape& stacked, int radial_res,
//                                           int axial_res_for_spherical_segments, double weld_tol,
//                                           std::vector<Eigen::Vector3d>& V_out,
//                                           std::vector<geometry::TriangleMesh::Face>& F_out)
// {
//   using geometry::ShapeType;
//   using geometry::TriangleMesh;
//   using Face = TriangleMesh::Face;
//   using Index = TriangleMesh::Index;

//   V_out.clear();
//   F_out.clear();

//   if (stacked.shapes.empty())
//     return false;

//   radial_res = std::max(radial_res, 3);
//   const double cap_tol = 1e-12;
//   const int num_entries = static_cast<int>(stacked.shapes.size());

//   bool warned_mesh_once = false;

//   // For each segment in the stack, we now:
//   //  - meshify in canonical coords (+Z is the primitive's "height/symmetry")
//   //  - bake instance scale
//   //  - optionally strip that primitive's own local end caps
//   //  - rotate into the shape's declared axes (s.axes)
//   //  - apply the *full* base_transform (which may include tilt / lateral offset)
//   //
//   // We then append those world-ish coords into V_out / F_out without trying to
//   // compress them into a single common axis frame.
//   //
//   // NOTE: We intentionally DO NOT:
//   //  - decompose base_transform into yaw_abs / z_base
//   //  - enforce coaxial alignment along stacked.axis_stack_direction
//   //  - apply a final R_axes from orthonormal_basis()
//   //
//   // That means each shape can be arbitrarily oriented / placed.
//   //
//   for (int si = 0; si < num_entries; ++si)
//   {
//     const auto& entry = stacked.shapes[si];
//     const auto& s = entry.shape;

//     // Skip inline Mesh shapes for now (or just append directly if you want)
//     if (s.type == ShapeType::Mesh)
//     {
//       if (!warned_mesh_once)
//       {
//         // std::cout << "meshifyStackedShapePrimitives: Mesh entries are currently ignored "
//         //           << "in fused primitive shell (TODO: stitch external meshes)." << std::endl;
//         warned_mesh_once = true;
//       }
//       continue;
//     }

//     // ---- 1. Get the primitive tessellation in its canonical frame ----
//     std::vector<Eigen::Vector3d> Vseg;
//     std::vector<Face> Fseg;

//     // axial resolution: cylinders/cones are 1 "slice" tall,
//     // spherical segments get finer sampling along height
//     const int axial_res =
//         (s.type == ShapeType::Cylinder || s.type == ShapeType::Cone) ? 1 : axial_res_for_spherical_segments;

//     if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial_res))
//     {
//       // std::cout << "meshifyStackedShapePrimitives: meshifyPrimitive failed for type '"
//       //           << geometry::shapeTypeToString(s.type) << "' (idx=" << si << ")." << std::endl;
//       return false;
//     }

//     // ---- 2. Bake per-instance non-uniform scale into local verts ----
//     const double sx = s.scale.x();
//     const double sy = s.scale.y();
//     const double sz = s.scale.z();
//     for (auto& p : Vseg)
//     {
//       p.x() *= sx;
//       p.y() *= sy;
//       p.z() *= sz;
//     }

//     // ---- 3. Optionally remove the primitive's own end caps ----
//     //
//     // We strip local z==0 and z==H caps BEFORE we rotate/tilt,
//     // so we don't get big internal "lids" between stacked segments.
//     //
//     // WARNING:
//     //   If segments are *not* actually touching end-to-end anymore
//     //   (because you tilted or offset them so they only partially overlap),
//     //   removing these caps will leave literal holes. At that point you
//     //   really need a boolean union, not cap-stripping.
//     //
//     double seg_H = 0.0;
//     switch (s.type)
//     {
//       case ShapeType::Cylinder:  // dims: [radius, height]
//         seg_H = s.dimensions.at(1) * sz;
//         break;
//       case ShapeType::Cone:  // dims: [r0, r1, height]
//         seg_H = s.dimensions.at(2) * sz;
//         break;
//       case ShapeType::SphericalSegment:  // dims: [r0?, r1?, height]
//         seg_H = s.dimensions.at(2) * sz;
//         break;
//       case ShapeType::Box:  // dims: [W, D, H]
//         seg_H = s.dimensions.at(2) * sz;
//         break;
//       default:
//         seg_H = 0.0;
//         break;
//     }

//     stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol);
//     stripCapAtPlaneZ(Vseg, Fseg, seg_H, cap_tol);

//     // ---- 4. Build the shape's own orientation matrix from s.axes ----
//     //
//     // buildCanonicalAxes(s, Xc, Yc, Zc) gives you an orthonormal, right-handed frame
//     // consistent with how this shape wants to be oriented in its *parent* frame.
//     //   - For revolute solids: Zc = symmetry axis, Xc/Yc span the base plane.
//     //   - For boxes: Xc = depth, Yc = width, Zc = height, etc.
//     //
//     // That means multiplying a canonical vertex by [Xc Yc Zc] puts it in the
//     // "shape frame" that respects s.axes.
//     //
//     Eigen::Vector3d Xc, Yc, Zc;
//     buildCanonicalAxes(s, Xc, Yc, Zc, 1e-9);

//     Eigen::Matrix3d R_shape;
//     R_shape.col(0) = Xc;
//     R_shape.col(1) = Yc;
//     R_shape.col(2) = Zc;

//     // ---- 5. Transform verts:
//     //        canonical -> (s.axes orientation) -> entry.base_transform
//     //
//     // entry.base_transform is a *full* SE(3) pose for this shape within the
//     // stacked frame. It may include tilt, arbitrary rotation, lateral offset, etc.
//     //
//     // We'll bake both rotations and the translation right into the vertex
//     // coordinates that we append to V_out.
//     //
//     const Eigen::Matrix3d R_entry = entry.base_transform.linear();
//     const Eigen::Vector3d t_entry = entry.base_transform.translation();

//     for (auto& p : Vseg)
//     {
//       // p was canonical+scaled, with Z=primitive's local "height".
//       // First orient it using the shape's declared axes:
//       Eigen::Vector3d p_oriented = R_shape * p;
//       // Then apply the segment's full pose in stacked frame:
//       p = R_entry * p_oriented + t_entry;
//     }

//     // ---- 6. Append into global buffers (stack frame coords)
//     appendMesh(Vseg, Fseg, V_out, F_out);
//   }

//   // If we never appended anything, bail
//   if (V_out.empty() || F_out.empty())
//     return false;

//   // ---- 7. Weld coincident verts across segments
//   //
//   // This will merge boundaries when two segments butt up cleanly.
//   // If they were tilted/offset and don't actually match perfectly,
//   // weldVertices won't fix gaps, but it also won't destroy anything.
//   //
//   weldVertices(V_out, F_out, weld_tol);

//   // NOTE: We DO NOT:
//   //   - add global planar caps,
//   //   - try to "close" the open ends,
//   //   - rotate everything again with orthonormal_basis(...).
//   //
//   // The fused mesh is now expressed directly in the StackedShape's own frame:
//   // whatever frame `entry.base_transform` lives in.
//   //
//   // Downstream code (like physics / RViz) should position this fused mesh
//   // in world by multiplying with the container's world pose (e.g. the bottom
//   // link pose or whatever you're using).
//   //
//   return true;
// }

inline bool meshifyStackedShapePrimitives(const geometry::StackedShape& stacked, int radial_res,
                                          int axial_res_for_spherical_segments, double weld_tol,
                                          std::vector<Eigen::Vector3d>& V_out,
                                          std::vector<geometry::TriangleMesh::Face>& F_out)
{
  using geometry::ShapeType;
  using geometry::TriangleMesh;
  using Face = TriangleMesh::Face;
  using Index = TriangleMesh::Index;

  V_out.clear();
  F_out.clear();

  if (stacked.shapes.empty())
    return false;

  radial_res = std::max(3, radial_res);
  const double cap_tol = 1e-12;

  bool warned_mesh_once = false;

  // Build each segment:
  //   1) Canonical tessellation (meshifyPrimitive) → z∈[0,H] in the primitive's canonical frame.
  //   2) Apply per-instance non-uniform scale (around origin) — still canonical.
  //   3) Strip canonical end caps at z=0 and z=H (prevents internal lids when segments butt).
  //   4) Apply the shape's axes (right-handed ONB derived from s.axes).
  //   5) Apply entry.base_transform (full SE3 in stacked frame: tilt/offset allowed).
  //
  for (int si = 0; si < static_cast<int>(stacked.shapes.size()); ++si)
  {
    const auto& entry = stacked.shapes[si];
    const auto& s = entry.shape;

    // Skip inline Mesh for now (or append your external mesh directly if desired).
    if (s.type == ShapeType::Mesh)
    {
      if (!warned_mesh_once)
      {
        // std::cout << "meshifyStackedShapePrimitives: Mesh entries are currently ignored "
        //              "(TODO: stitch/append external meshes)." << std::endl;
        warned_mesh_once = true;
      }
      continue;
    }

    // ---- 1) Canonical tessellation ----
    std::vector<Eigen::Vector3d> Vseg;
    std::vector<Face> Fseg;

    // Cylinders/cones: 1 axial slice; spherical segments: supplied axial resolution.
    const int axial_res = (s.type == ShapeType::Cylinder || s.type == ShapeType::Cone) ?
                              1 :
                              std::max(1, axial_res_for_spherical_segments);

    if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial_res))
      return false;

    // ---- 2) Apply per-instance non-uniform scale in canonical frame ----
    const Eigen::Vector3d sc = s.scale;
    if (sc != Eigen::Vector3d(1.0, 1.0, 1.0))
    {
      for (auto& p : Vseg)
        p = p.cwiseProduct(sc);
    }

    // Compute the canonical segment height AFTER scale for cap stripping.
    double seg_H = 0.0;
    switch (s.type)
    {
      case ShapeType::Cylinder:  // dims: [radius, height]
        seg_H = s.dimensions.at(1) * sc.z();
        break;
      case ShapeType::Cone:  // dims: [r0, r1, height]
        seg_H = s.dimensions.at(2) * sc.z();
        break;
      case ShapeType::SphericalSegment:  // dims: [a1, a2, height]
        seg_H = s.dimensions.at(2) * sc.z();
        break;
      case ShapeType::Box:  // dims: [W, D, H]
        seg_H = s.dimensions.at(2) * sc.z();
        break;
      default:
        seg_H = 0.0;
        break;
    }

    // ---- 3) Strip canonical end caps at z=0 and z=seg_H (before any rotation/tilt) ----
    // NOTE: If segments no longer meet cleanly (tilt/offset), this will leave openings.
    // In that case, a proper boolean union is needed.
    const bool is_first = (si == 0);
    const bool is_last = (si == static_cast<int>(stacked.shapes.size()) - 1);

    if (!is_first)
      stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol);  // remove bottom except on first
    if (!is_last)
      stripCapAtPlaneZ(Vseg, Fseg, seg_H, cap_tol);  // remove top except on last

    // stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol);
    // stripCapAtPlaneZ(Vseg, Fseg, seg_H, cap_tol);

    // ---- 4) Apply the shape's axes (ONB from s.axes) ----
    Eigen::Vector3d Xc, Yc, Zc;
    buildCanonicalAxes(s, Xc, Yc, Zc, 1e-9);  // produces a right-handed orthonormal basis

    Eigen::Matrix3d R_shape;
    R_shape.col(0) = Xc;
    R_shape.col(1) = Yc;
    R_shape.col(2) = Zc;

    // ---- 5) Apply the entry's full SE3 pose ----
    const Eigen::Matrix3d R_entry = entry.base_transform.linear();
    const Eigen::Vector3d t_entry = entry.base_transform.translation();

    for (auto& p : Vseg)
    {
      // canonical → shape axes → segment pose in stacked frame
      p = R_entry * (R_shape * p) + t_entry;
    }

    // Append into fused buffers (already in stacked frame coords).
    appendMesh(Vseg, Fseg, V_out, F_out);
  }

  if (V_out.empty() || F_out.empty())
    return false;

  // ---- 6) Weld coincident vertices across segments (tolerant merge) ----
  weldVertices(V_out, F_out, weld_tol);

  // We do NOT:
  //  - add global planar caps,
  //  - enforce a common axis frame,
  //  - re-apply any origin policy here (stacked frame is the reference).
  //
  // Result is expressed directly in the StackedShape frame.
  return true;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_SHAPE_H_
