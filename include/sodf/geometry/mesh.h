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

// Triangular prism with your convention:
// - Z = height axis
// - X = extrusion/depth (thickness)
// - Y = base-axis of the triangle (triangle lies in the YZ plane)
// Parameters: W (base width along +Y), D (extrusion thickness along X), H (triangle height along +Z), u (apex offset along +Y)
inline void meshifyTriangularPrism(double W, double D, double H, double u, std::vector<Eigen::Vector3d>& V,
                                   std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;
  V.clear();
  F.clear();

  const double x0 = -0.5 * D;  // lower cap (−X)
  const double x1 = +0.5 * D;  // upper cap (+X)

  // Base triangle in the YZ plane:
  //   A = (x, y=0,   z=0)
  //   B = (x, y=W,   z=0)
  //   C = (x, y=u,   z=H)
  const Eigen::Vector3d A0(x0, 0.0, 0.0);
  const Eigen::Vector3d B0(x0, W, 0.0);
  const Eigen::Vector3d C0(x0, u, H);

  const Eigen::Vector3d A1(x1, 0.0, 0.0);
  const Eigen::Vector3d B1(x1, W, 0.0);
  const Eigen::Vector3d C1(x1, u, H);

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

  // Caps:
  // x = -D/2 cap faces −X → reverse winding vs +X
  T(iA0, iC0, iB0);
  // x = +D/2 cap faces +X
  T(iA1, iB1, iC1);

  // Side quads split into triangles, connecting ring at x0 to ring at x1
  // Edge AB
  T(iA0, iB0, iB1);
  T(iA0, iB1, iA1);
  // Edge BC
  T(iB0, iC0, iC1);
  T(iB0, iC1, iB1);
  // Edge CA
  T(iC0, iA0, iA1);
  T(iC0, iA1, iC1);
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
  switch (s.type)
  {
    case geometry::ShapeType::Cylinder:
      meshifyCylinder(s.dimensions.at(0), s.dimensions.at(1), radial_res, V, F);
      return true;
    case geometry::ShapeType::Cone:
      meshifyConeFrustum(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, V, F);
      return true;
    case geometry::ShapeType::TriangularPrism:
    {
      const double W = s.dimensions.at(0);
      const double D = s.dimensions.at(1);
      const double H = s.dimensions.at(2);
      const double u = (s.dimensions.size() > 3) ? s.dimensions[3] : 0.0;
      meshifyTriangularPrism(W, D, H, u, V, F);
      return true;
    }
    case geometry::ShapeType::SphericalSegment:
      meshifySphericalSegment(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), radial_res, axial_res, V, F);
      return true;
    case geometry::ShapeType::Box:
      meshifyBox(s.dimensions.at(0), s.dimensions.at(1), s.dimensions.at(2), V, F);
      return true;
    case geometry::ShapeType::Mesh:
    default:
      return false;
  }
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_H_
