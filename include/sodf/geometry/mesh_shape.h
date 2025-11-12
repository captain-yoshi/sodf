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

// -----------------------------------------------------------------------------
// Cylinder: rings lie in the YZ plane; x=0 and x=H caps; outward −X/+X caps.
// -----------------------------------------------------------------------------
inline void meshifyCylinder(double radius, double height, int sides, std::vector<Eigen::Vector3d>& V,
                            std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  V.clear();
  F.clear();
  sides = std::max(3, sides);

  // two rings: x=0 and x=height
  for (int r = 0; r < 2; ++r)
  {
    const double x = (r == 0) ? 0.0 : height;
    for (int i = 0; i < sides; ++i)
    {
      const double a = 2.0 * M_PI * i / sides;
      // (x, y, z) with circle in YZ
      V.emplace_back(x, radius * std::cos(a), radius * std::sin(a));
    }
  }

  const Index iBaseC = static_cast<Index>(V.size());
  V.emplace_back(0.0, 0.0, 0.0);
  const Index iTopC = static_cast<Index>(V.size());
  V.emplace_back(height, 0.0, 0.0);

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // side quads (CCW outward)
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    const Index j0 = static_cast<Index>(i + sides);
    const Index j1 = static_cast<Index>(((i + 1) % sides) + sides);
    T(i0, j0, j1);
    T(i0, j1, i1);
  }

  // base cap (outward normal −X): reverse winding
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iBaseC, i1, i0);
  }

  // top cap (outward normal +X): CCW when viewed from +X
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iTopC, static_cast<Index>(sides + i0), static_cast<Index>(sides + i1));
  }
}

// -----------------------------------------------------------------------------
// Cone frustum: r0 at x=0, r1 at x=H; rings in YZ; caps −X/+X.
// -----------------------------------------------------------------------------
inline void meshifyConeFrustum(double r0, double r1, double h, int sides, std::vector<Eigen::Vector3d>& V,
                               std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  V.clear();
  F.clear();
  sides = std::max(3, sides);

  for (int r = 0; r < 2; ++r)
  {
    const double x = (r == 0) ? 0.0 : h;
    const double rr = (r == 0) ? r0 : r1;
    for (int i = 0; i < sides; ++i)
    {
      const double a = 2.0 * M_PI * i / sides;
      V.emplace_back(x, rr * std::cos(a), rr * std::sin(a));
    }
  }

  const Index iBaseC = static_cast<Index>(V.size());
  V.emplace_back(0.0, 0.0, 0.0);
  const Index iTopC = static_cast<Index>(V.size());
  V.emplace_back(h, 0.0, 0.0);

  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // side quads
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    const Index j0 = static_cast<Index>(i + sides);
    const Index j1 = static_cast<Index>(((i + 1) % sides) + sides);
    T(i0, j0, j1);
    T(i0, j1, i1);
  }

  // base cap (−X)
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iBaseC, i1, i0);
  }

  // top cap (+X)
  for (int i = 0; i < sides; ++i)
  {
    const Index i0 = static_cast<Index>(i);
    const Index i1 = static_cast<Index>((i + 1) % sides);
    T(iTopC, static_cast<Index>(sides + i0), static_cast<Index>(sides + i1));
  }
}

// -----------------------------------------------------------------------------
// Triangular prism: extruded along +X (lengthX), base triangle in YZ at x=0.
// Base parameters follow Shape spec: length_x, base_y, altitude_z, apex_offset_y.
// -----------------------------------------------------------------------------
inline void meshifyTriangularPrism(double lengthX, double baseY, double altitudeZ, double u,
                                   std::vector<Eigen::Vector3d>& V, std::vector<TriangleMesh::Face>& F)
{
  using Index = geometry::TriangleMesh::Index;
  V.clear();
  F.clear();

  // Base triangle at x=0 in YZ:
  //   A0=(0, 0,        0)
  //   B0=(0, baseY,    0)       // base along +Y
  //   C0=(0, u, altitudeZ)      // apex offset u along Y, altitude along +Z
  const Eigen::Vector3d A0(0.0, 0.0, 0.0);
  const Eigen::Vector3d B0(0.0, baseY, 0.0);
  const Eigen::Vector3d C0(0.0, u, altitudeZ);

  // Top triangle at x=lengthX (same YZ):
  const Eigen::Vector3d A1(lengthX, 0.0, 0.0);
  const Eigen::Vector3d B1(lengthX, baseY, 0.0);
  const Eigen::Vector3d C1(lengthX, u, altitudeZ);

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

  // Caps: base (−X) reversed; top (+X) CCW
  T(iA0, iC0, iB0);  // x=0 (−X)
  T(iA1, iB1, iC1);  // x=L (+X)

  // Sides (CCW outward)
  T(iA0, iB0, iB1);
  T(iA0, iB1, iA1);  // AB
  T(iB0, iC0, iC1);
  T(iB0, iC1, iB1);  // BC
  T(iC0, iA0, iA1);
  T(iC0, iA1, iC1);  // CA
}

// -----------------------------------------------------------------------------
// Box: axis-aligned, centered in Y/Z, base-aligned in X (x ∈ [0, Xlen])
// Dimensions follow Shape spec: x, y, z (extents).
// -----------------------------------------------------------------------------
inline void meshifyBox(double Xlen, double Ylen, double Zlen, std::vector<Eigen::Vector3d>& V,
                       std::vector<TriangleMesh::Face>& F)
{
  using Index = TriangleMesh::Index;

  const double hy = 0.5 * Ylen;
  const double hz = 0.5 * Zlen;

  // Centered in Y/Z, base-aligned in X
  V = {
    { 0, -hy, -hz },    { 0, +hy, -hz },    { 0, +hy, +hz },    { 0, -hy, +hz },    // 0..3 base (x=0)
    { Xlen, -hy, -hz }, { Xlen, +hy, -hz }, { Xlen, +hy, +hz }, { Xlen, -hy, +hz }  // 4..7 top  (x=Xlen)
  };

  F.clear();
  auto T = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // Base (x=0), outward −X → reversed
  T(0, 2, 1);
  T(0, 3, 2);

  // Top  (x=Xlen), outward +X
  T(4, 5, 6);
  T(4, 6, 7);

  // Sides (CCW outward)
  T(0, 1, 5);
  T(0, 5, 4);  // -Z side
  T(1, 2, 6);
  T(1, 6, 5);  // +Y
  T(2, 3, 7);
  T(2, 7, 6);  // +Z
  T(3, 0, 4);
  T(3, 4, 7);  // -Y
}

// -----------------------------------------------------------------------------
// Spherical segment along X: rings in YZ; caps −X/+X.
// a1 = base radius at x=0; a2 = top radius at x=H.
// -----------------------------------------------------------------------------
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

  // Geometry in X: sphere center at x0, radius r
  const double x0 = (a2 * a2 - a1 * a1 + H * H) / (2.0 * H);
  const double r = std::sqrt(std::max(0.0, a1 * a1 + x0 * x0));

  auto ring_radius = [&](double x) -> double {
    const double d2 = r * r - (x0 - x) * (x0 - x);
    return (d2 > 0.0) ? std::sqrt(d2) : 0.0;
  };

  // Build rings; collapse to a single vertex if the ring radius ~ 0
  std::vector<Index> ring_start(axial_res + 1);
  std::vector<int> ring_count(axial_res + 1);

  for (int k = 0; k <= axial_res; ++k)
  {
    const double xk = (double(k) / double(axial_res)) * H;
    const double rk = ring_radius(xk);

    ring_start[k] = static_cast<Index>(V.size());
    if (rk < eps_ring)
    {
      // Collapse to a pole (single vertex on axis)
      V.emplace_back(xk, 0.0, 0.0);
      ring_count[k] = 1;
    }
    else
    {
      // Regular ring with radial_res samples in YZ
      for (int i = 0; i < radial_res; ++i)
      {
        const double a = 2.0 * M_PI * double(i) / double(radial_res);
        V.emplace_back(xk, rk * std::cos(a), rk * std::sin(a));
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

    // A) both proper rings
    if (n0 > 1 && n1 > 1)
    {
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);

        const Index v00 = r0 + i0;  // lower ring (smaller x), current
        const Index v01 = r0 + i1;  // lower ring, next
        const Index v10 = r1 + i0;  // upper ring (larger x), current
        const Index v11 = r1 + i1;  // upper ring, next

        // Keep outward CCW (consistent with cylinder/cone bands)
        T(v00, v10, v11);
        T(v00, v11, v01);
      }
      continue;
    }

    // B) lower ring is a pole, upper is proper → fan from pole up
    if (n0 == 1 && n1 > 1)
    {
      const Index pole = r0;  // single vertex
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);
        const Index b0 = r1 + i0;
        const Index b1 = r1 + i1;
        // (pole, upper[i], upper[i+1]) → outward
        T(pole, b0, b1);
      }
      continue;
    }

    // C) upper ring is a pole, lower is proper → fan up to pole
    if (n0 > 1 && n1 == 1)
    {
      const Index pole = r1;
      for (int i = 0; i < radial_res; ++i)
      {
        const Index i0 = static_cast<Index>(i);
        const Index i1 = static_cast<Index>((i + 1) % radial_res);
        const Index a0 = r0 + i0;
        const Index a1 = r0 + i1;
        // (lower[i], pole, lower[i+1]) → outward
        T(a0, pole, a1);
      }
      continue;
    }

    // D) both are poles → nothing to add
  }

  // Planar caps: add only if radii > 0
  const bool have_base_cap = (a1 > eps);
  const bool have_top_cap = (a2 > eps);

  // Base cap (x=0, −X): reversed winding
  if (have_base_cap && ring_count.front() > 1)
  {
    const Index rB = ring_start.front();
    const Index c = static_cast<Index>(V.size());
    V.emplace_back(0.0, 0.0, 0.0);
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      T(c, rB + i1, rB + i0);
    }
  }

  // Top cap (x=H, +X): CCW
  if (have_top_cap && ring_count.back() > 1)
  {
    const Index rT = ring_start.back();
    const Index c = static_cast<Index>(V.size());
    V.emplace_back(H, 0.0, 0.0);
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = static_cast<Index>(i);
      const Index i1 = static_cast<Index>((i + 1) % radial_res);
      T(c, rT + i0, rT + i1);
    }
  }
}

// -----------------------------------------------------------------------------
// Canonical tessellation only: verts in base-aligned frame with x ∈ [0, H].
// No origin policy, no per-instance scale, no custom axes.
// -----------------------------------------------------------------------------
inline bool meshifyPrimitive(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                             std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
{
  using geometry::dim;
  using geometry::DimRole;
  using geometry::ShapeType;

  V.clear();
  F.clear();

  const int rr = std::max(3, radial_res);
  const int ar = std::max(1, axial_res);

  switch (s.type)
  {
    // Cylinder dims (canonical): {height_x, radius}
    // meshifyCylinder expects: (radius, height, sides, V, F)
    case ShapeType::Cylinder:
    {
      const double r = dim(s).at(DimRole::Radius);
      const double h = dim(s).at(DimRole::Height);
      meshifyCylinder(r, h, rr, V, F);
      return true;
    }

    // Cone dims (canonical): {height_x, base_radius, top_radius}
    // meshifyConeFrustum expects: (r0, r1, h, sides, V, F)
    case ShapeType::Cone:
    {
      const double r0 = dim(s).at(DimRole::BaseRadius);
      const double r1 = dim(s).at(DimRole::TopRadius);
      const double h = dim(s).at(DimRole::Height);
      meshifyConeFrustum(r0, r1, h, rr, V, F);
      return true;
    }

    // TriangularPrism dims: {length_x, base_y, altitude_z, apex_offset_y}
    // meshifyTriangularPrism expects: (lengthX, baseY, altitudeZ, u, V, F)
    case ShapeType::TriangularPrism:
    {
      const double Lx = dim(s).at(DimRole::Height);
      const double By = dim(s).at(DimRole::Base);
      const double Az = dim(s).at(DimRole::Altitude);
      const double U = dim(s).has(DimRole::ApexOffset) ? dim(s).at(DimRole::ApexOffset) : 0.0;
      meshifyTriangularPrism(Lx, By, Az, U, V, F);
      return true;
    }

    // SphericalSegment dims: {height_x, base_radius, top_radius}
    // meshifySphericalSegment expects: (a1, a2, H, radial_res, axial_res, V, F)
    case ShapeType::SphericalSegment:
    {
      const double a1 = dim(s).at(DimRole::BaseRadius);
      const double a2 = dim(s).at(DimRole::TopRadius);
      const double H = dim(s).at(DimRole::Height);
      meshifySphericalSegment(a1, a2, H, rr, ar, V, F);
      return true;
    }

    // Box dims: {x, y, z}
    // meshifyBox expects: (Xlen, Ylen, Zlen, V, F)
    case ShapeType::Box:
    {
      const double Xlen = dim(s).at(DimRole::X);
      const double Ylen = dim(s).at(DimRole::Y);
      const double Zlen = dim(s).at(DimRole::Z);
      meshifyBox(Xlen, Ylen, Zlen, V, F);
      return true;
    }

    case ShapeType::Mesh:
    default:
      return false;  // non-primitive or unsupported here
  }
}

// -----------------------------------------------------------------------------
// Canonical tessellation + per-instance options:
//   1) origin policy translation
//   2) non-uniform instance scale (around origin)
//   3) user-provided axes (right-handed ONB from s.axes[0..2])
// -----------------------------------------------------------------------------
inline bool meshifyPrimitiveApplied(const geometry::Shape& s, std::vector<Eigen::Vector3d>& V,
                                    std::vector<TriangleMesh::Face>& F, int radial_res = 64, int axial_res = 16)
{
  if (!meshifyPrimitive(s, V, F, radial_res, axial_res))
    return false;

  // 1) Apply origin policy (moves vertices so (0,0,0) matches the policy)
  // NOTE: applyOriginPolicyMesh must interpret BaseCenter along X.
  applyOriginPolicyMesh(s, V, F);

  // 2) Apply per-instance non-uniform scale around origin
  if (s.scale != Eigen::Vector3d(1.0, 1.0, 1.0))
  {
    for (auto& p : V)
      p = p.cwiseProduct(s.scale);
  }

  // 3) Apply axes per canonical convention (primary=X, refs=Y/Z).
  //    buildCanonicalAxes() already routes through getShapePrimaryAxis/U/V
  //    and produces a robust right-handed ONB with fallbacks.
  {
    Eigen::Vector3d Xc, Yc, Zc;
    buildCanonicalAxes(s, Xc, Yc, Zc, 1e-9);

    Eigen::Matrix3d R;
    R.col(0) = Xc;  // canonical X = primary (Normal/Symmetry/Extrusion or authored X)
    R.col(1) = Yc;  // canonical Y = in-plane/reference U
    R.col(2) = Zc;  // canonical Z = in-plane/reference V

    for (auto& p : V)
      p = R * p;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Stacked Shape meshing (primitives only), with cap stripping along X and
// height computed from *scale.x()*.
// -----------------------------------------------------------------------------
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

  for (int si = 0; si < static_cast<int>(stacked.shapes.size()); ++si)
  {
    const auto& entry = stacked.shapes[si];
    const auto& s = entry.shape;

    // Skip inline Mesh for now (or append external mesh directly if desired).
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

    // 1) Canonical tessellation
    std::vector<Eigen::Vector3d> Vseg;
    std::vector<Face> Fseg;

    const int axial_res = (s.type == ShapeType::Cylinder || s.type == ShapeType::Cone) ?
                              1 :
                              std::max(1, axial_res_for_spherical_segments);

    if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial_res))
      return false;

    // 2) Apply per-instance non-uniform scale in canonical frame
    const Eigen::Vector3d sc = s.scale;
    if (sc != Eigen::Vector3d(1.0, 1.0, 1.0))
    {
      for (auto& p : Vseg)
        p = p.cwiseProduct(sc);
    }

    // Height along X after scale
    double seg_H = 0.0;
    switch (s.type)
    {
      case ShapeType::Cylinder:  // {height_x, radius}
        seg_H = dim(s).at(DimRole::Height) * sc.x();
        break;

      case ShapeType::Cone:  // {height_x, base_radius, top_radius}
        seg_H = dim(s).at(DimRole::Height) * sc.x();
        break;

      case ShapeType::SphericalSegment:  // {height_x, base_radius, top_radius}
        seg_H = dim(s).at(DimRole::Height) * sc.x();
        break;

      case ShapeType::Box:  // {x, y, z}
        seg_H = dim(s).at(DimRole::Height) * sc.x();
        break;

      case ShapeType::TriangularPrism:  // {length_x, base_y, altitude_z, apex_offset_y}
        seg_H = dim(s).at(DimRole::Height) * sc.x();
        break;

      default:
        seg_H = 0.0;
        break;
    }

    // 3) Rotate into the shape’s canonical axes (Primary = Xc, U = Yc, V = Zc).
    Eigen::Vector3d Xc, Yc, Zc;
    buildCanonicalAxes(s, Xc, Yc, Zc, 1e-9);  // uses getShapePrimaryAxis/U/V internally
    Eigen::Matrix3d R_shape;
    R_shape.col(0) = Xc;
    R_shape.col(1) = Yc;
    R_shape.col(2) = Zc;
    for (auto& p : Vseg)
      p = R_shape * p;

    // 4) Strip end caps *along the primary axis* using the generic helper
    const bool is_first = (si == 0);
    const bool is_last = (si == static_cast<int>(stacked.shapes.size()) - 1);
    if (!is_first)
      stripCapAtPlaneAlong(Vseg, Fseg, Xc, 0.0, cap_tol);
    if (!is_last)
      stripCapAtPlaneAlong(Vseg, Fseg, Xc, seg_H, cap_tol);

    // 5) Apply the entry’s SE(3) pose in the stacked frame
    const Eigen::Matrix3d R_entry = entry.base_transform.linear();
    const Eigen::Vector3d t_entry = entry.base_transform.translation();
    for (auto& p : Vseg)
      p = R_entry * p + t_entry;

    // Append into fused buffers (already in stacked frame coords).
    appendMesh(Vseg, Fseg, V_out, F_out);
  }

  if (V_out.empty() || F_out.empty())
    return false;

  // 6) Weld coincident vertices across segments
  weldVertices(V_out, F_out, weld_tol);

  // No global caps / axis enforcement / origin policy here.
  // Result is expressed directly in the StackedShape frame.
  return true;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_SHAPE_H_
