#ifndef SODF_COMPONENTS_MESH_BUILDER_H_
#define SODF_COMPONENTS_MESH_BUILDER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <array>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <sodf/geometry/eigen.h>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/mesh.h>
#include <sodf/components/shape.h>

namespace sodf {
namespace components {

static inline void stripCapAtPlaneZ(std::vector<Eigen::Vector3d>& V, std::vector<geometry::TriangleMesh::Face>& F,
                                    double z_plane, double tol)
{
  auto is_cap = [&](const geometry::TriangleMesh::Face& t) {
    const double z0 = V[t[0]].z(), z1 = V[t[1]].z(), z2 = V[t[2]].z();
    return (std::abs(z0 - z_plane) <= tol) && (std::abs(z1 - z_plane) <= tol) && (std::abs(z2 - z_plane) <= tol);
  };

  std::vector<geometry::TriangleMesh::Face> keep;
  keep.reserve(F.size());
  for (const auto& tri : F)
    if (!is_cap(tri))
      keep.push_back(tri);
  F.swap(keep);
}

static inline void appendMesh(const std::vector<Eigen::Vector3d>& VB,
                              const std::vector<geometry::TriangleMesh::Face>& FB, std::vector<Eigen::Vector3d>& VA,
                              std::vector<geometry::TriangleMesh::Face>& FA)
{
  using Index = geometry::TriangleMesh::Index;
  const Index off = static_cast<Index>(VA.size());
  VA.insert(VA.end(), VB.begin(), VB.end());
  FA.reserve(FA.size() + FB.size());
  for (auto f : FB)
    FA.push_back({ static_cast<Index>(f[0] + off), static_cast<Index>(f[1] + off), static_cast<Index>(f[2] + off) });
}

static inline void addPlanarCapFanN(std::vector<Eigen::Vector3d>& V, std::vector<geometry::TriangleMesh::Face>& F,
                                    geometry::TriangleMesh::Index ring_start, int ring_size, double plane_z,
                                    bool normal_up)
{
  using Index = geometry::TriangleMesh::Index;
  if (ring_size <= 0)
    return;
  const Index c = static_cast<Index>(V.size());
  V.emplace_back(0.0, 0.0, plane_z);
  for (int i = 0; i < ring_size; ++i)
  {
    const Index i0 = static_cast<Index>(ring_start + i);
    const Index i1 = static_cast<Index>(ring_start + ((i + 1) % ring_size));
    if (normal_up)
      F.push_back({ c, i0, i1 });
    else
      F.push_back({ c, i1, i0 });
  }
}

static inline void weldVertices(std::vector<Eigen::Vector3d>& V, std::vector<geometry::TriangleMesh::Face>& F,
                                double tol)
{
  using Index = geometry::TriangleMesh::Index;

  if (tol <= 0.0)
    return;

  struct Key
  {
    long long x, y, z;
    bool operator==(const Key& o) const
    {
      return x == o.x && y == o.y && z == o.z;
    }
  };
  struct KeyHash
  {
    size_t operator()(const Key& k) const
    {
      size_t h = (size_t)k.x;
      h ^= (size_t)k.y + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
      h ^= (size_t)k.z + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
      return h;
    }
  };

  const double inv = 1.0 / tol;
  std::unordered_map<Key, Index, KeyHash> map;
  map.reserve(V.size() * 2);
  std::vector<Eigen::Vector3d> Vnew;
  Vnew.reserve(V.size());
  std::vector<Index> remap(V.size(), std::numeric_limits<Index>::max());

  for (Index i = 0; i < static_cast<Index>(V.size()); ++i)
  {
    const auto& p = V[i];
    Key k{ llround(p.x() * inv), llround(p.y() * inv), llround(p.z() * inv) };
    auto it = map.find(k);
    if (it == map.end())
    {
      Index nid = static_cast<Index>(Vnew.size());
      map.emplace(k, nid);
      Vnew.push_back(p);
      remap[i] = nid;
    }
    else
    {
      remap[i] = it->second;
    }
  }

  for (auto& t : F)
  {
    t[0] = remap[t[0]];
    t[1] = remap[t[1]];
    t[2] = remap[t[2]];
  }
  V.swap(Vnew);
}

static inline Eigen::Matrix3d orthonormal_basis(const Eigen::Vector3d& z_in, const Eigen::Vector3d& x_hint)
{
  Eigen::Vector3d z = z_in.normalized();
  Eigen::Vector3d x = (x_hint - z * (z.dot(x_hint))).normalized();
  if (!std::isfinite(x.squaredNorm()) || x.squaredNorm() < 1e-12)
  {
    Eigen::Vector3d tmp = (std::abs(z.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    x = (tmp - z * (z.dot(tmp))).normalized();
  }
  Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R;
  R << x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z();
  return R;  // columns: x, y, z (in stack/world frame)
}

static inline bool decompose_abs_twist_lift_wrt_axes(const Eigen::Isometry3d& T_abs, const Eigen::Vector3d& z_hat_in,
                                                     const Eigen::Vector3d& x_hint_in, double& yaw_out,
                                                     double& z_base_out, double tilt_tol, double xy_tol)
{
  const Eigen::Vector3d z_hat = z_hat_in.normalized();
  const Eigen::Matrix3d B = orthonormal_basis(z_hat, x_hint_in);  // cols: x_hat,y_hat,z_hat

  const Eigen::Matrix3d Rw = T_abs.linear();
  const Eigen::Vector3d tw = T_abs.translation();

  // Lateral shift must be ~0 wrt z_hat
  const double z_comp = z_hat.dot(tw);
  const Eigen::Vector3d t_perp = tw - z_hat * z_comp;
  if (t_perp.norm() > xy_tol)
  {
    std::cout << "meshifyStackedShapePrimitives: lateral shift " << t_perp.norm() << " exceeds tolerance (" << xy_tol
              << ")." << std::endl;
    return false;
  }

  // No tilt of the stack axis
  if ((Rw * z_hat - z_hat).norm() > tilt_tol)
  {
    std::cout << "meshifyStackedShapePrimitives: tilt of stack axis exceeds tolerance (" << tilt_tol << " rad)."
              << std::endl;
    return false;
  }

  // Yaw about z_hat
  const Eigen::Matrix3d Rloc = B.transpose() * Rw * B;
  yaw_out = std::atan2(Rloc(1, 0), Rloc(0, 0));
  z_base_out = z_comp;
  return true;
}

// -----------------------------------------------------------------------------
// Main function with lambdas replaced by the helpers above
// -----------------------------------------------------------------------------
inline bool meshifyStackedShapePrimitives(const components::StackedShape& stacked, int radial_res,
                                          int axial_res_for_spherical_segments, double weld_tol,
                                          std::vector<Eigen::Vector3d>& V_out,
                                          std::vector<geometry::TriangleMesh::Face>& F_out)
{
  using Index = geometry::TriangleMesh::Index;
  using Face = geometry::TriangleMesh::Face;

  // ---------------- build ----------------
  V_out.clear();
  F_out.clear();
  if (stacked.shapes.empty())
    return false;

  radial_res = std::max(radial_res, 3);
  const double cap_tol = 1e-12;

  // Tolerances derived from weld_tol
  const double xy_tol = std::max(1e-9, 10.0 * weld_tol);  // lateral shift tolerance
  const double tilt_tol = 1e-6;                           // radians

  // Canonical stack basis → world rotation (applied once at the end)
  const Eigen::Matrix3d R_axes = orthonormal_basis(stacked.axis_stack_direction, stacked.axis_stack_reference);

  // Track global bottom/top & ring starts/sizes (no assumption on order)
  double z_bottom = std::numeric_limits<double>::infinity();
  double z_top = -std::numeric_limits<double>::infinity();
  Index bottom_ring_start = std::numeric_limits<Index>::max();
  Index top_ring_start = std::numeric_limits<Index>::max();
  int bottom_ring_size = 0;
  int top_ring_size = 0;

  struct Placed
  {
    double z_base, z_top;
    size_t idx;
  };
  std::vector<Placed> placed;
  placed.reserve(stacked.shapes.size());

  bool warned_mesh_once = false;

  for (size_t si = 0; si < stacked.shapes.size(); ++si)
  {
    const auto& entry = stacked.shapes[si];
    const auto& s = entry.shape;

    // Ignore meshes in primitive fuser
    if (s.type == geometry::ShapeType::Mesh)
    {
      if (!warned_mesh_once)
      {
        std::cout << "meshifyStackedShapePrimitives: Mesh entries are ignored in the fused primitive shell."
                  << std::endl;
        warned_mesh_once = true;
      }
      continue;
    }

    // Canonical (unscaled) segment height
    double seg_H = 0.0;
    switch (s.type)
    {
      case geometry::ShapeType::Cylinder:
        seg_H = s.dimensions.at(1);
        break;  // (r,h)
      case geometry::ShapeType::Cone:
        seg_H = s.dimensions.at(2);
        break;  // (r0,r1,h)
      case geometry::ShapeType::SphericalSegment:
        seg_H = s.dimensions.at(2);
        break;  // (a1,a2,H)
      case geometry::ShapeType::Box:
        seg_H = s.dimensions.at(2);
        break;  // (W,L,H)
      default:
        break;
    }

    // Generate primitive in canonical +Z, z∈[0..seg_H]
    std::vector<Eigen::Vector3d> Vseg;
    std::vector<Face> Fseg;

    const int axial = (s.type == geometry::ShapeType::Cylinder || s.type == geometry::ShapeType::Cone) ?
                          1 :
                          axial_res_for_spherical_segments;

    if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial))
    {
      std::cout << "meshifyStackedShapePrimitives: meshifyPrimitive failed for type '"
                << geometry::shapeTypeToString(s.type) << "'" << std::endl;
      return false;
    }

    // Apply per-instance scale BEFORE cap stripping; adjust H accordingly
    const double scale_x = s.scale.x(), scale_y = s.scale.y(), scale_z = s.scale.z();
    const double seg_H_scaled = seg_H * scale_z;
    for (auto& p : Vseg)
    {
      p.x() *= scale_x;
      p.y() *= scale_y;
      p.z() *= scale_z;
    }

    // Strip local caps at scaled planes
    stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol);
    stripCapAtPlaneZ(Vseg, Fseg, seg_H_scaled, cap_tol);

    // Decompose absolute entry pose wrt stack axes
    double yaw_abs = 0.0, z_base = 0.0;
    if (!decompose_abs_twist_lift_wrt_axes(entry.base_transform, stacked.axis_stack_direction,
                                           stacked.axis_stack_reference, yaw_abs, z_base, tilt_tol, xy_tol))
    {
      std::cout << "meshifyStackedShapePrimitives: invalid entry transform for type '"
                << geometry::shapeTypeToString(s.type) << "' (idx=" << si << ")." << std::endl;
      return false;
    }

    // Place this segment in canonical frame (yaw about +Z, translate along +Z)
    const double c_yaw = std::cos(yaw_abs);
    const double s_yaw = std::sin(yaw_abs);
    Eigen::Matrix2d Rz;
    Rz << c_yaw, -s_yaw, s_yaw, c_yaw;

    for (auto& p : Vseg)
    {
      Eigen::Vector2d xy = Rz * Eigen::Vector2d(p.x(), p.y());
      p.x() = xy.x();
      p.y() = xy.y();
      p.z() += z_base;
    }

    // Determine ring size & top ring start for this primitive
    int ring_size_this = 0;
    Index top_ring_start_local = 0;
    switch (s.type)
    {
      case geometry::ShapeType::Cylinder:
      case geometry::ShapeType::Cone:
        ring_size_this = radial_res;
        top_ring_start_local = static_cast<Index>(radial_res);
        break;
      case geometry::ShapeType::SphericalSegment:
        ring_size_this = radial_res;
        top_ring_start_local = static_cast<Index>(axial * radial_res);
        break;
      case geometry::ShapeType::Box:
        ring_size_this = 4;                            // bottom has 4 verts (0..3)
        top_ring_start_local = static_cast<Index>(4);  // top starts at 4 (4..7)
        break;
      default:
        break;
    }

    // Track global bottom/top and their ring starts + sizes
    if (z_base < z_bottom)
    {
      z_bottom = z_base;
      bottom_ring_start = static_cast<Index>(V_out.size());  // bottom ring is first ring in Vseg
      bottom_ring_size = ring_size_this;
    }
    const double seg_top = z_base + seg_H_scaled;
    if (seg_top > z_top)
    {
      z_top = seg_top;
      top_ring_start = static_cast<Index>(V_out.size()) + top_ring_start_local;
      top_ring_size = ring_size_this;
    }

    // Append to global mesh
    appendMesh(Vseg, Fseg, V_out, F_out);

    // For diagnostics
    placed.push_back({ z_base, seg_top, si });
  }

  // If nothing added (e.g., all were meshes), bail
  if (V_out.empty() || F_out.empty())
    return false;

  // Sort and warn about gaps/overlaps (diagnostic only)
  if (placed.size() >= 2)
  {
    std::sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) { return a.z_base < b.z_base; });

    const double z_tol = std::max(1e-9, 10.0 * weld_tol);
    for (size_t i = 1; i < placed.size(); ++i)
    {
      const double gap = placed[i].z_base - placed[i - 1].z_top;
      if (gap > z_tol)
      {
        std::cout << "meshifyStackedShapePrimitives: gap of " << gap << " between segments " << placed[i - 1].idx
                  << " and " << placed[i].idx << "; shell not watertight." << std::endl;
      }
      else if (gap < -z_tol)
      {
        std::cout << "meshifyStackedShapePrimitives: overlap of " << -gap << " between segments " << placed[i - 1].idx
                  << " and " << placed[i].idx << "; internal faces likely." << std::endl;
      }
    }
  }

  // Global planar caps at true bottom/top planes (use recorded ring sizes)
  if (bottom_ring_start != std::numeric_limits<Index>::max() && top_ring_start != std::numeric_limits<Index>::max() &&
      bottom_ring_size > 0 && top_ring_size > 0)
  {
    addPlanarCapFanN(V_out, F_out, bottom_ring_start, bottom_ring_size, /*plane_z=*/z_bottom, /*normal_up=*/false);
    addPlanarCapFanN(V_out, F_out, top_ring_start, top_ring_size, /*plane_z=*/z_top, /*normal_up=*/true);
  }

  // Weld then rotate to the user axes
  weldVertices(V_out, F_out, weld_tol);

  const Eigen::Matrix3d R = R_axes;
  for (auto& p : V_out)
    p = R * p;

  return !V_out.empty() && !F_out.empty();
}

/* inline bool meshifyStackedShapePrimitives(const components::StackedShape& stacked, int radial_res, */
/*                                           int axial_res_for_spherical_segments, double weld_tol, */
/*                                           std::vector<Eigen::Vector3d>& V_out, */
/*                                           std::vector<geometry::TriangleMesh::Face>& F_out) */
/* { */
/*   using Index = geometry::TriangleMesh::Index; */
/*   using Face = geometry::TriangleMesh::Face; */

/*   auto stripCapAtPlaneZ = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, double z_plane, double tol) { */
/*     auto is_cap = [&](const Face& t) { */
/*       const double z0 = V[t[0]].z(), z1 = V[t[1]].z(), z2 = V[t[2]].z(); */
/*       return (std::abs(z0 - z_plane) <= tol) && (std::abs(z1 - z_plane) <= tol) && (std::abs(z2 - z_plane) <= tol); */
/*     }; */
/*     std::vector<Face> keep; */
/*     keep.reserve(F.size()); */
/*     for (const auto& tri : F) */
/*       if (!is_cap(tri)) */
/*         keep.push_back(tri); */
/*     F.swap(keep); */
/*   }; */

/*   auto appendMesh = [](const std::vector<Eigen::Vector3d>& VB, const std::vector<Face>& FB, */
/*                        std::vector<Eigen::Vector3d>& VA, std::vector<Face>& FA) { */
/*     const Index off = static_cast<Index>(VA.size()); */
/*     VA.insert(VA.end(), VB.begin(), VB.end()); */
/*     FA.reserve(FA.size() + FB.size()); */
/*     for (auto f : FB) */
/*       FA.push_back({ static_cast<Index>(f[0] + off), static_cast<Index>(f[1] + off), static_cast<Index>(f[2] + off) }); */
/*   }; */

/*   // generalized: cap fan for an N-vertex ring */
/*   auto addPlanarCapFanN = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, Index ring_start, int ring_size, */
/*                              double plane_z, bool normal_up) { */
/*     if (ring_size <= 0) */
/*       return; */
/*     const Index c = static_cast<Index>(V.size()); */
/*     V.emplace_back(0.0, 0.0, plane_z); */
/*     for (int i = 0; i < ring_size; ++i) */
/*     { */
/*       const Index i0 = static_cast<Index>(ring_start + i); */
/*       const Index i1 = static_cast<Index>(ring_start + ((i + 1) % ring_size)); */
/*       if (normal_up) */
/*         F.push_back({ c, i0, i1 }); */
/*       else */
/*         F.push_back({ c, i1, i0 }); */
/*     } */
/*   }; */

/*   auto weldVertices = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, double tol) { */
/*     if (tol <= 0.0) */
/*       return; */
/*     struct Key */
/*     { */
/*       long long x, y, z; */
/*       bool operator==(const Key& o) const */
/*       { */
/*         return x == o.x && y == o.y && z == o.z; */
/*       } */
/*     }; */
/*     struct KeyHash */
/*     { */
/*       size_t operator()(const Key& k) const */
/*       { */
/*         size_t h = (size_t)k.x; */
/*         h ^= (size_t)k.y + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2); */
/*         h ^= (size_t)k.z + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2); */
/*         return h; */
/*       } */
/*     }; */
/*     const double inv = 1.0 / tol; */
/*     std::unordered_map<Key, Index, KeyHash> map; */
/*     map.reserve(V.size() * 2); */
/*     std::vector<Eigen::Vector3d> Vnew; */
/*     Vnew.reserve(V.size()); */
/*     std::vector<Index> remap(V.size(), std::numeric_limits<Index>::max()); */
/*     for (Index i = 0; i < static_cast<Index>(V.size()); ++i) */
/*     { */
/*       const auto& p = V[i]; */
/*       Key k{ llround(p.x() * inv), llround(p.y() * inv), llround(p.z() * inv) }; */
/*       auto it = map.find(k); */
/*       if (it == map.end()) */
/*       { */
/*         Index nid = static_cast<Index>(Vnew.size()); */
/*         map.emplace(k, nid); */
/*         Vnew.push_back(p); */
/*         remap[i] = nid; */
/*       } */
/*       else */
/*       { */
/*         remap[i] = it->second; */
/*       } */
/*     } */
/*     for (auto& t : F) */
/*     { */
/*       t[0] = remap[t[0]]; */
/*       t[1] = remap[t[1]]; */
/*       t[2] = remap[t[2]]; */
/*     } */
/*     V.swap(Vnew); */
/*   }; */

/*   auto orthonormal_basis = [](const Eigen::Vector3d& z_in, const Eigen::Vector3d& x_hint) { */
/*     Eigen::Vector3d z = z_in.normalized(); */
/*     Eigen::Vector3d x = (x_hint - z * (z.dot(x_hint))).normalized(); */
/*     if (!std::isfinite(x.squaredNorm()) || x.squaredNorm() < 1e-12) */
/*     { */
/*       Eigen::Vector3d tmp = (std::abs(z.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX(); */
/*       x = (tmp - z * (z.dot(tmp))).normalized(); */
/*     } */
/*     Eigen::Vector3d y = z.cross(x); */
/*     Eigen::Matrix3d R; */
/*     R << x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z(); */
/*     return R;  // columns: x, y, z (in stack/world frame) */
/*   }; */

/*   // Decompose absolute pose wrt arbitrary stack axes. */
/*   auto decompose_abs_twist_lift_wrt_axes = [&](const Eigen::Isometry3d& T_abs, const Eigen::Vector3d& z_hat_in, */
/*                                                const Eigen::Vector3d& x_hint_in, double& yaw_out, double& z_base_out, */
/*                                                double tilt_tol, double xy_tol) -> bool { */
/*     const Eigen::Vector3d z_hat = z_hat_in.normalized(); */
/*     const Eigen::Matrix3d B = orthonormal_basis(z_hat, x_hint_in);  // cols: x_hat,y_hat,z_hat */

/*     const Eigen::Matrix3d Rw = T_abs.linear(); */
/*     const Eigen::Vector3d tw = T_abs.translation(); */

/*     // Lateral shift must be ~0 wrt z_hat */
/*     const double z_comp = z_hat.dot(tw); */
/*     const Eigen::Vector3d t_perp = tw - z_hat * z_comp; */
/*     if (t_perp.norm() > xy_tol) */
/*     { */
/*       std::cout << "meshifyStackedShapePrimitives: lateral shift " << t_perp.norm() << " exceeds tolerance (" << xy_tol */
/*                 << ")." << std::endl; */
/*       return false; */
/*     } */

/*     // No tilt of the stack axis */
/*     if ((Rw * z_hat - z_hat).norm() > tilt_tol) */
/*     { */
/*       std::cout << "meshifyStackedShapePrimitives: tilt of stack axis exceeds tolerance (" << tilt_tol << " rad)." */
/*                 << std::endl; */
/*       return false; */
/*     } */

/*     // Yaw about z_hat */
/*     const Eigen::Matrix3d Rloc = B.transpose() * Rw * B; */
/*     yaw_out = std::atan2(Rloc(1, 0), Rloc(0, 0)); */
/*     z_base_out = z_comp; */
/*     return true; */
/*   }; */

/*   // ---------------- build ---------------- */
/*   V_out.clear(); */
/*   F_out.clear(); */
/*   if (stacked.shapes.empty()) */
/*     return false; */

/*   radial_res = std::max(radial_res, 3); */
/*   const double cap_tol = 1e-12; */

/*   // Tolerances derived from weld_tol */
/*   const double xy_tol = std::max(1e-9, 10.0 * weld_tol);  // lateral shift tolerance */
/*   const double tilt_tol = 1e-6;                           // radians */

/*   // Canonical stack basis → world rotation (applied once at the end) */
/*   const Eigen::Matrix3d R_axes = orthonormal_basis(stacked.axis_stack_direction, stacked.axis_stack_reference); */

/*   // Track global bottom/top & ring starts/sizes (no assumption on order) */
/*   double z_bottom = std::numeric_limits<double>::infinity(); */
/*   double z_top = -std::numeric_limits<double>::infinity(); */
/*   Index bottom_ring_start = std::numeric_limits<Index>::max(); */
/*   Index top_ring_start = std::numeric_limits<Index>::max(); */
/*   int bottom_ring_size = 0; */
/*   int top_ring_size = 0; */

/*   // For gap/overlap diagnostics */
/*   struct Placed */
/*   { */
/*     double z_base, z_top; */
/*     size_t idx; */
/*   }; */
/*   std::vector<Placed> placed; */
/*   placed.reserve(stacked.shapes.size()); */

/*   bool warned_mesh_once = false; */

/*   for (size_t si = 0; si < stacked.shapes.size(); ++si) */
/*   { */
/*     const auto& entry = stacked.shapes[si]; */
/*     const auto& s = entry.shape; */

/*     // Ignore meshes in primitive fuser */
/*     if (s.type == geometry::ShapeType::Mesh) */
/*     { */
/*       if (!warned_mesh_once) */
/*       { */
/*         std::cout << "meshifyStackedShapePrimitives: Mesh entries are ignored in the fused primitive shell." */
/*                   << std::endl; */
/*         warned_mesh_once = true; */
/*       } */
/*       continue; */
/*     } */

/*     // Canonical (unscaled) segment height */
/*     double seg_H = 0.0; */
/*     switch (s.type) */
/*     { */
/*       case geometry::ShapeType::Cylinder: */
/*         seg_H = s.dimensions.at(1); */
/*         break;  // (r,h) */
/*       case geometry::ShapeType::Cone: */
/*         seg_H = s.dimensions.at(2); */
/*         break;  // (r0,r1,h) */
/*       case geometry::ShapeType::SphericalSegment: */
/*         seg_H = s.dimensions.at(2); */
/*         break;  // (a1,a2,H) */
/*       case geometry::ShapeType::Box: */
/*         seg_H = s.dimensions.at(2); */
/*         break;  // (W,L,H) with +Z height */
/*       default: */
/*         break; */
/*     } */

/*     // Generate primitive in canonical +Z, z∈[0..seg_H] */
/*     std::vector<Eigen::Vector3d> Vseg; */
/*     std::vector<Face> Fseg; */

/*     int axial = (s.type == geometry::ShapeType::Cylinder || s.type == geometry::ShapeType::Cone) ? */
/*                     1 : */
/*                     axial_res_for_spherical_segments; */

/*     if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial)) */
/*     { */
/*       std::cout << "meshifyStackedShapePrimitives: meshifyPrimitive failed for type '" */
/*                 << geometry::shapeTypeToString(s.type) << "'" << std::endl; */
/*       return false; */
/*     } */

/*     // Apply per-instance scale BEFORE cap stripping; adjust H accordingly */
/*     const double scale_x = s.scale.x(), scale_y = s.scale.y(), scale_z = s.scale.z(); */
/*     const double seg_H_scaled = seg_H * scale_z; */
/*     for (auto& p : Vseg) */
/*     { */
/*       p.x() *= scale_x; */
/*       p.y() *= scale_y; */
/*       p.z() *= scale_z; */
/*     } */

/*     // Strip local caps at scaled planes */
/*     stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol); */
/*     stripCapAtPlaneZ(Vseg, Fseg, seg_H_scaled, cap_tol); */

/*     // Decompose absolute entry pose wrt stack axes */
/*     double yaw_abs = 0.0, z_base = 0.0; */
/*     if (!decompose_abs_twist_lift_wrt_axes(entry.base_transform,  // absolute in stack frame */
/*                                            stacked.axis_stack_direction, stacked.axis_stack_reference, yaw_abs, z_base, */
/*                                            tilt_tol, xy_tol)) */
/*     { */
/*       std::cout << "meshifyStackedShapePrimitives: invalid entry transform for type '" */
/*                 << geometry::shapeTypeToString(s.type) << "' (idx=" << si << ")." << std::endl; */
/*       return false; */
/*     } */

/*     // Place this segment in canonical frame (yaw about +Z, translate along +Z) */
/*     const double c_yaw = std::cos(yaw_abs); */
/*     const double s_yaw = std::sin(yaw_abs); */
/*     Eigen::Matrix2d Rz; */
/*     Rz << c_yaw, -s_yaw, s_yaw, c_yaw; */

/*     for (auto& p : Vseg) */
/*     { */
/*       Eigen::Vector2d xy = Rz * Eigen::Vector2d(p.x(), p.y()); */
/*       p.x() = xy.x(); */
/*       p.y() = xy.y(); */
/*       p.z() += z_base; */
/*     } */

/*     // Determine ring size & top ring start for this primitive */
/*     int ring_size_this = 0; */
/*     Index top_ring_start_local = 0; */
/*     switch (s.type) */
/*     { */
/*       case geometry::ShapeType::Cylinder: */
/*       case geometry::ShapeType::Cone: */
/*         ring_size_this = radial_res; */
/*         top_ring_start_local = static_cast<Index>(radial_res); */
/*         break; */
/*       case geometry::ShapeType::SphericalSegment: */
/*         ring_size_this = radial_res; */
/*         top_ring_start_local = static_cast<Index>(axial * radial_res); */
/*         break; */
/*       case geometry::ShapeType::Box: */
/*         ring_size_this = 4;                            // bottom has 4 verts (0..3) */
/*         top_ring_start_local = static_cast<Index>(4);  // top starts at 4 (4..7) */
/*         break; */
/*       default: */
/*         break; */
/*     } */

/*     // Track global bottom/top and their ring starts + sizes */
/*     if (z_base < z_bottom) */
/*     { */
/*       z_bottom = z_base; */
/*       bottom_ring_start = static_cast<Index>(V_out.size());  // bottom ring is first ring in Vseg */
/*       bottom_ring_size = ring_size_this; */
/*     } */
/*     const double seg_top = z_base + seg_H_scaled; */
/*     if (seg_top > z_top) */
/*     { */
/*       z_top = seg_top; */
/*       top_ring_start = static_cast<Index>(V_out.size()) + top_ring_start_local; */
/*       top_ring_size = ring_size_this; */
/*     } */

/*     // Append to global mesh */
/*     appendMesh(Vseg, Fseg, V_out, F_out); */

/*     // For diagnostics */
/*     placed.push_back({ z_base, seg_top, si }); */
/*   } */

/*   // If nothing added (e.g., all were meshes), bail */
/*   if (V_out.empty() || F_out.empty()) */
/*     return false; */

/*   // Sort and warn about gaps/overlaps (diagnostic only) */
/*   if (placed.size() >= 2) */
/*   { */
/*     std::sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) { return a.z_base < b.z_base; }); */
/*     const double z_tol = std::max(1e-9, 10.0 * weld_tol); */
/*     for (size_t i = 1; i < placed.size(); ++i) */
/*     { */
/*       const double gap = placed[i].z_base - placed[i - 1].z_top; */
/*       if (gap > z_tol) */
/*       { */
/*         std::cout << "meshifyStackedShapePrimitives: gap of " << gap << " between segments " << placed[i - 1].idx */
/*                   << " and " << placed[i].idx << "; shell not watertight." << std::endl; */
/*       } */
/*       else if (gap < -z_tol) */
/*       { */
/*         std::cout << "meshifyStackedShapePrimitives: overlap of " << -gap << " between segments " << placed[i - 1].idx */
/*                   << " and " << placed[i].idx << "; internal faces likely." << std::endl; */
/*       } */
/*     } */
/*   } */

/*   // Global planar caps at true bottom/top planes (use recorded ring sizes) */
/*   if (bottom_ring_start != std::numeric_limits<Index>::max() && top_ring_start != std::numeric_limits<Index>::max() && */
/*       bottom_ring_size > 0 && top_ring_size > 0) */
/*   { */
/*     addPlanarCapFanN(V_out, F_out, bottom_ring_start, bottom_ring_size, /\*plane_z=*\/z_bottom, /\*normal_up=*\/false); */
/*     addPlanarCapFanN(V_out, F_out, top_ring_start, top_ring_size, /\*plane_z=*\/z_top, /\*normal_up=*\/true); */
/*   } */

/*   // Weld then rotate to the user axes */
/*   weldVertices(V_out, F_out, weld_tol); */

/*   const Eigen::Matrix3d R = R_axes; */
/*   for (auto& p : V_out) */
/*     p = R * p; */

/*   return !V_out.empty() && !F_out.empty(); */
/* } */

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_MESH_BUILDER_H_
