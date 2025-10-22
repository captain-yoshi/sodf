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

#include <sodf/geometry/eigen.h>
#include <sodf/geometry/shape.h>
#include <sodf/geometry/mesh.h>
#include <sodf/components/shape.h>

namespace sodf {
namespace components {

inline bool meshifyStackedShapePrimitives(const components::StackedShape& stacked, int radial_res,
                                          int axial_res_for_spherical_segments, double weld_tol,
                                          std::vector<Eigen::Vector3d>& V_out,
                                          std::vector<geometry::TriangleMesh::Face>& F_out)
{
  using Index = geometry::TriangleMesh::Index;
  using Face = geometry::TriangleMesh::Face;

  auto stripCapAtPlaneZ = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, double z_plane, double tol) {
    auto is_cap = [&](const Face& t) {
      const double z0 = V[t[0]].z(), z1 = V[t[1]].z(), z2 = V[t[2]].z();
      return (std::abs(z0 - z_plane) <= tol) && (std::abs(z1 - z_plane) <= tol) && (std::abs(z2 - z_plane) <= tol);
    };
    std::vector<Face> keep;
    keep.reserve(F.size());
    for (const auto& tri : F)
      if (!is_cap(tri))
        keep.push_back(tri);
    F.swap(keep);
  };

  auto appendMesh = [](const std::vector<Eigen::Vector3d>& VB, const std::vector<Face>& FB,
                       std::vector<Eigen::Vector3d>& VA, std::vector<Face>& FA) {
    const Index off = Index(VA.size());
    VA.insert(VA.end(), VB.begin(), VB.end());
    FA.reserve(FA.size() + FB.size());
    for (auto f : FB)
      FA.push_back({ Index(f[0] + off), Index(f[1] + off), Index(f[2] + off) });
  };

  auto addPlanarCapFan = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, Index ring_start, int radial_res,
                            double plane_z, bool normal_up) {
    if (radial_res <= 0)
      return;
    const Index c = Index(V.size());
    V.emplace_back(0.0, 0.0, plane_z);
    for (int i = 0; i < radial_res; ++i)
    {
      const Index i0 = Index(ring_start + i);
      const Index i1 = Index(ring_start + ((i + 1) % radial_res));
      if (normal_up)
        F.push_back({ c, i0, i1 });
      else
        F.push_back({ c, i1, i0 });
    }
  };

  auto weldVertices = [](std::vector<Eigen::Vector3d>& V, std::vector<Face>& F, double tol) {
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
    for (Index i = 0; i < Index(V.size()); ++i)
    {
      const auto& p = V[i];
      Key k{ llround(p.x() * inv), llround(p.y() * inv), llround(p.z() * inv) };
      auto it = map.find(k);
      if (it == map.end())
      {
        Index nid = Index(Vnew.size());
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
  };

  auto orthonormal_basis = [](const Eigen::Vector3d& z_in, const Eigen::Vector3d& x_hint) {
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
    return R;
  };

  auto decompose_twist_lift = [](const Eigen::Isometry3d& T, double& yaw, double& dz, double tilt_tol = 1e-8,
                                 double xy_tol = 1e-12) -> bool {
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d t = T.translation();
    if (std::abs(t.x()) > xy_tol || std::abs(t.y()) > xy_tol)
      return false;  // coaxial only
    Eigen::Vector3d Rz = R * Eigen::Vector3d::UnitZ();
    if ((Rz - Eigen::Vector3d::UnitZ()).norm() > tilt_tol)
      return false;                      // no tilt
    yaw = std::atan2(R(1, 0), R(0, 0));  // yaw about +Z
    dz = t.z();                          // lift from base_{i-1} to base_i  (relative)
    return true;
  };

  V_out.clear();
  F_out.clear();
  if (stacked.shapes.empty())
    return false;

  radial_res = std::max(radial_res, 3);
  const double cap_tol = 1e-12;

  const Eigen::Matrix3d R_axes = orthonormal_basis(stacked.axis_stack_direction, stacked.axis_stack_reference);

  Index global_bottom_ring_start = std::numeric_limits<Index>::max();
  Index last_top_ring_start = std::numeric_limits<Index>::max();

  double z_cursor = 0.0;  // absolute TOP of the previous segment (for i=0, this is base0 top=0)
  double prev_H = 0.0;    // height of previous segment
  double yaw_acc = 0.0;
  double last_top_z = 0.0;

  for (size_t si = 0; si < stacked.shapes.size(); ++si)
  {
    const auto& entry = stacked.shapes[si];
    const auto& s = entry.shape;

    // segment height
    double seg_H = 0.0;
    switch (s.type)
    {
      case geometry::ShapeType::Cylinder:
        seg_H = s.dimensions.at(1);
        break;
      case geometry::ShapeType::Cone:
        seg_H = s.dimensions.at(2);
        break;
      case geometry::ShapeType::SphericalSegment:
        seg_H = s.dimensions.at(2);
        break;
      case geometry::ShapeType::Box:
        seg_H = s.dimensions.at(2);
        break;
      default:
        break;
    }

    // meshify z element [0,seg_H]
    std::vector<Eigen::Vector3d> Vseg;
    std::vector<Face> Fseg;
    int axial = (s.type == geometry::ShapeType::Cylinder || s.type == geometry::ShapeType::Cone) ?
                    1 :
                    axial_res_for_spherical_segments;
    if (!geometry::meshifyPrimitive(s, Vseg, Fseg, radial_res, axial))
      return false;

    stripCapAtPlaneZ(Vseg, Fseg, 0.0, cap_tol);
    stripCapAtPlaneZ(Vseg, Fseg, seg_H, cap_tol);

    // interpret transform as PREV_BASE -> CURR_BASE
    double yaw_step = 0.0, dz_step = 0.0;
    if (!decompose_twist_lift(entry.relative_transform, yaw_step, dz_step))
      return false;

    const double prev_base = (si == 0) ? 0.0 : (z_cursor - prev_H);
    const double z_base_this = prev_base + dz_step;  // *** key line ***
    const double yaw_this = yaw_acc + yaw_step;

    // place this segment
    const double cy = std::cos(yaw_this), sy = std::sin(yaw_this);
    Eigen::Matrix2d Rz;
    Rz << cy, -sy, sy, cy;
    for (auto& p : Vseg)
    {
      Eigen::Vector2d xy = Rz * Eigen::Vector2d(p.x(), p.y());
      p.x() = xy.x();
      p.y() = xy.y();
      p.z() += z_base_this;
    }

    // ring bookkeeping
    if (si == 0)
      global_bottom_ring_start = Index(V_out.size());
    Index top_ring_start_local =
        (s.type == geometry::ShapeType::SphericalSegment) ? Index(axial * radial_res) : Index(radial_res);
    last_top_ring_start = Index(V_out.size()) + top_ring_start_local;

    appendMesh(Vseg, Fseg, V_out, F_out);

    // advance
    prev_H = seg_H;
    z_cursor = z_base_this + seg_H;  // new top
    yaw_acc += yaw_step;
    last_top_z = z_cursor;
  }

  // global caps
  addPlanarCapFan(V_out, F_out, global_bottom_ring_start, radial_res, 0.0, false);
  addPlanarCapFan(V_out, F_out, last_top_ring_start, radial_res, last_top_z, true);

  // weld + rotate to axes
  weldVertices(V_out, F_out, weld_tol);
  for (auto& p : V_out)
    p = R_axes * p;

  return true;
}

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_MESH_BUILDER_H_
