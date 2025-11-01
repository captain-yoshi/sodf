#ifndef SODF_GEOMETRY_MESH_H_
#define SODF_GEOMETRY_MESH_H_

#include <memory>
#include <variant>
#include <vector>
#include <Eigen/Core>

namespace sodf {
namespace geometry {

struct TriangleMesh
{
  using Index = uint32_t;
  using Face = std::array<Index, 3>;

  std::vector<Eigen::Vector3d> V;  // vertcies
  std::vector<Face> F;             // CCW/outward
};

struct MeshRef
{
  std::string uri;  // asset identifier only (no scale)
};

using TriangleMeshPtr = std::shared_ptr<const TriangleMesh>;
using InlineMeshPtr = std::shared_ptr<const TriangleMesh>;

using MeshSource = std::variant<std::monostate,  // no mesh
                                MeshRef,         // external mesh by URI
                                InlineMeshPtr>;  // inline mesh (heap)

static inline bool hasTriangles(const std::vector<Eigen::Vector3d>& V, const std::vector<TriangleMesh::Face>& F)
{
  return !V.empty() && !F.empty();
}

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

inline double signed_tetra_volume(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                                  const Eigen::Vector3d& d)
{
  return (a - d).dot((b - d).cross(c - d)) / 6.0;
}

// -------------------- Triangle clip (returns 0–4 verts) --------------------

inline std::vector<Eigen::Vector3d> clip_tri_halfspace(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                                       const Eigen::Vector3d& c, const Eigen::Vector3d& n,
                                                       double offset, double eps = 1e-12)
{
  const double EPSn = eps * (n.norm() + 1.0);

  auto inside = [&](const Eigen::Vector3d& p) -> bool { return (n.dot(p) - offset) <= EPSn; };

  auto intersect = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) -> Eigen::Vector3d {
    const Eigen::Vector3d d = p1 - p0;
    const double denom = n.dot(d);
    double t;
    if (std::abs(denom) <= EPSn)
      t = 0.5;  // nearly parallel
    else
      t = (offset - n.dot(p0)) / denom;
    if (t < 0.0)
      t = 0.0;
    else if (t > 1.0)
      t = 1.0;
    return p0 + t * d;
  };

  const Eigen::Vector3d P[3] = { a, b, c };
  std::vector<Eigen::Vector3d> tmp;
  tmp.reserve(4);

  for (int i = 0; i < 3; ++i)
  {
    const Eigen::Vector3d& S = P[i];
    const Eigen::Vector3d& E = P[(i + 1) % 3];
    const bool Sin = inside(S), Ein = inside(E);
    if (Sin && Ein)
      tmp.push_back(E);
    else if (Sin && !Ein)
      tmp.push_back(intersect(S, E));
    else if (!Sin && Ein)
    {
      tmp.push_back(intersect(S, E));
      tmp.push_back(E);
    }
  }

  auto nearly_eq = [&](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    return (x - y).cwiseAbs().maxCoeff() <= EPSn;
  };

  std::vector<Eigen::Vector3d> out;
  out.reserve(tmp.size());
  for (const auto& p : tmp)
    if (out.empty() || !nearly_eq(out.back(), p))
      out.push_back(p);
  if (out.size() >= 2 && nearly_eq(out.front(), out.back()))
    out.pop_back();
  return out;
}

inline std::vector<Eigen::Vector3d> clip_tri_zleq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                                  const Eigen::Vector3d& c, double h, double eps = 1e-12)
{
  return clip_tri_halfspace(a, b, c, Eigen::Vector3d::UnitZ(), h, eps);
}

// -------------------- Tetra volume under a half-space --------------------

inline double clipped_tetra_volume_halfspace(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                                             const Eigen::Vector3d& p2, const Eigen::Vector3d& p3,
                                             const Eigen::Vector3d& n, double offset, double eps = 1e-12)
{
  const double EPSn = eps * (n.norm() + 1.0);

  auto inside = [&](const Eigen::Vector3d& p) -> bool { return (n.dot(p) - offset) <= EPSn; };

  auto intersect = [&](const Eigen::Vector3d& A, const Eigen::Vector3d& B) -> Eigen::Vector3d {
    const Eigen::Vector3d d = B - A;
    const double denom = n.dot(d);
    double t;
    if (std::abs(denom) <= EPSn)
      t = 0.5;  // nearly parallel, neutral midpoint
    else
      t = (offset - n.dot(A)) / denom;
    if (t < 0.0)
      t = 0.0;
    else if (t > 1.0)
      t = 1.0;
    return A + t * d;
  };

  std::array<Eigen::Vector3d, 4> P{ p0, p1, p2, p3 };
  std::vector<int> below;
  below.reserve(4);
  std::vector<int> above;
  above.reserve(4);

  for (int i = 0; i < 4; ++i)
    (inside(P[i]) ? below : above).push_back(i);

  if (below.size() == 4)
    return std::abs(signed_tetra_volume(p0, p1, p2, p3));
  if (below.empty())
    return 0.0;

  if (below.size() == 1)
  {
    const int ib = below[0];
    const int a = above[0], b = above[1], c = above[2];
    const Eigen::Vector3d v0 = P[ib];
    const Eigen::Vector3d v1 = intersect(P[ib], P[a]);
    const Eigen::Vector3d v2 = intersect(P[ib], P[b]);
    const Eigen::Vector3d v3 = intersect(P[ib], P[c]);
    return std::abs(signed_tetra_volume(v0, v1, v2, v3));
  }

  if (below.size() == 3)
  {
    const int ia = above[0];
    const int b0 = below[0], b1 = below[1], b2 = below[2];
    const Eigen::Vector3d q0 = intersect(P[ia], P[b0]);
    const Eigen::Vector3d q1 = intersect(P[ia], P[b1]);
    const Eigen::Vector3d q2 = intersect(P[ia], P[b2]);
    const double v_full = std::abs(signed_tetra_volume(p0, p1, p2, p3));
    const double v_cap = std::abs(signed_tetra_volume(P[ia], q0, q1, q2));
    const double kept = v_full - v_cap;
    return (kept > 0.0) ? kept : 0.0;
  }

  // 2 below / 2 above → convex hexahedron, decompose into 4 tets
  {
    const int i0 = below[0], i1 = below[1];
    const int j0 = above[0], j1 = above[1];

    const Eigen::Vector3d A0 = P[i0];
    const Eigen::Vector3d A1 = P[i1];

    const Eigen::Vector3d L0 = intersect(P[i0], P[j0]);
    const Eigen::Vector3d L1 = intersect(P[i0], P[j1]);
    const Eigen::Vector3d U0 = intersect(P[i1], P[j0]);
    const Eigen::Vector3d U1 = intersect(P[i1], P[j1]);

    double v = 0.0;
    v += std::abs(signed_tetra_volume(A0, A1, L0, U0));
    v += std::abs(signed_tetra_volume(A0, L0, L1, U0));
    v += std::abs(signed_tetra_volume(A0, L1, U1, U0));
    v += std::abs(signed_tetra_volume(A0, A1, U0, U1));
    return v;
  }
}

inline double clipped_tetra_volume_zleq(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                        const Eigen::Vector3d& p3, double h, double eps = 1e-12)
{
  return clipped_tetra_volume_halfspace(p0, p1, p2, p3, Eigen::Vector3d::UnitZ(), h, eps);
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_H_
