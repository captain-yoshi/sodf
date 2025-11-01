#include <sodf/physics/fluid_domain_shape.h>
#include <sodf/geometry/mesh.h>

#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <array>
#include <unordered_set>

#include <PolynomialRoots.hh>

namespace sodf {
namespace physics {

namespace {

inline double box_fill_height(double width, double length, double V, double Vmax)
{
  if (V <= 0.0)
    return 0.0;
  if (V >= Vmax)
    return Vmax / (width * length);
  return V / (width * length);
}

inline double box_fill_volume(double width, double length, double h, double Hmax)
{
  if (h <= 0.0)
    return 0.0;
  if (h >= Hmax)
    return width * length * Hmax;
  return width * length * h;
}

inline double cyl_fill_height(double radius, double V, double Vmax)
{
  const double A = M_PI * radius * radius;
  if (V <= 0.0)
    return 0.0;
  if (V >= Vmax)
    return Vmax / A;
  return V / A;
}

inline double cyl_fill_volume(double radius, double h, double Hmax)
{
  const double A = M_PI * radius * radius;
  if (h <= 0.0)
    return 0.0;
  if (h >= Hmax)
    return A * Hmax;
  return A * h;
}

inline double frustum_fill_volume(double r0, double r1, double h, double H)
{
  if (h <= 0.0)
    return 0.0;
  if (h >= H)
    h = H;
  const double t = h / H;
  const double r = r0 + (r1 - r0) * t;
  // exact sub-frustum volume
  return (M_PI * h / 3.0) * (r0 * r0 + r0 * r + r * r);
}

inline double frustum_fill_height_cubic(double r0, double r1, double H, double V, double Vmax)
{
  if (V <= 0.0)
    return 0.0;
  if (V >= Vmax)
    return H;

  const double k = (r1 - r0) / H;
  // V(h) = π/3 (k^2 h^3 + 3 r0 k h^2 + 3 r0^2 h)
  // Put as a h^3 + b h^2 + c h + d = 0
  const double a = (M_PI / 3.0) * k * k;
  const double b = M_PI * r0 * k;
  const double c = M_PI * r0 * r0;
  const double d = -V;

  double roots[5] = {};
  PolynomialRoots::Cubic cubic(a, b, c, d);
  const int n = cubic.getPositiveRoots(roots);
  if (n == 0)
    return 0.0;
  // The lowest positive root in [0,H] is the physical one for a monotone V(h)
  double h = roots[0];
  for (int i = 1; i < n; ++i)
    if (roots[i] < h)
      h = roots[i];
  if (h < 0.0)
    h = 0.0;
  if (h > H)
    h = H;
  return h;
}

inline void spherical_segment_params(double r_base, double r_top, double H, double& z0, double& R)
{
  // distance from base plane to sphere center
  z0 = (r_top * r_top - r_base * r_base + H * H) / (2.0 * H);
  R = std::sqrt(std::max(0.0, r_base * r_base + z0 * z0));
}

inline double spherical_segment_volume_up_to(double r_base, double r_top, double H, double h)
{
  if (h <= 0.0)
    return 0.0;
  if (h >= H)
    h = H;
  double z0, R;
  spherical_segment_params(r_base, r_top, H, z0, R);

  const double ah2 = std::max(0.0, R * R - (z0 - h) * (z0 - h));
  const double ah = std::sqrt(ah2);
  return M_PI * h / 6.0 * (3.0 * r_base * r_base + 3.0 * ah * ah + h * h);
}

inline double spherical_segment_height_from_volume(double r_base, double r_top, double H, double V, double Vmax)
{
  if (V <= 0.0)
    return 0.0;
  if (V >= Vmax)
    return H;

  double lo = 0.0, hi = H, mid = 0.0;
  for (int it = 0; it < 100; ++it)
  {
    mid = 0.5 * (lo + hi);
    const double v = spherical_segment_volume_up_to(r_base, r_top, H, mid);
    if (std::abs(v - V) < 1e-9)
      break;
    (v < V) ? lo = mid : hi = mid;
  }
  return mid;
}

}  // anonymous namespace

FluidBoxShape::FluidBoxShape(double width, double length, double H) : DomainShapeBase(H), width_(width), length_(length)
{
  max_fill_volume_ = box_fill_volume(width_, length_, H, H);
}

double FluidBoxShape::getFillHeight(double V) const
{
  return box_fill_height(width_, length_, V, max_fill_volume_);
}
double FluidBoxShape::getFillVolume(double h) const
{
  return box_fill_volume(width_, length_, h, max_fill_height_);
}

FluidCylinderShape::FluidCylinderShape(double radius, double H) : DomainShapeBase(H), radius_(radius)
{
  max_fill_volume_ = cyl_fill_volume(radius_, H, H);
}

double FluidCylinderShape::getFillHeight(double V) const
{
  return cyl_fill_height(radius_, V, max_fill_volume_);
}
double FluidCylinderShape::getFillVolume(double h) const
{
  return cyl_fill_volume(radius_, h, max_fill_height_);
}

FluidConeShape::FluidConeShape(double r0, double r1, double H) : DomainShapeBase(H), base_radius_(r0), top_radius_(r1)
{
  max_fill_volume_ = frustum_fill_volume(r0, r1, H, H);
}

double FluidConeShape::getFillHeight(double V) const
{
  // Use your cubic solver (fast & accurate) instead of pure bisection
  return frustum_fill_height_cubic(base_radius_, top_radius_, max_fill_height_, V, max_fill_volume_);
}
double FluidConeShape::getFillVolume(double h) const
{
  return frustum_fill_volume(base_radius_, top_radius_, h, max_fill_height_);
}

FluidSphericalSegmentShape::FluidSphericalSegmentShape(double r_base, double r_top, double H)
  : DomainShapeBase(H), base_radius_(r_base), top_radius_(r_top)
{
  max_fill_volume_ = spherical_segment_volume_up_to(base_radius_, top_radius_, H, H);
}

double FluidSphericalSegmentShape::getFillHeight(double V) const
{
  return spherical_segment_height_from_volume(base_radius_, top_radius_, max_fill_height_, V, max_fill_volume_);
}
double FluidSphericalSegmentShape::getFillVolume(double h) const
{
  return spherical_segment_volume_up_to(base_radius_, top_radius_, max_fill_height_, h);
}

FluidConvexMeshShape::FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> triangles_local)
  : DomainShapeBase(/*max_fill_height*/ 0.0), verts_local_(std::move(vertices_local)), tris_(std::move(triangles_local))
{
  if (verts_local_.size() < 4 || tris_.empty())
    throw std::invalid_argument("FluidConvexMeshShape: need at least 4 vertices and some triangles");

  // Interior (local) as centroid
  interior_local_.setZero();
  for (const auto& v : verts_local_)
    interior_local_ += v;
  interior_local_ /= double(verts_local_.size());

  tetrahedralizeLocal_();
  computeCapacityFromLocal_();
}

double FluidConvexMeshShape::getFillHeight(double V) const
{
  // Stateless fallback: treat LOCAL +Z as world +Z, base at origin
  FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = Eigen::Isometry3d::Identity();
  env.p_base_world = Eigen::Vector3d::Zero();
  return getFillHeightWithEnv(V, env);
}

double FluidConvexMeshShape::getFillVolume(double h) const
{
  FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = Eigen::Isometry3d::Identity();
  env.p_base_world = Eigen::Vector3d::Zero();
  return getFillVolumeWithEnv(h, env);
}

// ITiltAware stateless API
// LOCAL to gravity (base plane z baked in where z=0 is base)
inline Eigen::Vector3d toG0(const Eigen::Vector3d& vL, const FillEnv& env, const FluidConvexMeshShape::GravitySpace& gs)
{
  Eigen::Vector3d vG = gs.R_wg * (env.T_world_domain * vL);
  vG.z() -= gs.base_z_g;
  return vG;
}

// gravity (shifted) to WORLD (add back base z, rotate back)
inline Eigen::Vector3d g0ToWorld(Eigen::Vector3d pG0, const FluidConvexMeshShape::GravitySpace& gs)
{
  pG0.z() += gs.base_z_g;
  return gs.R_wg.transpose() * pG0;
}

double FluidConvexMeshShape::getFillVolumeWithEnv(double h_world, const FillEnv& env) const
{
  const auto gs = makeGravitySpace_(env);

  const double h = std::clamp(h_world, 0.0, gs.Hmax());  // clip plane in the same space
  double Vsum = 0.0;

  const uint32_t N = static_cast<uint32_t>(verts_local_.size());
  for (const auto& t : tets_local_)
  {
    const Eigen::Vector3d A = (t.a == N) ? toG0(interior_local_, env, gs) : toG0(verts_local_[t.a], env, gs);
    const Eigen::Vector3d B = (t.b == N) ? toG0(interior_local_, env, gs) : toG0(verts_local_[t.b], env, gs);
    const Eigen::Vector3d C = (t.c == N) ? toG0(interior_local_, env, gs) : toG0(verts_local_[t.c], env, gs);
    const Eigen::Vector3d D = (t.d == N) ? toG0(interior_local_, env, gs) : toG0(verts_local_[t.d], env, gs);

    Vsum += geometry::clipped_tetra_volume_zleq(A, B, C, D, h, 1e-12);
  }

  if (Vsum < 0.0)
    Vsum = 0.0;
  if (Vsum > max_fill_volume_)
    Vsum = max_fill_volume_;
  return Vsum;
}

double FluidConvexMeshShape::getFillHeightWithEnv(double V, const FillEnv& env) const
{
  if (V <= 0.0)
    return 0.0;
  if (V >= max_fill_volume_)
  {
    const auto gs = makeGravitySpace_(env);
    return gs.Hmax();  // orientation-dependent height span
  }

  const auto gs = makeGravitySpace_(env);
  const double Hmax = gs.Hmax();  // [0, Hmax] since base plane z=0
  double lo = 0.0, hi = Hmax;

  for (int it = 0; it < 70; ++it)
  {
    const double mid = 0.5 * (lo + hi);
    const double Vm = getFillVolumeWithEnv(mid, env);
    if (!std::isfinite(Vm))
    {
      hi = mid;
      continue;
    }

    const double tolV = std::max(1e-15, 1e-8 * std::abs(max_fill_volume_));
    if (std::abs(Vm - V) <= tolV)
      return mid;
    (Vm < V) ? lo = mid : hi = mid;
  }
  return 0.5 * (lo + hi);
}

bool FluidConvexMeshShape::buildFilledVolumeAtHeightWithEnv(double h_world,
                                                            std::vector<Eigen::Vector3d>& tri_list_world,
                                                            const FillEnv& env) const
{
  tri_list_world.clear();

  const auto gs = makeGravitySpace_(env);
  const double H = std::clamp(h_world, 0.0, gs.Hmax());
  if (H <= 0.0)
    return false;

  constexpr double eps = 1e-12;

  auto inside = [&](double z) { return z <= H + eps; };

  auto interpOnH = [&](const Eigen::Vector3d& P, const Eigen::Vector3d& Q) -> Eigen::Vector3d {
    const double dz = (Q.z() - P.z());
    if (std::abs(dz) < eps)
      return P;  // parallel / coincident edge
    const double t = (H - P.z()) / dz;
    return P + t * (Q - P);
  };

  auto area2 = [](const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) {
    return (B - A).cross(C - A).squaredNorm();
  };

  // 1) Side walls: per-face cases, reusing original vertices
  for (const auto& f : tris_)
  {
    // Face in shifted gravity space
    const Eigen::Vector3d aG = toG0(verts_local_[f[0]], env, gs);
    const Eigen::Vector3d bG = toG0(verts_local_[f[1]], env, gs);
    const Eigen::Vector3d cG = toG0(verts_local_[f[2]], env, gs);
    const bool aIn = inside(aG.z());
    const bool bIn = inside(bG.z());
    const bool cIn = inside(cG.z());
    const int inCount = int(aIn) + int(bIn) + int(cIn);

    if (inCount == 0)
    {
      // whole face above the plane, discard
      continue;
    }

    // Source face normal in WORLD for winding check
    const Eigen::Vector3d aW_src = env.T_world_domain * verts_local_[f[0]];
    const Eigen::Vector3d bW_src = env.T_world_domain * verts_local_[f[1]];
    const Eigen::Vector3d cW_src = env.T_world_domain * verts_local_[f[2]];
    const Eigen::Vector3d nW_src = (bW_src - aW_src).cross(cW_src - aW_src);

    auto pushTriWorldMatchWinding = [&](const Eigen::Vector3d& A_g0, const Eigen::Vector3d& B_g0,
                                        const Eigen::Vector3d& C_g0) {
      Eigen::Vector3d A = g0ToWorld(A_g0, gs);
      Eigen::Vector3d B = g0ToWorld(B_g0, gs);
      Eigen::Vector3d C = g0ToWorld(C_g0, gs);

      // Cull true zeros
      if (area2(A, B, C) <= 0.0)
        return;

      // match source winding
      Eigen::Vector3d n = (B - A).cross(C - A);
      if (n.dot(nW_src) < 0.0)
        std::swap(B, C);

      tri_list_world.push_back(A);
      tri_list_world.push_back(B);
      tri_list_world.push_back(C);
    };

    if (inCount == 3)
    {
      // whole face below the plane: emit original triangle as-is
      pushTriWorldMatchWinding(aG, bG, cG);
      continue;
    }

    if (inCount == 1)
    {
      // One inside, two outside: triangle clipped to a smaller triangle
      // Find inside vertex vI and its two edges intersections on plane H
      Eigen::Vector3d vI, i0, i1;

      if (aIn)
      {
        vI = aG;
        i0 = interpOnH(aG, bG);
        i1 = interpOnH(aG, cG);
      }
      else if (bIn)
      {
        vI = bG;
        i0 = interpOnH(bG, cG);
        i1 = interpOnH(bG, aG);
      }
      else
      {  // cIn
        vI = cG;
        i0 = interpOnH(cG, aG);
        i1 = interpOnH(cG, bG);
      }

      // Emit single tri (vI, i0, i1)
      pushTriWorldMatchWinding(vI, i0, i1);
      continue;
    }

    // inCount == 2
    // Two inside, one outside: clipped polygon is a quad; emit TWO tris reusing both inside vertices.
    // Determine which is the outside vertex and build ordered quad:
    // inside vertices: v0, v1 ; intersections: k0 on (v0,out), k1 on (v1,out)
    if (!aIn)
    {
      Eigen::Vector3d v0 = bG, v1 = cG;
      Eigen::Vector3d k0 = interpOnH(bG, aG);
      Eigen::Vector3d k1 = interpOnH(cG, aG);
      // Order around the original edge (v0 -> k0 -> k1 -> v1) is consistent
      pushTriWorldMatchWinding(v0, k0, k1);
      pushTriWorldMatchWinding(v0, k1, v1);
    }
    else if (!bIn)
    {
      Eigen::Vector3d v0 = cG, v1 = aG;
      Eigen::Vector3d k0 = interpOnH(cG, bG);
      Eigen::Vector3d k1 = interpOnH(aG, bG);
      pushTriWorldMatchWinding(v0, k0, k1);
      pushTriWorldMatchWinding(v0, k1, v1);
    }
    else
    {  // !cIn
      Eigen::Vector3d v0 = aG, v1 = bG;
      Eigen::Vector3d k0 = interpOnH(aG, cG);
      Eigen::Vector3d k1 = interpOnH(bG, cG);
      pushTriWorldMatchWinding(v0, k0, k1);
      pushTriWorldMatchWinding(v0, k1, v1);
    }
  }

  // 2) Top cap (reuse your robust section polygon)
  std::vector<Eigen::Vector3d> loop_world;
  if (computeSectionPolygonAtHeightWithEnv(H, loop_world, env) && loop_world.size() >= 3)
  {
    // Ensure cap faces +up_world
    const Eigen::Vector3d up_world = -env.g_down_world.normalized();
    if (((loop_world[1] - loop_world[0]).cross(loop_world[2] - loop_world[0])).dot(up_world) < 0.0)
      std::reverse(loop_world.begin(), loop_world.end());

    // v0 fan: emit exactly N-2 triangles (no cull)
    const Eigen::Vector3d v0 = loop_world[0];
    for (size_t i = 1; i + 1 < loop_world.size(); ++i)
    {
      tri_list_world.push_back(v0);
      tri_list_world.push_back(loop_world[i]);
      tri_list_world.push_back(loop_world[i + 1]);
    }
  }

  return !tri_list_world.empty();
}

bool FluidConvexMeshShape::buildFilledVolumeAtVolumeWithEnv(double V, std::vector<Eigen::Vector3d>& tri_list_world,
                                                            const FillEnv& env) const
{
  tri_list_world.clear();

  if (V <= 0.0)
    return false;

  if (V >= max_fill_volume_)
  {
    const auto gs = makeGravitySpace_(env);
    return buildFilledVolumeAtHeightWithEnv(gs.Hmax(), tri_list_world, env);
  }

  const double h = getFillHeightWithEnv(V, env);
  return buildFilledVolumeAtHeightWithEnv(h, tri_list_world, env);
}

// ------------------ immutable mesh helpers -----------------------------------

void FluidConvexMeshShape::tetrahedralizeLocal_()
{
  tets_local_.clear();
  tets_local_.reserve(tris_.size());
  const Index N = static_cast<Index>(verts_local_.size());  // interior is N
  for (const auto& f : tris_)
  {
    tets_local_.push_back(Tet{ f[0], f[1], f[2], N });
  }
}

void FluidConvexMeshShape::computeCapacityFromLocal_()
{
  // rigid-transform invariant → compute once in local using interior fan
  const Index N = static_cast<Index>(verts_local_.size());
  double total = 0.0;
  for (const auto& t : tets_local_)
  {
    const Eigen::Vector3d& a = (t.a == N) ? interior_local_ : verts_local_[t.a];
    const Eigen::Vector3d& b = (t.b == N) ? interior_local_ : verts_local_[t.b];
    const Eigen::Vector3d& c = (t.c == N) ? interior_local_ : verts_local_[t.c];
    const Eigen::Vector3d& d = (t.d == N) ? interior_local_ : verts_local_[t.d];
    total += std::abs(geometry::signed_tetra_volume(a, b, c, d));
  }
  max_fill_volume_ = total;
}

Eigen::Matrix3d FluidConvexMeshShape::makeWorldToGravity_(const Eigen::Vector3d& g_down_world)
{
  const Eigen::Vector3d up = -g_down_world.normalized();
  Eigen::Vector3d x = up.unitOrthogonal();
  Eigen::Vector3d y = up.cross(x);
  Eigen::Matrix3d R_gw;
  R_gw.col(0) = x;
  R_gw.col(1) = y;
  R_gw.col(2) = up;
  return R_gw.transpose();
}

FluidConvexMeshShape::GravitySpace FluidConvexMeshShape::makeGravitySpace_(const FillEnv& env) const
{
  GravitySpace gs;
  gs.R_wg = makeWorldToGravity_(env.g_down_world);
  // base plane height in gravity space
  gs.base_z_g = (gs.R_wg * env.p_base_world).z();

  double zmin = +std::numeric_limits<double>::infinity();
  double zmax = -std::numeric_limits<double>::infinity();

  for (const auto& vL : verts_local_)
  {
    const Eigen::Vector3d vW = env.T_world_domain * vL;
    const Eigen::Vector3d vG = gs.R_wg * vW;  // NOTE: no subtraction of p_base_world (XY untouched)
    zmin = std::min(zmin, vG.z());
    zmax = std::max(zmax, vG.z());
  }

  gs.z_min = zmin;
  gs.z_max = zmax;
  return gs;
}

void FluidConvexMeshShape::orderConvexLoopXY(std::vector<Eigen::Vector3d>& pts) const
{
  if (pts.size() <= 2)
    return;
  Eigen::Vector2d c(0, 0);
  for (auto& p : pts)
    c += p.head<2>();
  c /= double(pts.size());
  std::sort(pts.begin(), pts.end(), [&](const Eigen::Vector3d& A, const Eigen::Vector3d& B) {
    const double a = std::atan2(A.y() - c.y(), A.x() - c.x());
    const double b = std::atan2(B.y() - c.y(), B.x() - c.x());
    return a < b;
  });
}

bool FluidConvexMeshShape::computeSectionPolygonAtHeightWithEnv(double h_world,
                                                                std::vector<Eigen::Vector3d>& poly_world,
                                                                const FillEnv& env) const
{
  poly_world.clear();

  const auto gs = makeGravitySpace_(env);
  const double Hmax = gs.Hmax();
  if (Hmax <= 0.0)
    return false;

  // Work in shifted gravity space (z=0 at base plane)
  const double h = std::clamp(h_world, 0.0, Hmax);
  constexpr double epsH = 1e-12;

  // De-dup in gravity XY (shifted space)
  struct Key
  {
    long long x, y;
    bool operator==(const Key& o) const
    {
      return x == o.x && y == o.y;
    }
  };
  struct KeyHash
  {
    size_t operator()(const Key& k) const
    {
      size_t h = (size_t)k.x;
      h ^= (size_t)k.y + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
      return h;
    }
  };
  const double snap = 1e-9, inv = 1.0 / snap;

  std::unordered_set<Key, KeyHash> seen;
  seen.reserve(tris_.size() * 2);
  std::vector<Eigen::Vector3d> loop_g0;
  loop_g0.reserve(tris_.size() * 2);

  auto pushUnique = [&](const Eigen::Vector3d& p) {
    Key k{ llround(p.x() * inv), llround(p.y() * inv) };
    if (seen.insert(k).second)
      loop_g0.push_back(p);
  };

  auto I = [&](const Eigen::Vector3d& A, const Eigen::Vector3d& B) -> Eigen::Vector3d {
    const double dz = (B.z() - A.z());
    if (std::abs(dz) < epsH)
      return A;
    const double t = (h - A.z()) / dz;
    return A + t * (B - A);
  };

  auto edgeCross = [&](const Eigen::Vector3d& P, const Eigen::Vector3d& Q) {
    const double d0 = P.z() - h, d1 = Q.z() - h;
    const bool cross = (d0 <= 0.0 && d1 >= 0.0) || (d0 >= 0.0 && d1 <= 0.0);
    if (!cross)
      return;

    if (std::abs(d0) < epsH && std::abs(d1) < epsH)
    {
      pushUnique(P);
      pushUnique(Q);
    }
    else if (std::abs(d0) < epsH)
    {
      pushUnique(P);
    }
    else if (std::abs(d1) < epsH)
    {
      pushUnique(Q);
    }
    else
    {
      pushUnique(I(P, Q));
    }
  };

  // --- Edge/plane intersections in shifted gravity space (use toG0_) ---
  for (const auto& tri : tris_)
  {
    const Eigen::Vector3d a = toG0(verts_local_[tri[0]], env, gs);
    const Eigen::Vector3d b = toG0(verts_local_[tri[1]], env, gs);
    const Eigen::Vector3d c = toG0(verts_local_[tri[2]], env, gs);
    edgeCross(a, b);
    edgeCross(b, c);
    edgeCross(c, a);
  }

  if (loop_g0.size() < 3)
    return false;

  // Order convex CCW in XY (shifted gravity space)
  orderConvexLoopXY(loop_g0);

  // Map to WORLD using the same base-z shift (no +p_base_world XY translation)
  poly_world.reserve(loop_g0.size());
  for (const auto& pG0 : loop_g0)
    poly_world.push_back(g0ToWorld(pG0, gs));

  return poly_world.size() >= 3;
}

}  // namespace physics
}  // namespace sodf
