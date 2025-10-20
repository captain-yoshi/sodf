#include <sodf/physics/fluid_domain_shape.h>

#include <cmath>
#include <PolynomialRoots.hh>

#include <sodf/geometry/shape.h>

namespace sodf {
namespace physics {

double getFillHeight(const FluidBoxShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;
  return volume / (shape.length_ * shape.width_);
}

double getFillHeight(const FluidCylinderShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;
  return volume / (M_PI * shape.radius_ * shape.radius_);
}

double getFillHeight(const FluidConeShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;

  double r1 = shape.base_radius_;
  double r2 = shape.top_radius_;
  double H = shape.getMaxFillHeight();

  double k = (r2 - r1) / H;

  // Coefficients for V(h) = a h^3 + b h^2 + c h
  double a = (M_PI / 3.0) * k * k;
  double b = M_PI * r1 * k;
  double c = M_PI * r1 * r1;
  double d = -volume;

  double roots[5] = {};
  PolynomialRoots::Cubic csolve(a, b, c, d);
  auto roots_size = csolve.getPositiveRoots(roots);
  if (roots_size == 0)
    return 0.0;
  return roots[0];
}

double getFillHeight(const FluidSphericalSegmentShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;

  double a1 = shape.base_radius_;  // base radius at y=0
  double a2 = shape.top_radius_;   // top radius at y=H
  double H = shape.getMaxFillHeight();

  // Compute z0 (distance from base to sphere center)
  double z0 = (a2 * a2 - a1 * a1 + H * H) / (2 * H);
  double r = std::sqrt(a1 * a1 + z0 * z0);

  // Lambda for segment volume up to height h
  auto segment_volume = [&](double h) {
    double ah2 = r * r - (z0 - h) * (z0 - h);
    double ah = (ah2 > 0) ? std::sqrt(ah2) : 0.0;
    return M_PI * h / 6.0 * (3 * a1 * a1 + 3 * ah * ah + h * h);
  };

  // Binary search for h
  double low = 0.0, high = H, mid = 0.0;
  for (int iter = 0; iter < 100; ++iter)
  {
    mid = 0.5 * (low + high);
    double v = segment_volume(mid);
    if (std::abs(v - volume) < 1e-09)  // adjust tolerance as needed
      break;
    if (v < volume)
      low = mid;
    else
      high = mid;
  }
  return mid;
}

double getFillVolume(const FluidBoxShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;
  return shape.length_ * shape.width_ * height;
}

double getFillVolume(const FluidCylinderShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;
  return M_PI * shape.radius_ * shape.radius_ * height;
}

double getFillVolume(const FluidConeShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;

  double H = shape.getMaxFillHeight();
  double r_start = shape.base_radius_;
  double r_end = shape.base_radius_ + (shape.top_radius_ - shape.base_radius_) * (height / H);

  // Frustum volume formula
  double volume = (M_PI * height / 3.0) * (r_start * r_start + r_start * r_end + r_end * r_end);

  return volume;
}

double getFillVolume(const FluidSphericalSegmentShape& shape, double height)
{
  // Height is always positive, meaning "distance away from the reference point"
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;

  double a1 = shape.base_radius_;  // base (at height 0)
  double a2 = shape.top_radius_;   // top (at height H)
  double H = shape.getMaxFillHeight();

  // Step 1: Compute z0 (distance from base to sphere center)
  double z0 = (a2 * a2 - a1 * a1 + H * H) / (2 * H);

  // Step 2: Compute sphere radius
  double r = std::sqrt(a1 * a1 + z0 * z0);

  // Step 3: Compute cross-section radius at fill height
  double ah2 = r * r - (z0 - height) * (z0 - height);
  double ah = (ah2 > 0) ? std::sqrt(ah2) : 0.0;

  // Step 4: Plug into segment formula
  return M_PI * height / 6.0 * (3 * a1 * a1 + 3 * ah * ah + height * height);
}

// FluidBoxShape

FluidBoxShape::FluidBoxShape(double width, double length, double height)
  : width_(width), length_(length), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidBoxShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidBoxShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidCylinderShape

FluidCylinderShape::FluidCylinderShape(double radius, double height) : radius_(radius), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidCylinderShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidCylinderShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidConeShape

FluidConeShape::FluidConeShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidConeShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidConeShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidSphericalSegmentShape

FluidSphericalSegmentShape::FluidSphericalSegmentShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
{
  if (!geometry::isValidSegment(base_radius, top_radius, height))
  {
    std::ostringstream oss;
    oss << "Incompatible spherical segment dimensions: "
        << "base_radius = " << base_radius << ", "
        << "top_radius = " << top_radius << ", "
        << "height = " << height << ". "
        << "No valid sphere exists for these values.";
    throw std::invalid_argument(oss.str());
  }

  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidSphericalSegmentShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidSphericalSegmentShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

FluidSphericalSegmentShape fromBaseTop(double r1, double r2)
{
  double h = geometry::inferSegmentHeightFromRadii(r1, r2);
  return FluidSphericalSegmentShape(r1, r2, h);
}

FluidSphericalSegmentShape fromBaseHeight(double r1, double h)
{
  double r2 = geometry::inferTopRadiusFromHeight(r1, h);
  return FluidSphericalSegmentShape(r1, r2, h);
}

FluidSphericalSegmentShape fromBaseSphereRadius(double r1, double R)
{
  if (R <= r1)
    throw std::invalid_argument("Sphere radius must be greater than base radius");

  double theta = std::asin(r1 / R);
  double h = R * std::cos(theta);
  double r2 = R * std::sin(theta + h / R);  // or recompute more directly

  return FluidSphericalSegmentShape(r1, r2, h);
}

double signedTetraVolume(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                         const Eigen::Vector3d& d)
{
  return ((b - a).cross(c - a)).dot(d - a) / 6.0;
}

Eigen::Matrix3d FluidConvexMeshShape::makeWorldToGravity(const Eigen::Vector3d& g_world_in)
{
  Eigen::Vector3d z = g_world_in.normalized();  // target +Z
  Eigen::Vector3d tmp = (std::abs(z.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d x = (tmp.cross(z)).normalized();
  Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R_g_to_world;
  R_g_to_world.col(0) = x;
  R_g_to_world.col(1) = y;
  R_g_to_world.col(2) = z;
  return R_g_to_world.transpose();  // world→g
}

double clippedTetraVolumeZleq(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                              const Eigen::Vector3d& p3, double h, double epsilon)
{
  // Intersection of segment AB with plane z = h.
  // Only called on crossing edges; clamp t just in case.
  auto I = [h, epsilon](const Eigen::Vector3d& A, const Eigen::Vector3d& B) -> Eigen::Vector3d {
    const double dz = (B.z() - A.z());
    if (std::abs(dz) < epsilon)
      return A;  // nearly horizontal; caller won't rely on this edge crossing
    double t = (h - A.z()) / dz;
    if (t < 0.0)
      t = 0.0;
    if (t > 1.0)
      t = 1.0;
    return A + t * (B - A);
  };

  // Classify vertices
  std::array<Eigen::Vector3d, 4> P = { p0, p1, p2, p3 };
  std::vector<int> below;
  below.reserve(4);
  std::vector<int> above;
  above.reserve(4);
  for (int i = 0; i < 4; ++i)
  {
    if (P[i].z() <= h + epsilon)
      below.push_back(i);
    else
      above.push_back(i);
  }

  // Trivial cases
  if (below.size() == 4)
    return std::abs(signedTetraVolume(p0, p1, p2, p3));
  if (below.empty())
    return 0.0;

  // 1-below / 3-above: the kept part is a small tetra with apex at the single below vertex
  if (below.size() == 1)
  {
    const int ib = below[0];
    const int a = above[0], b = above[1], c = above[2];
    const Eigen::Vector3d v0 = P[ib];
    const Eigen::Vector3d v1 = I(P[ib], P[a]);
    const Eigen::Vector3d v2 = I(P[ib], P[b]);
    const Eigen::Vector3d v3 = I(P[ib], P[c]);
    return std::abs(signedTetraVolume(v0, v1, v2, v3));
  }

  // 3-below / 1-above: kept volume = full tetra − small tetra above the plane
  if (below.size() == 3)
  {
    const int ia = above[0];  // the single vertex above h
    const int b0 = below[0], b1 = below[1], b2 = below[2];

    // intersections from the above vertex down to each below vertex
    const Eigen::Vector3d q0 = I(P[ia], P[b0]);
    const Eigen::Vector3d q1 = I(P[ia], P[b1]);
    const Eigen::Vector3d q2 = I(P[ia], P[b2]);

    const double v_full = std::abs(signedTetraVolume(p0, p1, p2, p3));
    const double v_cap = std::abs(signedTetraVolume(P[ia], q0, q1, q2));
    const double kept = v_full - v_cap;
    return (kept > 0.0) ? kept : 0.0;
  }

  // 2-below / 2-above: hexahedron → decompose into 4 tets
  {
    const int i0 = below[0], i1 = below[1];
    const int j0 = above[0], j1 = above[1];

    const Eigen::Vector3d A0 = P[i0];
    const Eigen::Vector3d A1 = P[i1];

    // crossing edges
    const Eigen::Vector3d L0 = I(P[i0], P[j0]);  // i0–j0
    const Eigen::Vector3d L1 = I(P[i0], P[j1]);  // i0–j1
    const Eigen::Vector3d U0 = I(P[i1], P[j0]);  // i1–j0
    const Eigen::Vector3d U1 = I(P[i1], P[j1]);  // i1–j1

    double v = 0.0;
    v += std::abs(signedTetraVolume(A0, A1, L0, U0));
    v += std::abs(signedTetraVolume(A0, L0, L1, U0));
    v += std::abs(signedTetraVolume(A0, L1, U1, U0));
    v += std::abs(signedTetraVolume(A0, A1, U0, U1));
    return v;
  }
}

double FluidConvexMeshShape::computeMaxFillHeightAt(const std::vector<Eigen::Vector3d>& verts_w,
                                                    const Eigen::Vector3d& g_world, const Eigen::Vector3d& base_world)
{
  if (verts_w.empty())
    return 0.0;
  Eigen::Matrix3d R = makeWorldToGravity(g_world);
  const double base_z = (R * base_world).z();

  double zmax = -std::numeric_limits<double>::infinity();
  for (const auto& v : verts_w)
  {
    const double z = (R * v).z() - base_z;  // height above the chosen base plane
    if (z > zmax)
      zmax = z;
  }
  if (!std::isfinite(zmax))
    return 0.0;
  return std::max(0.0, zmax);
}

// --------------------------- constructors ---------------------------

FluidConvexMeshShape::FluidConvexMeshShape(const std::vector<Eigen::Vector3d>& vertices_world,
                                           const std::vector<Tri>& tris, const Eigen::Vector3d& gravity_world,
                                           const Eigen::Vector3d& base_plane_point_world)
  : DomainShape(computeMaxFillHeightAt(vertices_world, gravity_world, base_plane_point_world))
  , verts_local_(vertices_world)
  ,  // treat provided world verts as initial "local"
  tris_(tris)
  , T_wo_(Eigen::Isometry3d::Identity())
  , g_world_(gravity_world)
  , p_base_world_(base_plane_point_world)
{
  if (verts_local_.size() < 4 || tris_.empty())
    throw std::invalid_argument("FluidConvexMeshShape: need at least 4 vertices and some triangles");

  // Interior (local) as centroid
  interior_local_.setZero();
  for (const auto& v : verts_local_)
    interior_local_ += v;
  interior_local_ /= static_cast<double>(verts_local_.size());

  // Build local tets (indices stable forever in local space)
  tetrahedralizeLocal_();

  // Capacity (rigid-invariant) from local tets
  computeCapacityFromLocal_();

  // Build gravity-space cache & max_fill_height_ from current state
  updateGravitySpace_();
  // (max_fill_height_ already set by DomainShape base; updateGravitySpace_ recomputes it consistently)
}

FluidConvexMeshShape::FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> tris_local)
  : DomainShape(0.0), verts_local_(std::move(vertices_local)), tris_(std::move(tris_local))
{
  if (verts_local_.size() < 4 || tris_.empty())
    throw std::invalid_argument("FluidConvexMeshShape: need at least 4 vertices and some triangles");

  interior_local_.setZero();
  for (const auto& v : verts_local_)
    interior_local_ += v;
  interior_local_ /= static_cast<double>(verts_local_.size());

  tetrahedralizeLocal_();
  computeCapacityFromLocal_();
  // Height will be computed when user calls setState(...) → updateGravitySpace_()
}

// --------------------------- preprocessing ---------------------------

void FluidConvexMeshShape::tetrahedralizeLocal_()
{
  tets_local_.clear();
  tets_local_.reserve(tris_.size());
  // Interior will be appended conceptually as a virtual vertex at index N
  const int N = static_cast<int>(verts_local_.size());
  for (const auto& f : tris_)
  {
    tets_local_.push_back(Tet{ f.a, f.b, f.c, N });
  }
}

void FluidConvexMeshShape::computeCapacityFromLocal_()
{
  // Sum abs volumes of local tets (interior + each face)
  const int N = static_cast<int>(verts_local_.size());
  double total = 0.0;
  for (const auto& t : tets_local_)
  {
    const Eigen::Vector3d& a = verts_local_[t.a];
    const Eigen::Vector3d& b = verts_local_[t.b];
    const Eigen::Vector3d& c = verts_local_[t.c];
    const Eigen::Vector3d d = interior_local_;  // implicit interior
    total += std::abs(signedTetraVolume(a, b, c, d));
  }
  max_fill_volume_ = total;
}

void FluidConvexMeshShape::updateGravitySpace_()
{
  // Build world→g transform
  R_world_to_g_ = makeWorldToGravity(g_world_);
  const Eigen::Vector3d base_g = R_world_to_g_ * p_base_world_;

  // Transform all verts into world then gravity, subtract base z so base plane → z=0
  const int N = static_cast<int>(verts_local_.size());
  verts_g_.resize(N + 1);  // +1 for interior

  for (int i = 0; i < N; ++i)
  {
    const Eigen::Vector3d v_world = T_wo_ * verts_local_[i];
    Eigen::Vector3d v_g = R_world_to_g_ * v_world;
    v_g.z() -= base_g.z();
    verts_g_[i] = v_g;
  }
  // Interior
  const Eigen::Vector3d c_world = T_wo_ * interior_local_;
  Eigen::Vector3d c_g = R_world_to_g_ * c_world;
  c_g.z() -= base_g.z();
  verts_g_[N] = c_g;

  // Recompute z extents and height
  z_min_ = std::numeric_limits<double>::infinity();
  z_max_ = -std::numeric_limits<double>::infinity();
  for (const auto& v : verts_g_)
  {
    z_min_ = std::min(z_min_, v.z());
    z_max_ = std::max(z_max_, v.z());
  }
  if (std::abs(z_min_) < 1e-10)
    z_min_ = 0.0;
  max_fill_height_ = std::max(0.0, z_max_ - z_min_);
}

// --------------------------- queries ---------------------------

double FluidConvexMeshShape::getFillVolume(double fill_height) const
{
  if (fill_height <= 0.0)
    return 0.0;
  if (fill_height >= max_fill_height_)
    return max_fill_volume_;

  const double h = std::clamp(fill_height, 0.0, max_fill_height_);

  double vol = 0.0;
  // Clip each local tet, but using GRAVITY-SPACE verts.
  // Our local tetra list references three face verts and the interior (as last).
  const int N = static_cast<int>(verts_g_.size()) - 1;  // interior index is N
  (void)N;

  for (const auto& t : tets_local_)
  {
    const Eigen::Vector3d& a = verts_g_[t.a];
    const Eigen::Vector3d& b = verts_g_[t.b];
    const Eigen::Vector3d& c = verts_g_[t.c];
    const Eigen::Vector3d& d = verts_g_.back();  // interior is last
    vol += clippedTetraVolumeZleq(a, b, c, d, h, kEps);
  }
  if (vol < 0.0)
    vol = 0.0;
  if (vol > max_fill_volume_)
    vol = max_fill_volume_;
  return vol;
}

double FluidConvexMeshShape::getFillHeight(double fill_volume) const
{
  if (fill_volume <= 0.0)
    return 0.0;
  if (fill_volume >= max_fill_volume_)
    return max_fill_height_;

  double lo = 0.0, hi = max_fill_height_;
  for (int iter = 0; iter < 64; ++iter)
  {
    const double mid = 0.5 * (lo + hi);
    const double v = getFillVolume(mid);
    if (std::abs(v - fill_volume) <= kEps * std::max(1.0, max_fill_volume_))
      return mid;
    (v < fill_volume) ? lo = mid : hi = mid;
  }
  return 0.5 * (lo + hi);
}

// --------------------------- state setters ---------------------------

void FluidConvexMeshShape::setWorldPose(const Eigen::Isometry3d& T_wo)
{
  T_wo_ = T_wo;
  updateGravitySpace_();
}

void FluidConvexMeshShape::setGravityWorld(const Eigen::Vector3d& g_world)
{
  g_world_ = g_world.normalized();
  updateGravitySpace_();
}

void FluidConvexMeshShape::setBasePointWorld(const Eigen::Vector3d& p_world)
{
  p_base_world_ = p_world;
  updateGravitySpace_();
}

void FluidConvexMeshShape::setState(const Eigen::Isometry3d& T_wo, const Eigen::Vector3d& g_world,
                                    const Eigen::Vector3d& p_base_world)
{
  T_wo_ = T_wo;
  g_world_ = g_world.normalized();
  p_base_world_ = p_base_world;
  updateGravitySpace_();
}

}  // namespace physics
}  // namespace sodf
