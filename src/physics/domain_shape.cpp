#include <sodf/geometry/frame.h>
#include <sodf/geometry/mesh_shape.h>
#include <sodf/physics/domain_shape.h>
#include <sodf/physics/domain_shape_conversion.h>
#include <sodf/physics/fluid_domain_shape.h>
#include <stdexcept>

namespace sodf {
namespace physics {

DomainShape::DomainShape(const geometry::StackedShape& stack, DomainType dtype)
{
  // Can analytics ever be used for this stack (in its own local frame)?
  segments_locally_axis_aligned_ = validateStackLocallyAnalytic(stack, analytic_tilt_eps_rad_, analytic_lateral_eps_m_);

  // Build analytic segments from eligible primitives
  segments_.clear();
  segments_.reserve(stack.shapes.size());

  for (const auto& entry : stack.shapes)
  {
    const geometry::Shape& s = entry.shape;

    // Prefer analytic primitive
    if (auto seg = shapeToDomainShape(s, dtype))
    {
      segments_.push_back(std::move(seg));
      continue;
    }

    // Mesh shapes don’t become analytic segments; keep (first) mesh as a tilt-aware fallback
    // TODO fuse them or throw
    if (s.type == geometry::ShapeType::Mesh && !mesh_cache_)
    {
      mesh_cache_ = shapeMeshToDomainShape(s, dtype);
    }
  }
}

DomainShape::DomainShape(const geometry::Shape& shape, DomainType dtype)
{
  // Analytic primitive, store segment
  if (auto seg = shapeToDomainShape(shape, DomainType::Fluid))
  {
    segments_.push_back(std::move(seg));
    segments_locally_axis_aligned_ = true;  // a single eligible primitive
  }
  // Mesh single shape, add directly to mesh_cache
  else if (shape.type == geometry::ShapeType::Mesh)
  {
    mesh_cache_ = shapeMeshToDomainShape(shape, dtype);
    segments_locally_axis_aligned_ = false;  // mesh is not an analytic segment
  }
  else
  {
    // Unsupported shape for now (polygonal primitives, etc.)
    segments_locally_axis_aligned_ = false;
  }
}

void DomainShape::setParallelEpsilon(double eps_rad)
{
  parallel_eps_ = std::max(1e-12, eps_rad);
}

std::vector<DomainShapeBasePtr>& DomainShape::segments()
{
  return segments_;
}
const std::vector<DomainShapeBasePtr>& DomainShape::segments() const
{
  return segments_;
}

void DomainShape::setMeshCache(DomainShapeBasePtr mesh)
{
  mesh_cache_ = std::move(mesh);
}
const DomainShapeBasePtr& DomainShape::meshCache() const
{
  return mesh_cache_;
}

bool DomainShape::setMeshCacheFromStack(const geometry::StackedShape& stack, int radial_res, int axial_res_spherical,
                                        double weld_tol)
{
  using geometry::TriangleMesh;

  std::vector<Eigen::Vector3d> V;
  std::vector<TriangleMesh::Face> Fidx;

  // Build a fused shell in the StackedShape frame
  if (!meshifyStackedShapePrimitives(stack,
                                     /*radial_res=*/std::max(3, radial_res),
                                     /*axial_res_for_spherical_segments=*/std::max(1, axial_res_spherical),
                                     /*weld_tol=*/std::max(0.0, weld_tol), V, Fidx))
  {
    return false;
  }

  // Convert TriangleMesh::Face -> FluidConvexMeshShape index type
  std::vector<physics::FluidConvexMeshShape::Tri> F;
  F.reserve(Fidx.size());
  for (const auto& t : Fidx)
  {
    F.push_back({ t[0], t[1], t[2] });
  }

  geometry::TriangleMesh m;
  m.F = F;
  m.V = V;

  // Install into the cache (tilt-aware, stateless interface)
  mesh_cache_ = std::make_shared<physics::FluidConvexMeshShape>(std::move(V), std::move(F));
  return true;
}

bool DomainShape::hasMesh() const
{
  return static_cast<bool>(mesh_cache_);
}
bool DomainShape::hasSegments() const
{
  return !segments_.empty();
}

bool DomainShape::nearlyParallelWorld(const Eigen::Vector3d& height_axis_world, const Eigen::Vector3d& g_down_world,
                                      double eps_rad)
{
  const auto up = -normalizedOr(g_down_world, Eigen::Vector3d(0, 0, -1));
  const double sin_th = normalizedOr(height_axis_world, Eigen::Vector3d(0, 0, 1)).cross(up).norm();
  return sin_th <= std::sin(std::max(1e-12, eps_rad));
}

bool DomainShape::canUseAnalyticNow(const FillEnv& env) const
{
  if (!segments_locally_axis_aligned_ || segments_.empty())
    return false;
  const Eigen::Vector3d axis_w = heightAxisWorld(env);
  return nearlyParallelWorld(axis_w, env.g_down_world, parallel_eps_);
}

double DomainShape::maxFillHeight(const std::optional<FillEnv>& /*env*/) const
{
  if (hasSegments())
    return physics::getMaxFillHeight(segments_);
  return hasMesh() ? mesh_cache_->getMaxFillHeight() : 0.0;
}

double DomainShape::maxFillVolume(const std::optional<FillEnv>& /*env*/) const
{
  if (hasSegments())
    return physics::getMaxFillVolume(segments_);
  return hasMesh() ? mesh_cache_->getMaxFillVolume() : 0.0;
}

double DomainShape::heightFromVolume(double V, const FillEnv& env, double tol)
{
  if (hasSegments() && canUseAnalyticNow(env))
    return physics::getFillHeight(segments_, V, tol);

  const auto* i = getTiltAwareIfSupported(mesh_cache_);
  if (!i)
    throw std::runtime_error("Mesh exists but is not tilt-aware (stateless)!");
  return i->getFillHeightWithEnv(V, env);
}

double DomainShape::volumeFromHeight(double h, const FillEnv& env, double tol)
{
  if (hasSegments() && canUseAnalyticNow(env))
    return physics::getFillVolume(segments_, h, tol);

  const auto* i = getTiltAwareIfSupported(mesh_cache_);
  if (!i)
    throw std::runtime_error("Mesh exists but is not tilt-aware (stateless)!");
  return i->getFillVolumeWithEnv(h, env);
}

bool DomainShape::buildFilledVolumeAtHeight(double h, std::vector<Eigen::Vector3d>& tris_world, const FillEnv& env) const
{
  const auto* i = getTiltAwareIfSupported(mesh_cache_);
  if (!i)
    return false;
  return i->buildFilledVolumeAtHeightWithEnv(h, tris_world, env);
}

bool DomainShape::buildFilledVolumeAtVolume(double V, std::vector<Eigen::Vector3d>& tris_world, const FillEnv& env) const
{
  const auto* i = getTiltAwareIfSupported(mesh_cache_);
  if (!i)
    return false;
  return i->buildFilledVolumeAtVolumeWithEnv(V, tris_world, env);
}

const TiltAwareInterface* DomainShape::getTiltAwareIfSupported(const DomainShapeBasePtr& p)
{
  return dynamic_cast<const TiltAwareInterface*>(p.get());
}

Eigen::Vector3d DomainShape::normalizedOr(const Eigen::Vector3d& v, const Eigen::Vector3d& fb)
{
  const double n = v.norm();
  return (n > 0.0 && std::isfinite(n)) ? (v / n) : fb;
}

bool DomainShape::isAnalyticEligiblePrimitive(const geometry::Shape& s)
{
  using geometry::ShapeType;
  switch (s.type)
  {
    case ShapeType::Box:
    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      return true;
    default:
      return false;  // Mesh or other primitives must go through mesh path
  }
}

bool DomainShape::validateStackLocallyAnalytic(const geometry::StackedShape& stack, double tilt_eps_rad,
                                               double lateral_eps_m)
{
  if (stack.shapes.empty())
    return false;

  const Eigen::Vector3d z_hat = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d x_hint = Eigen::Vector3d::UnitX();

  for (const auto& e : stack.shapes)
  {
    // Only allow primitives you have analytic formulas for
    if (!isAnalyticEligiblePrimitive(e.shape))  // your existing predicate
      return false;

    // Only allow pure yaw about +Z and pure lift along +Z (within tolerances)
    double yaw = 0.0, z_base = 0.0;
    if (!geometry::decompose_abs_twist_lift_wrt_axes(e.base_transform, z_hat, x_hint, yaw, z_base, tilt_eps_rad,
                                                     lateral_eps_m))
    {
      return false;
    }
  }
  return true;
}

}  // namespace physics
}  // namespace sodf
