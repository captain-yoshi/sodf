#ifndef SODF_PHYSICS_DOMAIN_SHAPE_H_
#define SODF_PHYSICS_DOMAIN_SHAPE_H_

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <functional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sodf/geometry/shape.h>
#include <sodf/physics/domain_shape_base.h>

namespace sodf {
namespace physics {

// High-level orchestrator that picks analytic fast-path vs mesh fallback
enum class DomainType
{
  Fluid /*, Solid, Granular*/
};

class DomainShape
{
public:
  explicit DomainShape(const geometry::StackedShape& stack, DomainType dtype);
  explicit DomainShape(const geometry::Shape& shape, DomainType dtype);

  DomainType type = DomainType::Fluid;
  std::string stacked_shape_id;  // optional provenance

  // Canonical height axis in the domain's *local* frame (default +X).
  // Keep a setter in case a non-standard local up is authored.
  void setHeightAxisLocal(const Eigen::Vector3d& axis_local)
  {
    height_axis_local_ = normalizedOr(axis_local, Eigen::Vector3d::UnitX());
  }

  // World axis for the current env (computed on demand).
  Eigen::Vector3d heightAxisWorld(const FillEnv& env) const
  {
    return (env.T_world_domain.linear() * height_axis_local_).normalized();
  }

  // Configure analytic-vs-tilt parallel test tolerance (radians).
  void setParallelEpsilon(double eps_rad);
  double parallelEpsilon() const
  {
    return parallel_eps_;
  }

  std::vector<DomainShapeBasePtr>& segments();
  const std::vector<DomainShapeBasePtr>& segments() const;

  void setMeshCache(DomainShapeBasePtr mesh);
  const DomainShapeBasePtr& meshCache() const;

  bool setMeshCacheFromStack(const geometry::StackedShape& stack, int radial_res = 46, int axial_res_spherical = 16,
                             double weld_tol = 1e-9);

  bool hasMesh() const;
  bool hasSegments() const;

  // --- stateless path selection (uses env each time) -------------------------
  static bool nearlyParallelWorld(const Eigen::Vector3d& height_axis_world, const Eigen::Vector3d& g_down_world,
                                  double eps_rad);

  // Local+gravity analytic readiness
  bool canUseAnalyticNow(const FillEnv& env) const;

  // --- maxima (analytics preferred; mesh uses stored capacity) ---------------
  double maxFillHeight(const std::optional<FillEnv>& env = std::nullopt) const;
  double maxFillVolume(const std::optional<FillEnv>& env = std::nullopt) const;

  // --- V ↔ h stateless (hybrid) ---------------------------------------------
  double heightFromVolume(double V, const FillEnv& env, double tol = 1e-15);
  double volumeFromHeight(double h, const FillEnv& env, double tol = 1e-15);

  // --- optional geometry hooks (mesh path only) ------------------------------
  bool buildFilledVolumeAtHeight(double h, std::vector<Eigen::Vector3d>& tris_world, const FillEnv& env) const;
  bool buildFilledVolumeAtVolume(double V, std::vector<Eigen::Vector3d>& tris_world, const FillEnv& env) const;

private:
  static const TiltAwareInterface* getTiltAwareIfSupported(const DomainShapeBasePtr& p);
  static Eigen::Vector3d normalizedOr(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback);

  static bool isAnalyticEligiblePrimitive(const geometry::Shape& s);

  // Evaluate local eligibility from a stack; does not retain transforms
  static bool validateStackLocallyAnalytic(const geometry::StackedShape& stack, double tilt_eps_rad,
                                           double lateral_eps_m);

private:
  std::vector<DomainShapeBasePtr> segments_;
  DomainShapeBasePtr mesh_cache_;

  // Canonical default is +X (height/extrusion axis)
  Eigen::Vector3d height_axis_local_{ 1, 0, 0 };  // UNIT, domain local

  // Radians for gravity alignment test (axis ~ parallel to -g_down)
  double parallel_eps_ = 1e-6;

  // Local analytic tolerances for stacked shapes:
  // - tilt wrt +X (allowed twist about +X, no tilt away from +X)
  // - lateral offset in YZ (meters)
  double analytic_tilt_eps_rad_ = 1e-6;
  double analytic_lateral_eps_m_ = 1e-9;

  // Precomputed: true iff the provided stack/shape was locally coaxial+unshifted (+X)
  // and primitives analytic-eligible
  bool segments_locally_axis_aligned_ = false;
};

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_DOMAIN_SHAPE_H_
