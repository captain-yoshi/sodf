#ifndef SODF_PHYSICS_DOMAIN_SHAPE_BASE_H_
#define SODF_PHYSICS_DOMAIN_SHAPE_BASE_H_

#include <memory>
#include <vector>
#include <optional>
#include <functional>
#include <cmath>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sodf {
namespace physics {

// Stateless environment bundle for tilt/off-axis queries
struct FillEnv
{
  Eigen::Vector3d g_down_world;      // Gravity (DOWN). Need not be unit.
  Eigen::Isometry3d T_world_domain;  // Domain pose in world
  Eigen::Vector3d p_base_world;      // World-space point where h=0 (inner bottom)
};

// Axis-aligned abstract shape (LOCAL +Z = up). Immutable for queries.
class DomainShapeBase
{
public:
  explicit DomainShapeBase(double max_fill_height) : max_fill_height_(max_fill_height)
  {
  }
  virtual ~DomainShapeBase() = default;

  // core V ↔ h mapping (local, +Z up)
  virtual double getFillHeight(double fill_volume) const = 0;  // V -> h
  virtual double getFillVolume(double fill_height) const = 0;  // h -> V

  double getMaxFillHeight() const
  {
    return max_fill_height_;
  }
  double getMaxFillVolume() const
  {
    return max_fill_volume_;
  }

protected:
  double max_fill_volume_ = 0.0;  // capacity
  double max_fill_height_ = 0.0;  // vertical extent (local)
};

using DomainShapeBasePtr = std::shared_ptr<DomainShapeBase>;

double getMaxFillHeight(const std::vector<DomainShapeBasePtr>& segments);
double getFillHeight(const std::vector<DomainShapeBasePtr>& segments, double volume, double tolerance);
double getMaxFillVolume(const std::vector<DomainShapeBasePtr>& segments);
double getFillVolume(const std::vector<DomainShapeBasePtr>& segments, double height, double tolerance);

// Optional mixin for shapes that support tilt/off-axis via pure stateless calls
struct TiltAwareInterface
{
  virtual ~TiltAwareInterface() = default;

  virtual double getFillHeightWithEnv(double fill_volume, const FillEnv& env) const = 0;
  virtual double getFillVolumeWithEnv(double fill_height_world, const FillEnv& env) const = 0;

  // Optional viz hooks (stateless, default no-op)
  virtual bool buildFilledVolumeAtHeightWithEnv(double /*h_world*/, std::vector<Eigen::Vector3d>& /*tris_world*/,
                                                const FillEnv& /*env*/) const
  {
    return false;
  }

  virtual bool buildFilledVolumeAtVolumeWithEnv(double /*V*/, std::vector<Eigen::Vector3d>& /*tris_world*/,
                                                const FillEnv& /*env*/) const
  {
    return false;
  }
};

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_DOMAIN_SHAPE_BASE_H_
