#ifndef SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
#define SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_

#include <sodf/physics/domain_shape.h>

namespace sodf {
namespace physics {

// Forward declarations
class FluidBoxShape;
class FluidCylinderShape;
class FluidConeShape;
class FluidSphericalSegmentShape;

/**
 * @brief Computes the fill height (level) in a stacked fluid domain for a given fill volume.
 */
double getFillHeight(const FluidBoxShape& domain, double fill_volume);
double getFillHeight(const FluidCylinderShape& domain, double fill_volume);
double getFillHeight(const FluidConeShape& domain, double fill_volume);
double getFillHeight(const FluidSphericalSegmentShape& domain, double fill_volume);

double getFillVolume(const FluidBoxShape& domain, double fill_height);
double getFillVolume(const FluidCylinderShape& domain, double fill_height);
double getFillVolume(const FluidConeShape& domain, double fill_height);
double getFillVolume(const FluidSphericalSegmentShape& domain, double fill_height);

/**
 * @brief Rectangular Prism ("Box") fluid domain shape.
 *
 * Models a right rectangular prism (box) with given width, length, and height.
 * Provides closed-form solutions for fill height/volume relationships.
 */
class FluidBoxShape : public DomainShape
{
public:
  FluidBoxShape(double width, double length, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double width_;
  const double length_;
};

/**
 * @brief Cylinder fluid domain shape (right circular cylinder).
 *
 * Models a cylinder with constant radius and height. Implements analytic formulas for fill height/volume.
 */
class FluidCylinderShape : public DomainShape
{
public:
  FluidCylinderShape(double radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double radius_;
};

/**
 * @brief Fluid domain cone or frustum (truncated cone) shape.
 *
 * Represents the volume and geometry of a cone or frustum defined by a base radius,
 * a top radius, and a height. The shape can be a perfect cone (if either the base
 * or top radius is zero), or a frustum (if both radii are nonzero).
 *
 * The base is at z = 0 (with radius base_radius_) and the top is at z = max_fill_height (with radius top_radius_).
 * If base_radius_ < top_radius_, the cone is "upside-down" (widest at the top).
 * If base_radius_ > top_radius_, the cone tapers upwards (widest at the base).
 *
 * The class supports calculation of the fill height required to reach a specified volume,
 * as well as the volume up to a specified fill height.
 */
class FluidConeShape : public DomainShape
{
public:
  FluidConeShape(double base_radius, double top_radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double base_radius_;  // Radius at base (z = 0)
  const double top_radius_;   // Radius at top (z = max_fill_height)
};

/**
 * @brief Fluid domain spherical segment shape (portion of a sphere "cut" by two planes).
 *
 * Represents a 3D shape corresponding to a segment of a sphere,
 * defined by the base radius, top radius, and segment height.
 *
 * The segment is modeled such that:
 *   - The flat, circular base of the segment is at z = 0.
 *   - The "tip" (the furthest point from the base) is at z = max_fill_height.
 *   - The full sphere's center lies along the symmetry axis.
 *
 * The geometric convention is that the segment is "standing" with the base at the bottom (z = 0),
 * and the tip at z = max_fill_height.
 */
class FluidSphericalSegmentShape : public DomainShape
{
public:
  FluidSphericalSegmentShape(double base_radius, double top_radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double base_radius_;  // radius at the base (z = 0)
  const double top_radius_;   // radius at the top (z = max_fill_height)
};

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
