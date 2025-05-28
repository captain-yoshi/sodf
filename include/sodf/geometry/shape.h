#ifndef SHAPE_H_
#define SHAPE_H_

#include <vector>
#include <optional>
#include <memory>

namespace sodf {
namespace geometry {

class BaseShape;
using BaseShapePtr = std::shared_ptr<BaseShape>;
class BoxShape;
class CylinderShape;
class ConeShape;
class SphericalCapShape;

/**
 * @brief Computes the height of the shape/s given a volume
 */
double getHeight(const std::vector<BaseShapePtr>& shapes);
double getHeight(const std::vector<BaseShapePtr>& shapes, double volume, double epsilon);
double getHeight(const BoxShape& shape, double volume);
double getHeight(const CylinderShape& shape, double volume);
double getHeight(const ConeShape& shape, double volume);
double getHeight(const SphericalCapShape& shape, double volume);

double getVolume(const std::vector<BaseShapePtr>& shapes);
double getVolume(const std::vector<BaseShapePtr>& shapes, double height, double epsilon);
double getVolume(const BoxShape& shape, double height);
double getVolume(const CylinderShape& shape, double height);
double getVolume(const ConeShape& shape, double height);
double getVolume(const SphericalCapShape& shape, double height);

/**
 * @brief Abstract base class for computing height and volume of 3D shapes.
 *
 * Provides an interface for all 3D shapes that support computation of either:
 *   - The height corresponding to a specified volume (e.g., for fill-level sensors or dispensing),
 *   - The volume contained up to a specified height (e.g., for partially filled containers).
 *
 * Derived classes must implement the geometric relationship between height and volume
 * for their specific shape.
 */
class BaseShape
{
public:
  BaseShape(double height);

  virtual ~BaseShape(){};

  virtual double getHeight(double volume) const = 0;
  virtual double getVolume(double height) const = 0;

  double height() const;
  double volume() const;

protected:
  double volume_;
  double height_;
};

/**
 * @brief Rectangular Prism ("Box") shape.
 *
 * Models a right rectangular prism (box) with given width, length, and height.
 * Provides closed-form solutions for height/volume relationships.
 */
class BoxShape : public BaseShape
{
public:
  BoxShape(double width, double length, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double width_;
  const double length_;
};

/**
 * @brief Cylinder shape (right circular cylinder).
 *
 * Models a cylinder with constant radius and height. Implements analytic formulas for height/volume.
 */
class CylinderShape : public BaseShape
{
public:
  CylinderShape(double radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double radius_;
};

/**
 * @brief A geometric cone or frustum (truncated cone) shape.
 *
 * Represents the volume and geometry of a cone or frustum defined by a base radius,
 * a top radius, and a height. The shape can be a perfect cone (if either the base
 * or top radius is zero), or a frustum (if both radii are nonzero).
 *
 * The base is at z = 0 (with radius base_radius_) and the top is at z = height (with radius top_radius_).
 * If base_radius_ < top_radius_, the cone is "upside-down" (widest at the top).
 * If base_radius_ > top_radius_, the cone tapers upwards (widest at the base).
 *
 * The class supports calculation of the height required to reach a specified volume,
 * as well as the volume up to a specified height.
 *
 * @param base_radius_  Radius of the base (at z = 0)
 * @param top_radius_   Radius of the top (at z = height)
 * @param height        Height of the cone/frustum
 * @param alpha_        Half-angle of the cone (derived from radii and height)
 */
class ConeShape : public BaseShape
{
public:
  ConeShape(double base_radius, double top_radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double base_radius_;  // Radius at base (z = 0)
  const double top_radius_;   // Radius at top (z = height)
  const double alpha_;        // Half-angle of the cone (derived)
};

/**
 * @brief Spherical cap shape (portion of a sphere "cut" by a plane).
 *
 * Represents a 3D shape corresponding to a segment of a sphere ("cap"),
 * defined by the height of the cap (from the flat base to the tip) and the cap radius.
 *
 * The cap is modeled such that:
 *   - The flat, circular base of the cap is at z = 0.
 *   - The "tip" (the furthest point from the base) is at z = height.
 *   - The full sphere's center lies along the symmetry axis below the base,
 *     at a distance (radius_ - height) from the tip.
 *
 * @param cap_radius_  Radius of the base circle (flat face at z = 0).
 * @param height       Height of the cap (distance from base to tip).
 * @param radius_      Radius of the sphere from which the cap is formed.
 *
 * The geometric convention is that the cap is "standing" with the base at the bottom (z = 0),
 * and the tip at z = height. The tangent circle (base) is at the bottom; the tip is at the top.
 */
class SphericalCapShape : public BaseShape
{
public:
  SphericalCapShape(double cap_radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double cap_radius_;  // radius of the spherical cap
  const double radius_;      // radius of the sphere
};

}  // namespace geometry
}  // namespace sodf

#endif  // SHAPE_H_
