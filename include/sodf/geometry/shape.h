#ifndef SHAPE_H_
#define SHAPE_H_

#include <vector>
#include <optional>
#include <memory>

namespace sodf {
namespace geometry {

class BaseShape;
using BaseShapePtr = std::shared_ptr<BaseShape>;
class RectangularPrismShape;
class CylinderShape;
class TruncatedConeShape;
class SphericalCapShape;

/**
 * @brief Computes the height of the shape
 */

/**
 * @brief Computes the height of the shape/s given a volume
 */
double getHeight(const std::vector<BaseShapePtr>& shapes);
double getHeight(const std::vector<BaseShapePtr>& shapes, double volume, double epsilon);
double getHeight(const RectangularPrismShape& shape, double volume);
double getHeight(const CylinderShape& shape, double volume);
double getHeight(const TruncatedConeShape& shape, double volume);
double getHeight(const SphericalCapShape& shape, double volume);

double getVolume(const std::vector<BaseShapePtr>& shapes);
double getVolume(const std::vector<BaseShapePtr>& shapes, double height, double epsilon);
double getVolume(const RectangularPrismShape& shape, double height);
double getVolume(const CylinderShape& shape, double height);
double getVolume(const TruncatedConeShape& shape, double height);
double getVolume(const SphericalCapShape& shape, double height);

/**
 * @brief Base class for computing height/volume of 3D shapes
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
  double height_;  // Z axis
};

/**
 * @brief Rectangular Prism
 */
class RectangularPrismShape : public BaseShape
{
public:
  RectangularPrismShape(double width /*x*/, double length /*y*/, double height /*z*/);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double width_;   // X axis
  const double length_;  // Y axis
};

/**
 * @brief Cylinder
 */
class CylinderShape : public BaseShape
{
public:
  CylinderShape(double radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double radius_;  // Plane along X and Y axis
};

/**
 * @brief Truncated Cone
 */
class TruncatedConeShape : public BaseShape
{
public:
  TruncatedConeShape(double base_radius, double top_radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double base_radius_;
  const double top_radius_;
  const double alpha_;  // angle
};

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
