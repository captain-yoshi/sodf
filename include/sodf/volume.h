#ifndef VOLUME_H_
#define VOLUME_H_

#include <vector>
#include <optional>
#include <memory>

namespace sodf {
namespace geometry {

class BaseVolume;
using BaseVolumePtr = std::shared_ptr<BaseVolume>;
class RectangularPrismVolume;
class CylinderVolume;
class TruncatedConeVolume;
class SphericalCapVolume;

/**
 * @brief Computes the height of the shape
 */

/**
 * @brief Computes the height of the shape/s given a volume
 */
double getHeight(const std::vector<BaseVolumePtr>& shapes);
double getHeight(const std::vector<BaseVolumePtr>& shapes, double volume, double epsilon);
double getHeight(const RectangularPrismVolume& shape, double volume);
double getHeight(const CylinderVolume& shape, double volume);
double getHeight(const TruncatedConeVolume& shape, double volume);
double getHeight(const SphericalCapVolume& shape, double volume);

double getVolume(const std::vector<BaseVolumePtr>& shapes);
double getVolume(const std::vector<BaseVolumePtr>& shapes, double height, double epsilon);
double getVolume(const RectangularPrismVolume& shape, double height);
double getVolume(const CylinderVolume& shape, double height);
double getVolume(const TruncatedConeVolume& shape, double height);
double getVolume(const SphericalCapVolume& shape, double height);

/**
 * @brief Base class for computing height/volume of 3D shapes
 */
class BaseVolume
{
public:
  BaseVolume(double height);

  virtual ~BaseVolume(){};

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
class RectangularPrismVolume : public BaseVolume
{
public:
  RectangularPrismVolume(double width /*x*/, double length /*y*/, double height /*z*/);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double width_;   // X axis
  const double length_;  // Y axis
};

/**
 * @brief Cylinder
 */
class CylinderVolume : public BaseVolume
{
public:
  CylinderVolume(double radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double radius_;  // Plane along X and Y axis
};

/**
 * @brief Truncated Cone
 */
class TruncatedConeVolume : public BaseVolume
{
public:
  TruncatedConeVolume(double base_radius, double top_radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double base_radius_;
  const double top_radius_;
  const double alpha_;  // angle
};

class SphericalCapVolume : public BaseVolume
{
public:
  SphericalCapVolume(double cap_radius, double height);

  double getHeight(double volume) const override;
  double getVolume(double height) const override;

  const double cap_radius_;  // radius of the spherical cap
  const double radius_;      // radius of the sphere
};

}  // namespace geometry
}  // namespace sodf

#endif  // VOLUME_H_
