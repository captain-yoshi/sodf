#ifndef VOLUME_H_
#define VOLUME_H_

#include <vector>
#include <optional>

namespace sodf {
namespace geometry {

class BaseVolume;
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
double getHeight(std::vector<BaseVolume> shapes);
double getHeight(std::vector<BaseVolume> shapes, double volume);
double getHeight(const RectangularPrismVolume& shape, double volume);
double getHeight(const CylinderVolume& shape, double volume);
double getHeight(const TruncatedConeVolume& shape, double volume);
double getHeight(const SphericalCapVolume& shape, double volume);

double getVolume(std::vector<BaseVolume> shapes);
double getVolume(std::vector<BaseVolume> shapes, double height);
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
  BaseVolume(double height, double volume);

  virtual ~BaseVolume(){};

  virtual double getHeight(double volume) = 0;
  virtual double getVolume(double height) = 0;

  const double volume_;
  const double height_;  // Z axis
};

/**
 * @brief Rectangular Prism
 */
class RectangularPrismVolume : public BaseVolume
{
public:
  RectangularPrismVolume(double width /*x*/, double length /*y*/, double height /*z*/);

  double getHeight(double volume) override;
  double getVolume(double height) override;

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

  double getHeight(double volume) override;
  double getVolume(double height) override;

  const double radius_;  // Plane along X and Y axis
};

/**
 * @brief Truncated Cone
 */
class TruncatedConeVolume : public BaseVolume
{
public:
  TruncatedConeVolume(double base_radius, double top_radius, double height);

  double getHeight(double volume) override;
  double getVolume(double height) override;

  const double base_radius_;
  const double top_radius_;
};

class SphericalCapVolume : public BaseVolume
{
public:
  SphericalCapVolume(double radius, double height);

  double getHeight(double volume) override;
  double getVolume(double height) override;

  const double radius_;  // radius of the base of the cap
};

}  // namespace geometry
}  // namespace sodf

#endif  // VOLUME_H_
