#include <sodf/volume.h>

#include <cmath>
#include <PolynomialRoots.hh>

namespace sodf {
namespace geometry {

double getHeight(std::vector<BaseVolume> shapes)
{
  double height = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    height += shapes[i].height();

  return height;
}

double getHeight(std::vector<BaseVolume> shapes, double volume, double epsilon)
{
  double height = 0.0;

  if (shapes.empty() || volume <= 0.0)
    return height;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape.volume() == 0.0)
      return 0.0;

    if (volume >= shape.volume())
    {
      height += shape.height();
      volume = volume - shape.volume();
    }
    else
    {
      double shape_height = shape.getHeight(volume);
      if (shape_height == 0.0)
        return 0.0;

      height += shape_height;
      volume = 0;
      break;
    }
  }

  // return 0 if desired height is greater then the sum of the shapes
  if (std::abs(volume) > epsilon)
    return 0.0;

  return height;
}

double getHeight(const RectangularPrismVolume& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (shape.length_ * shape.width_);
}

double getHeight(const CylinderVolume& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (M_PI * shape.radius_ * shape.radius_);
}

double getHeight(const TruncatedConeVolume& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return ((double)3.0 * volume / M_PI) * (double)1.0 /
         (shape.base_radius_ * shape.base_radius_ + shape.base_radius_ * shape.top_radius_ +
          shape.top_radius_ * shape.top_radius_);
}

double getHeight(const SphericalCapVolume& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  double a = -M_PI / 3.0;
  double b = M_PI * shape.radius_;
  double c = 0;
  double d = -volume;

  double roots[5] = {};

  PolynomialRoots::Cubic csolve(a, b, c, d);
  csolve.info(std::cout);
  auto roots_size = csolve.getPositiveRoots(roots);

  if (roots_size == 0)
    return 0.0;
  else
    return roots[0];
}

double getVolume(std::vector<BaseVolume> shapes)
{
  double volume = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    volume += shapes[i].volume();

  return volume;
}

double getVolume(std::vector<BaseVolume> shapes, double height, double epsilon)
{
  double volume = 0.0;

  if (shapes.empty() || height <= 0.0)
    return volume;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape.height() == 0.0)
      return 0.0;

    if (height >= shape.height())
    {
      volume += shape.volume();
      height = height - shape.height();
    }
    else
    {
      double shape_volume = shape.getVolume(height);
      if (shape_volume == 0.0)
        return 0.0;

      volume += shape_volume;
      height = 0;
      break;
    }
  }

  // return 0 if desired height is greater then the sum of the shapes
  if (std::abs(height) > epsilon)
    return 0.0;

  return volume;
}

double getVolume(const RectangularPrismVolume& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return shape.length_ * shape.width_ * height;
}

double getVolume(const CylinderVolume& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return M_PI * shape.radius_ * shape.radius_ * height;
}

double getVolume(const TruncatedConeVolume& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  // Compute new top radii wrt. original shape
  double new_top_radius = (std::tan(shape.alpha_) * height - shape.base_radius_) * -1.0;

  return (1.0 / 3.0) * M_PI * height *
         (shape.base_radius_ * shape.base_radius_ + shape.base_radius_ * new_top_radius +
          new_top_radius * new_top_radius);
}

double getVolume(const SphericalCapVolume& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  // Use the sphere radius formula
  return M_PI * height * height * (shape.radius_ - (1.0 / 3.0) * height);
}

BaseVolume::BaseVolume(double height) : height_(height)
{
}

double BaseVolume::height() const
{
  return height_;
}

double BaseVolume::volume() const
{
  return volume_;
}

RectangularPrismVolume::RectangularPrismVolume(double width /*x*/, double length /*y*/, double height /*z*/)
  : width_(width), length_(length), BaseVolume(height)
{
  volume_ = getVolume(height);
}

double RectangularPrismVolume::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double RectangularPrismVolume::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

CylinderVolume::CylinderVolume(double radius, double height) : radius_(radius), BaseVolume(height)
{
  volume_ = getVolume(height);
}

double CylinderVolume::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double CylinderVolume::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

TruncatedConeVolume::TruncatedConeVolume(double base_radius, double top_radius, double height)
  : base_radius_(base_radius)
  , top_radius_(top_radius)
  , alpha_(std::atan2(base_radius - top_radius, height))
  , BaseVolume(height)
{
  volume_ = getVolume(height);
}

double TruncatedConeVolume::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}
double TruncatedConeVolume::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

SphericalCapVolume::SphericalCapVolume(double cap_radius, double height)
  : cap_radius_(cap_radius), radius_((cap_radius * cap_radius + height * height) / (2 * height)), BaseVolume(height)
{
  volume_ = getVolume(height);
}

double SphericalCapVolume::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double SphericalCapVolume::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

}  // namespace geometry
}  // namespace sodf
