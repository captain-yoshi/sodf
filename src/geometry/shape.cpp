#include <sodf/geometry/shape.h>

#include <cmath>
#include <PolynomialRoots.hh>

namespace sodf {
namespace geometry {

double getHeight(const std::vector<BaseShapePtr>& shapes)
{
  double height = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    height += shapes[i]->height();

  return height;
}

double getHeight(const std::vector<BaseShapePtr>& shapes, double volume, double epsilon)
{
  double height = 0.0;

  if (shapes.empty() || volume <= 0.0)
    return height;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape->volume() == 0.0)
      return 0.0;

    if (volume >= shape->volume())
    {
      height += shape->height();
      volume = volume - shape->volume();
    }
    else
    {
      double shape_height = shape->getHeight(volume);
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

double getHeight(const RectangularPrismShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (shape.length_ * shape.width_);
}

double getHeight(const CylinderShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (M_PI * shape.radius_ * shape.radius_);
}

double getHeight(const TruncatedConeShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  double a = (1.0 / 3.0) * M_PI * std::tan(shape.alpha_) * std::tan(shape.alpha_);
  double b = -M_PI * std::tan(shape.alpha_) * shape.base_radius_;
  double c = M_PI * shape.base_radius_ * shape.base_radius_;
  double d = -volume;

  double roots[5] = {};

  PolynomialRoots::Cubic csolve(a, b, c, d);
  // csolve.info(std::cout);
  auto roots_size = csolve.getPositiveRoots(roots);

  if (roots_size == 0)
    return 0.0;
  else
    return roots[0];
}

double getHeight(const SphericalCapShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  double a = -M_PI / 3.0;
  double b = M_PI * shape.radius_;
  double c = 0;
  double d = -volume;

  double roots[5] = {};

  PolynomialRoots::Cubic csolve(a, b, c, d);
  // csolve.info(std::cout);
  auto roots_size = csolve.getPositiveRoots(roots);

  if (roots_size == 0)
    return 0.0;
  else
    return roots[0];
}

double getVolume(const std::vector<BaseShapePtr>& shapes)
{
  double volume = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    volume += shapes[i]->volume();

  return volume;
}

double getVolume(const std::vector<BaseShapePtr>& shapes, double height, double epsilon)
{
  double volume = 0.0;

  if (shapes.empty() || height <= 0.0)
    return volume;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape->height() == 0.0)
      return 0.0;

    if (height >= shape->height())
    {
      volume += shape->volume();
      height = height - shape->height();
    }
    else
    {
      double shape_volume = shape->getVolume(height);
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

double getVolume(const RectangularPrismShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return shape.length_ * shape.width_ * height;
}

double getVolume(const CylinderShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return M_PI * shape.radius_ * shape.radius_ * height;
}

double getVolume(const TruncatedConeShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  // Compute new top radii wrt. original shape
  // https://math.stackexchange.com/a/2219108
  double new_top_radius = (std::tan(shape.alpha_) * height - shape.base_radius_) * -1.0;

  return (1.0 / 3.0) * M_PI * height *
         (shape.base_radius_ * shape.base_radius_ + shape.base_radius_ * new_top_radius +
          new_top_radius * new_top_radius);
}

double getVolume(const SphericalCapShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  // Use the sphere radius formula
  return M_PI * height * height * (shape.radius_ - (1.0 / 3.0) * height);
}

BaseShape::BaseShape(double height) : height_(height)
{
}

double BaseShape::height() const
{
  return height_;
}

double BaseShape::volume() const
{
  return volume_;
}

RectangularPrismShape::RectangularPrismShape(double width /*x*/, double length /*y*/, double height /*z*/)
  : width_(width), length_(length), BaseShape(height)
{
  volume_ = getVolume(height);
}

double RectangularPrismShape::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double RectangularPrismShape::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

CylinderShape::CylinderShape(double radius, double height) : radius_(radius), BaseShape(height)
{
  volume_ = getVolume(height);
}

double CylinderShape::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double CylinderShape::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

TruncatedConeShape::TruncatedConeShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius)
  , top_radius_(top_radius)
  , alpha_(std::atan2(base_radius - top_radius, height))
  , BaseShape(height)
{
  volume_ = getVolume(height);
}

double TruncatedConeShape::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}
double TruncatedConeShape::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

SphericalCapShape::SphericalCapShape(double cap_radius, double height)
  : cap_radius_(cap_radius), radius_((cap_radius * cap_radius + height * height) / (2 * height)), BaseShape(height)
{
  volume_ = getVolume(height);
}

double SphericalCapShape::getHeight(double volume) const
{
  return ::sodf::geometry::getHeight(*this, volume);
}

double SphericalCapShape::getVolume(double height) const
{
  return ::sodf::geometry::getVolume(*this, height);
}

}  // namespace geometry
}  // namespace sodf
