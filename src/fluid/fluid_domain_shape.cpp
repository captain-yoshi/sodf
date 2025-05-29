#include <sodf/fluid/fluid_domain_shape.h>

#include <cmath>
#include <PolynomialRoots.hh>

namespace sodf {
namespace fluid {

double getFluidHeight(const std::vector<DomainShapePtr>& shapes)
{
  double height = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    height += shapes[i]->height();

  return height;
}

double getFluidHeight(const std::vector<DomainShapePtr>& shapes, double volume, double tolerance)
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
  if (std::abs(volume) > tolerance)
    return 0.0;

  return height;
}

double getFluidHeight(const BoxShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (shape.length_ * shape.width_);
}

double getFluidHeight(const CylinderShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  return volume / (M_PI * shape.radius_ * shape.radius_);
}

double getFluidHeight(const ConeShape& shape, double volume)
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

double getFluidHeight(const SphericalCapShape& shape, double volume)
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

double getFluidVolume(const std::vector<DomainShapePtr>& shapes)
{
  double volume = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    volume += shapes[i]->volume();

  return volume;
}

double getFluidVolume(const std::vector<DomainShapePtr>& shapes, double height, double tolerance)
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
  if (std::abs(height) > tolerance)
    return 0.0;

  return volume;
}

double getFluidVolume(const BoxShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return shape.length_ * shape.width_ * height;
}

double getFluidVolume(const CylinderShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  return M_PI * shape.radius_ * shape.radius_ * height;
}

double getFluidVolume(const ConeShape& shape, double height)
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

double getFluidVolume(const SphericalCapShape& shape, double height)
{
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  // Use the sphere radius formula
  return M_PI * height * height * (shape.radius_ - (1.0 / 3.0) * height);
}

DomainShape::DomainShape(double height, bool invert) : height_(height), invert_(invert)
{
}

double DomainShape::height() const
{
  return height_;
}

double DomainShape::volume() const
{
  return volume_;
}

BoxShape::BoxShape(double width /*x*/, double length /*y*/, double height, bool invert)
  : width_(width), length_(length), DomainShape(height, invert)
{
  volume_ = getVolume(height);
}

double BoxShape::getHeight(double volume) const
{
  return getFluidHeight(*this, volume);
}

double BoxShape::getVolume(double height) const
{
  return getFluidVolume(*this, height);
}

CylinderShape::CylinderShape(double radius, double height) : radius_(radius), DomainShape(height, false)
{
  volume_ = getVolume(height);
}

double CylinderShape::getHeight(double volume) const
{
  return getFluidHeight(*this, volume);
}

double CylinderShape::getVolume(double height) const
{
  return getFluidVolume(*this, height);
}

ConeShape::ConeShape(double base_radius, double top_radius, double height, bool invert)
  : base_radius_(base_radius)
  , top_radius_(top_radius)
  , alpha_(std::atan2(base_radius - top_radius, height))
  , DomainShape(height, invert)
{
  volume_ = getVolume(height);
}

double ConeShape::getHeight(double volume) const
{
  return getFluidHeight(*this, volume);
}
double ConeShape::getVolume(double height) const
{
  return getFluidVolume(*this, height);
}

SphericalCapShape::SphericalCapShape(double cap_radius, double height, bool invert)
  : cap_radius_(cap_radius)
  , radius_((cap_radius * cap_radius + height * height) / (2 * height))
  , DomainShape(height, invert)
{
  volume_ = getVolume(height);
}

double SphericalCapShape::getHeight(double volume) const
{
  return getFluidHeight(*this, volume);
}

double SphericalCapShape::getVolume(double height) const
{
  return getFluidVolume(*this, height);
}

}  // namespace fluid
}  // namespace sodf
