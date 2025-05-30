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

  double r1 = shape.base_radius_;
  double r2 = shape.top_radius_;
  double H = shape.height();

  double k = (r2 - r1) / H;

  // Coefficients for V(h) = a h^3 + b h^2 + c h
  double a = (M_PI / 3.0) * k * k;
  double b = M_PI * r1 * k;
  double c = M_PI * r1 * r1;
  double d = -volume;

  double roots[5] = {};
  PolynomialRoots::Cubic csolve(a, b, c, d);
  auto roots_size = csolve.getPositiveRoots(roots);
  if (roots_size == 0)
    return 0.0;
  return roots[0];
}

double getFluidHeight(const SphericalSegmentShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.volume())
    return 0.0;

  double a1 = shape.base_radius_;  // base radius at y=0
  double a2 = shape.top_radius_;   // top radius at y=H
  double H = shape.height();

  // Compute z0 (distance from base to sphere center)
  double z0 = (a2 * a2 - a1 * a1 + H * H) / (2 * H);
  double r = std::sqrt(a1 * a1 + z0 * z0);

  // Lambda for segment volume up to height h
  auto segment_volume = [&](double h) {
    double ah2 = r * r - (z0 - h) * (z0 - h);
    double ah = (ah2 > 0) ? std::sqrt(ah2) : 0.0;
    return M_PI * h / 6.0 * (3 * a1 * a1 + 3 * ah * ah + h * h);
  };

  // Binary search for h
  double low = 0.0, high = H, mid = 0.0;
  for (int iter = 0; iter < 100; ++iter)
  {
    mid = 0.5 * (low + high);
    double v = segment_volume(mid);
    if (std::abs(v - volume) < 1e-09)  // adjust tolerance as needed
      break;
    if (v < volume)
      low = mid;
    else
      high = mid;
  }
  return mid;
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

  double H = shape.height();
  double r_start, r_end;

  r_start = shape.base_radius_;
  r_end = shape.base_radius_ + (shape.top_radius_ - shape.base_radius_) * (height / H);

  // Frustum volume formula
  double volume = (M_PI * height / 3.0) * (r_start * r_start + r_start * r_end + r_end * r_end);

  return volume;
}

double getFluidVolume(const SphericalSegmentShape& shape, double height)
{
  // Height is always positive, meaning "distance away from the reference point"
  if (height <= 0.0 || height > shape.height())
    return 0.0;

  double a1 = shape.base_radius_;  // base (at height 0)
  double a2 = shape.top_radius_;   // top (at height H)
  double H = shape.height();

  // Step 1: Compute z0 (distance from base to sphere center)
  double z0 = (a2 * a2 - a1 * a1 + H * H) / (2 * H);

  // Step 2: Compute sphere radius
  double r = std::sqrt(a1 * a1 + z0 * z0);

  // Step 3: Compute cross-section radius at fluid height
  double ah2 = r * r - (z0 - height) * (z0 - height);
  double ah = (ah2 > 0) ? std::sqrt(ah2) : 0.0;

  // Step 4: Plug into segment formula
  return M_PI * height / 6.0 * (3 * a1 * a1 + 3 * ah * ah + height * height);
}

DomainShape::DomainShape(double height) : height_(height)
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

BoxShape::BoxShape(double width /*x*/, double length /*y*/, double height)
  : width_(width), length_(length), DomainShape(height)
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

CylinderShape::CylinderShape(double radius, double height) : radius_(radius), DomainShape(height)
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

ConeShape::ConeShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
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

SphericalSegmentShape::SphericalSegmentShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
{
  volume_ = getVolume(height);
}

double SphericalSegmentShape::getHeight(double volume) const
{
  return getFluidHeight(*this, volume);
}

double SphericalSegmentShape::getVolume(double height) const
{
  return getFluidVolume(*this, height);
}

}  // namespace fluid
}  // namespace sodf
