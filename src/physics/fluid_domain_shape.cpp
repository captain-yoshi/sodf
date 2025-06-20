#include <sodf/physics/fluid_domain_shape.h>

#include <cmath>
#include <PolynomialRoots.hh>

#include <sodf/geometry/shape.h>

namespace sodf {
namespace physics {

double getFillHeight(const FluidBoxShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;
  return volume / (shape.length_ * shape.width_);
}

double getFillHeight(const FluidCylinderShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;
  return volume / (M_PI * shape.radius_ * shape.radius_);
}

double getFillHeight(const FluidConeShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;

  double r1 = shape.base_radius_;
  double r2 = shape.top_radius_;
  double H = shape.getMaxFillHeight();

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

double getFillHeight(const FluidSphericalSegmentShape& shape, double volume)
{
  if (volume <= 0.0 || volume > shape.getMaxFillVolume())
    return 0.0;

  double a1 = shape.base_radius_;  // base radius at y=0
  double a2 = shape.top_radius_;   // top radius at y=H
  double H = shape.getMaxFillHeight();

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

double getFillVolume(const FluidBoxShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;
  return shape.length_ * shape.width_ * height;
}

double getFillVolume(const FluidCylinderShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;
  return M_PI * shape.radius_ * shape.radius_ * height;
}

double getFillVolume(const FluidConeShape& shape, double height)
{
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;

  double H = shape.getMaxFillHeight();
  double r_start = shape.base_radius_;
  double r_end = shape.base_radius_ + (shape.top_radius_ - shape.base_radius_) * (height / H);

  // Frustum volume formula
  double volume = (M_PI * height / 3.0) * (r_start * r_start + r_start * r_end + r_end * r_end);

  return volume;
}

double getFillVolume(const FluidSphericalSegmentShape& shape, double height)
{
  // Height is always positive, meaning "distance away from the reference point"
  if (height <= 0.0 || height > shape.getMaxFillHeight())
    return 0.0;

  double a1 = shape.base_radius_;  // base (at height 0)
  double a2 = shape.top_radius_;   // top (at height H)
  double H = shape.getMaxFillHeight();

  // Step 1: Compute z0 (distance from base to sphere center)
  double z0 = (a2 * a2 - a1 * a1 + H * H) / (2 * H);

  // Step 2: Compute sphere radius
  double r = std::sqrt(a1 * a1 + z0 * z0);

  // Step 3: Compute cross-section radius at fill height
  double ah2 = r * r - (z0 - height) * (z0 - height);
  double ah = (ah2 > 0) ? std::sqrt(ah2) : 0.0;

  // Step 4: Plug into segment formula
  return M_PI * height / 6.0 * (3 * a1 * a1 + 3 * ah * ah + height * height);
}

// FluidBoxShape

FluidBoxShape::FluidBoxShape(double width, double length, double height)
  : width_(width), length_(length), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidBoxShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidBoxShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidCylinderShape

FluidCylinderShape::FluidCylinderShape(double radius, double height) : radius_(radius), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidCylinderShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidCylinderShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidConeShape

FluidConeShape::FluidConeShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
{
  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidConeShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidConeShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

// FluidSphericalSegmentShape

FluidSphericalSegmentShape::FluidSphericalSegmentShape(double base_radius, double top_radius, double height)
  : base_radius_(base_radius), top_radius_(top_radius), DomainShape(height)
{
  if (!geometry::isValidSegment(base_radius, top_radius, height))
  {
    std::ostringstream oss;
    oss << "Incompatible spherical segment dimensions: "
        << "base_radius = " << base_radius << ", "
        << "top_radius = " << top_radius << ", "
        << "height = " << height << ". "
        << "No valid sphere exists for these values.";
    throw std::invalid_argument(oss.str());
  }

  max_fill_volume_ = physics::getFillVolume(*this, height);
}

double FluidSphericalSegmentShape::getFillHeight(double volume) const
{
  return physics::getFillHeight(*this, volume);
}

double FluidSphericalSegmentShape::getFillVolume(double height) const
{
  return physics::getFillVolume(*this, height);
}

FluidSphericalSegmentShape fromBaseTop(double r1, double r2)
{
  double h = geometry::inferSegmentHeightFromRadii(r1, r2);
  return FluidSphericalSegmentShape(r1, r2, h);
}

FluidSphericalSegmentShape fromBaseHeight(double r1, double h)
{
  double r2 = geometry::inferTopRadiusFromHeight(r1, h);
  return FluidSphericalSegmentShape(r1, r2, h);
}

FluidSphericalSegmentShape fromBaseSphereRadius(double r1, double R)
{
  if (R <= r1)
    throw std::invalid_argument("Sphere radius must be greater than base radius");

  double theta = std::asin(r1 / R);
  double h = R * std::cos(theta);
  double r2 = R * std::sin(theta + h / R);  // or recompute more directly

  return FluidSphericalSegmentShape(r1, r2, h);
}

}  // namespace physics
}  // namespace sodf
