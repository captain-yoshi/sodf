#ifndef SODF_PHYSICS_DOMAIN_SHAPE
#define SODF_PHYSICS_DOMAIN_SHAPE

#include <vector>
#include <memory>

namespace sodf {
namespace physics {

// forward declaration
class DomainShape;
using DomainShapePtr = std::shared_ptr<DomainShape>;

/**
 * @brief Computes the height (fill level) in a stacked domain for a given volume.
 */
double getMaxFillHeight(const std::vector<DomainShapePtr>& domains);
double getFillHeight(const std::vector<DomainShapePtr>& domains, double volume, double tolerance);

double getMaxFillVolume(const std::vector<DomainShapePtr>& domains);
double getFillVolume(const std::vector<DomainShapePtr>& domains, double height, double tolerance);

/**
 * @brief Abstract base class for computing fill height and fill volume in 3D domains.
 *
 * Provides a standard interface for all 3D domain shapes that support:
 *   - Calculating the fill height corresponding to a specified fill volume (e.g., for determining liquid/solid level sensors
 * or dispensing targets).
 *   - Calculating the fill volume contained up to a specified fill height (e.g., for tracking partially filled containers).
 *   - Querying the maximum possible fill height and fill volume (container's full capacity).
 *
 * Derived classes must implement the geometric relationship between height and volume
 * for their specific container shape. This abstraction enables consistent reasoning about
 * fill levels for fluids, powders, or other fillable materials, regardless of the underlying geometry.
 */
class DomainShape
{
public:
  DomainShape(double max_fill_height);

  virtual ~DomainShape()
  {
  }

  // Returns the fill height corresponding to the given fill volume (from base to fill surface)
  virtual double getFillHeight(double fill_volume) const = 0;

  // Returns the fill volume corresponding to the given fill height (from base)
  virtual double getFillVolume(double fill_height) const = 0;

  // Maximum possible fill height for this domain shape (container's vertical extent)
  double getMaxFillHeight() const
  {
    return max_fill_height_;
  }

  // Maximum possible fill volume for this domain shape (container's capacity)
  double getMaxFillVolume() const
  {
    return max_fill_volume_;
  }

protected:
  double max_fill_volume_ = 0.0;  // Maximum fillable volume (capacity)
  double max_fill_height_ = 0.0;  // Maximum fillable height
};
}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_DOMAIN_SHAPE
