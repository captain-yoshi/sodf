#include <sodf/physics/domain_shape_base.h>
#include <stdexcept>

namespace sodf {
namespace physics {

double getMaxFillHeight(const std::vector<DomainShapeBasePtr>& segments)
{
  double h = 0.0;
  for (const auto& s : segments)
    h += (s ? s->getMaxFillHeight() : 0.0);
  return h;
}

double getMaxFillVolume(const std::vector<DomainShapeBasePtr>& segments)
{
  double V = 0.0;
  for (const auto& s : segments)
    V += (s ? s->getMaxFillVolume() : 0.0);
  return V;
}

double getFillHeight(const std::vector<DomainShapeBasePtr>& segments, double volume, double /*tolerance*/)
{
  if (volume <= 0.0)
    return 0.0;

  double remaining = volume;
  double acc_h = 0.0;

  for (const auto& s : segments)
  {
    if (!s)
      continue;
    const double cap = s->getMaxFillVolume();
    if (remaining >= cap)
    {
      remaining -= cap;
      acc_h += s->getMaxFillHeight();
    }
    else
    {
      // fill partially in this segment
      acc_h += s->getFillHeight(remaining);
      return acc_h;
    }
  }
  // overflow → clamp to full
  return getMaxFillHeight(segments);
}

double getFillVolume(const std::vector<DomainShapeBasePtr>& segments, double height, double /*tolerance*/)
{
  if (height <= 0.0)
    return 0.0;

  double remaining_h = height;
  double acc_V = 0.0;

  for (const auto& s : segments)
  {
    if (!s)
      continue;
    const double hS = s->getMaxFillHeight();
    if (remaining_h >= hS)
    {
      remaining_h -= hS;
      acc_V += s->getMaxFillVolume();
    }
    else
    {
      // partial inside this segment
      acc_V += s->getFillVolume(remaining_h);
      return acc_V;
    }
  }
  // higher than stack → clamp to capacity
  return getMaxFillVolume(segments);
}

}  // namespace physics
}  // namespace sodf
