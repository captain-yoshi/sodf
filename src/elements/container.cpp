#include <sodf/elements/container.h>

namespace sodf {
namespace elements {

Container::Container(const std::vector<geometry::BaseVolumePtr>& shape, const geometry::Transform& bottom_tf)
  : max_volume_(geometry::getVolume(shape)), shape_(shape), bottom_tf_(bottom_tf), Element()
{
}

double Container::getCurrentVolume() const
{
  return volume_;
}

double Container::getRemainingVolume() const
{
  return max_volume_ - volume_;
}

double Container::getMaxVolume() const
{
  return max_volume_;
}

double Container::addVolume(double volume)
{
  if (volume_ + volume > max_volume_)
    return 0.0;

  volume_ += volume;
  return volume_;
}
double Container::removeVolume(double volume)
{
  if (volume_ - volume < 0.0)
    return 0.0;

  volume_ -= volume;
  return volume_;
}

double Container::getMaxHeight() const
{
  return geometry::getHeight(shape_);
}

double Container::getCurrentHeight() const
{
  return geometry::getHeight(shape_, volume_, 1e-04);
}

double Container::getHeightFromAddingVolume(double volume) const
{
  double new_volume = volume_ + volume;

  return geometry::getHeight(shape_, new_volume, 1e-04);
}

double Container::getHeightFromRemovingVolume(double volume) const
{
  double new_volume = volume_ - volume;

  return geometry::getHeight(shape_, new_volume, 1e-04);
}

const geometry::Transform& Container::bottomTF() const
{
  return bottom_tf_;
}

}  // namespace elements
}  // namespace sodf
