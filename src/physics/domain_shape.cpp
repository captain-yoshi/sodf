#include <sodf/physics/domain_shape.h>

#include <cmath>

namespace sodf {
namespace physics {

double getMaxFillHeight(const std::vector<DomainShapePtr>& shapes)
{
  double height = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    height += shapes[i]->getMaxFillHeight();

  return height;
}

double getFillHeight(const std::vector<DomainShapePtr>& shapes, double volume, double tolerance)
{
  double height = 0.0;

  if (shapes.empty() || volume <= 0.0)
    return height;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape->getMaxFillVolume() == 0.0)
      return 0.0;

    if (volume >= shape->getMaxFillVolume())
    {
      height += shape->getMaxFillHeight();
      volume = volume - shape->getMaxFillVolume();
    }
    else
    {
      double shape_height = shape->getFillHeight(volume);
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

double getMaxFillVolume(const std::vector<DomainShapePtr>& shapes)
{
  double volume = 0.0;

  for (std::size_t i = 0; i < shapes.size(); ++i)
    volume += shapes[i]->getMaxFillVolume();

  return volume;
}

double getFillVolume(const std::vector<DomainShapePtr>& shapes, double height, double tolerance)
{
  double volume = 0.0;

  if (shapes.empty() || height <= 0.0)
    return volume;

  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    auto& shape = shapes[i];

    // guard against bad shape
    if (shape->getMaxFillHeight() == 0.0)
      return 0.0;

    if (height >= shape->getMaxFillHeight())
    {
      volume += shape->getMaxFillVolume();
      height = height - shape->getMaxFillHeight();
    }
    else
    {
      double shape_volume = shape->getFillVolume(height);
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

DomainShape::DomainShape(double max_fill_height) : max_fill_height_(max_fill_height){};

}  // namespace physics
}  // namespace sodf
