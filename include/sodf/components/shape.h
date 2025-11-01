#ifndef SODF_COMPONENTS_SHAPE_H_
#define SODF_COMPONENTS_SHAPE_H_

#include <sodf/components/data_type.h>
#include <sodf/geometry/shape.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

// Forward declaration
struct ShapeComponent
{
  ElementMap<std::string, geometry::Shape> elements;
};

struct StackedShapeComponent
{
  ElementMap<std::string, geometry::StackedShape> elements;
};

}  // namespace components
}  // namespace sodf

#endif
