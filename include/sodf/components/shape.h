#ifndef SODF_COMPONENTS_SHAPE_H_
#define SODF_COMPONENTS_SHAPE_H_

#include <sodf/ecs.h>
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

struct StackedShapeEntry
{
  geometry::Shape shape;
  Eigen::Isometry3d relative_transform;  // relative to previous shape base
};

using StackedShapes = std::vector<StackedShapeEntry>;

struct StackedShapeComponent
{
  ElementMap<std::string, StackedShapes> elements;
};

}  // namespace components
}  // namespace sodf

#endif
