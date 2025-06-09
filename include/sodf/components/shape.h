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
  FlatMap<std::string, geometry::Shape> shape_map;
};

struct StackedShapeEntry
{
  geometry::Shape shape;
  Eigen::Isometry3d relative_transform;  // relative to previous shape base
};

using StackedShapes = std::vector<StackedShapeEntry>;

struct StackedShapeComponent
{
  FlatMap<std::string, StackedShapes> stack_map;
};

}  // namespace components
}  // namespace sodf

#endif
