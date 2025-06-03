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

struct CompositeShapeEntry
{
  geometry::Shape shape;                        // Geometry and properties
  Eigen::Isometry3d local_transform;  // FINAL local pose relative to composite origin
};

struct CompositeShape
{
  std::vector<CompositeShapeEntry> shapes;  // All primitives, fully positioned and oriented
};

struct CompositeShapeComponent
{
  FlatMap<std::string, CompositeShape> composite_shape_map;
};





}  // namespace components
}  // namespace sodf

#endif
