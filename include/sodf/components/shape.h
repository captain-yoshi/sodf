#ifndef SODF_COMPONENTS_SHAPE_H_
#define SODF_COMPONENTS_SHAPE_H_

#include <sodf/ecs.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

enum class ShapeType
{
  // 2D
  Line,
  Rectangle,
  Circle,
  Polygon,
  Triangle,

  // 3D
  Box,
  Cylinder,
  Sphere,
  Cone,
  SphericalSegment,
  Plane,
  Mesh,
};

/*
| ShapeType        | Dimensions                      | Axes                     | Vertices  | Description / Notes                                 |
|------------------|---------------------------------|--------------------------|-----------|-----------------------------------------------------|
| Line             | length (optional)               | direction                | 2 points  | Works for 2D or 3D, Z vertices = 0 for 2D           |
|                  |                                 |                          |           | Bounded line if length > 0, infinte = 0             |
| Rectangle        | width, height                   | normal, width, height    |           |                                                     |
| Circle           | radius                          | normal, major            |           |                                                     |
| Triangle         |                                 | normal, in-plane x-y     | 3 points  |                                                     |
| Polygon          |                                 | normal, in-plane x-y     | N points  | Width, and height are wrt. the centroid             |
| Box              | width, depth, height            | x, y, z                  |           |                                                     |
| Cylinder         | radius, height                  | symmetry, ref base plane |           |                                                     |
| Sphere           | radius                          | (none)                   |           |                                                     |
| Cone             | base_radius, top_radius, height | symmetry, ref base plane |           | Frustum if top_radius > 0, cone if = 0              |
| SphericalSegment | base_radius, top_radius, height | symmetry, ref base plane |           | Cap if one radius = 0, segment if both > 0          |
| Plane            | width, height (optional)        | normal, in-plane x-y     |           | Bounded plane if width & height > 0, infinte if = 0 |
| Mesh             | (none)                          | (none, defined in file)  | mesh_path |                                                     |
*/

struct Shape
{
  ShapeType type;
  std::vector<double> dimensions;
  std::vector<Eigen::Vector3d> axes;
  // Optional: additional data (e.g., list of points for polygons)
  std::vector<Eigen::Vector3d> vertices;
  // Optional: mesh reference, etc.
  std::string mesh_path;
  Eigen::Vector3d scale = { 1.0, 1.0, 1.0 };
};

struct ShapeComponent
{
  FlatMap<std::string, Shape> shape_map;
};

}  // namespace components
}  // namespace sodf

#endif
