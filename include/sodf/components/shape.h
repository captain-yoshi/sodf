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
  Rectangle,
  Circle,
  Polygon,
  Triangle,

  // 3D
  Box,
  Cylinder,
  Sphere,
  Cone,
  Plane,
  Mesh,
};

/*
    | ShapeType | Dimensions                        | Axes                               | Vertices   |
    |-----------|-----------------------------------|------------------------------------|------------|
    | Rectangle | width, height                     | normal, width, height              |            |
    | Circle    | radius                            | normal, major                      |            |
    | Triangle  |                                   | normal, in-plane x, in-plane y     | 3 points   |
    | Polygon   |                                   | normal, in-plane x, in-plane y     | N points   |
    | Box       | width, height, depth              | x, y, z                            |            |
    | Cylinder  | radius, height                    | symmetry, reference in base plane  |            |
    | Sphere    | radius                            | (none)                             |            |
    | Cone      | base_radius, top_radius*, height  | symmetry, reference in base plane  |            |
    | Plane     | (optional: width, height)         | normal, in-plane x, in-plane y     |            |
    | Mesh      | (none)                            | (none)                             | mesh_path  |

    *top_radius = 0 for a perfect cone, > 0 for a frustum
*/

struct Shape
{
  ShapeType type;
  std::vector<double> dimensions;
  std::vector<Eigen::Vector3d> axes;
  // Optional: additional data (e.g., list of points for polygons)
  std::vector<Eigen::Vector2d> vertices_2d;  // For polygons, triangles in 2D plane
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
