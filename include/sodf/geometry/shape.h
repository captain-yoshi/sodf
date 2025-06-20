#ifndef SODF_GEOMETRY_SHAPE_H_
#define SODF_GEOMETRY_SHAPE_H_

#include <sodf/geometry/eigen.h>

namespace sodf {
namespace geometry {

// clang-format off
/*
| ShapeType         | Dimensions                      | Axes                       | Vertices   | Description / Notes                                        |
|-------------------|---------------------------------|----------------------------|------------|------------------------------------------------------------|
| Line              | length (optional)               | direction                  | 2 points   | Works for 2D or 3D. Z vertices = 0 for 2D.                 |
|                   |                                 |                            |            | Bounded if length > 0, infinite if = 0.                    |
| Rectangle         | width, height                   | normal, width, height      |            |                                                            |
| Circle            | radius                          | normal, major              |            |                                                            |
| Triangle          |                                 | normal, in-plane x-y       | 3 points   |                                                            |
| Polygon           |                                 | normal, in-plane x-y       | N points   | Width and height are wrt. the centroid.                    |
| Box               | width, depth, height            | width, depth, height       |            |                                                            |
| Cylinder          | radius, height                  | symmetry, ref base plane   |            |                                                            |
| Sphere            | radius                          | (none)                     |            |                                                            |
| Cone              | base_radius, top_radius, height | symmetry, ref base plane   |            | Frustum if top_radius > 0, cone if = 0.                    |
| SphericalSegment  | base_radius, top_radius, height | symmetry, ref base plane   |            | Cap if one radius = 0, segment if both > 0.                |
| Plane             | width, height (optional)        | normal, in-plane x-y       |            | Bounded plane if width & height > 0, infinite if = 0.      |
| Mesh              | (none)                          | (none, defined in file)    | mesh_path  |                                                            |
*/
// clang-format on

enum class ShapeType
{
  // 2D + 3D
  Line,

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
  SphericalSegment,
  Plane,
  Mesh,
};

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

bool is2DShape(const Shape& shape);
double shapeHeight(const Shape& shape);

double shapeBaseRadius(const geometry::Shape& shape);
double shapeTopRadius(const geometry::Shape& shape);
double shapeMaxRadius(const geometry::Shape& shape);

const Eigen::Vector3d& getShapeNormalAxis(const Shape& shape);
const Eigen::Vector3d& getShapeReferenceAxis(const Shape& shape);
const Eigen::Vector3d& getShapeSymmetryAxis(const Shape& shape);

geometry::Shape truncateShapeToHeight(const geometry::Shape& shape, double new_height);

static bool isValidSegment(double r1, double r2, double h, double epsilon = 1e-12)
{
  if (r1 < 0 || r2 < 0 || h <= 0)
    return false;

  return true;
}

inline double inferSegmentHeightFromRadii(double r1, double r2)
{
  if (r1 < 0 || r2 < 0)
    throw std::invalid_argument("Radii must be non-negative");

  if (r1 == r2)
    throw std::invalid_argument("Radii are equal: height is undefined for flat spherical cap");

  // Solve for h such that sphere exists
  // R = (r2² + h² - r1²) / (2h)
  // Rearranged into: 2Rh = r2² + h² - r1² ⇒ Quadratic in h

  double num = r2 * r2 - r1 * r1;
  double den = 2 * std::sqrt((r1 * r1 + r2 * r2) / 2.0);  // Safe initial guess for R
  double h = std::abs(num / den);                         // Approximate; could refine if needed

  // Or: Solve directly via full expression
  return (r2 * r2 - r1 * r1) / (2 * std::sqrt(std::max(r1 * r1, r2 * r2)));
}

inline double inferTopRadiusFromHeight(double r1, double h)
{
  if (r1 < 0 || h <= 0)
    throw std::invalid_argument("Base radius must be non-negative and height > 0");

  // From: R = (r2² + h² - r1²) / (2h) ⇒ isolate r2
  // Assume same sphere used for both ends
  double R = (r1 * r1 + h * h) / (2 * h);

  if (R <= 0 || !std::isfinite(R))
    throw std::invalid_argument("Invalid curvature derived from base radius and height");

  double theta = std::asin(r1 / R);
  double phi = theta + h / R;
  return R * std::sin(phi);
}

Eigen::Vector3d getShapeCentroid(const Shape& shape);
ShapeType shapeTypeFromString(const std::string& str);
std::string shapeTypeToString(ShapeType type);

inline std::ostream& operator<<(std::ostream& os, ShapeType type)
{
  return os << shapeTypeToString(type);
}

inline std::ostream& operator<<(std::ostream& os, const Shape& shape)
{
  os << "Shape(type=" << shape.type << ", dimensions=[";
  for (size_t i = 0; i < shape.dimensions.size(); ++i)
  {
    os << shape.dimensions[i];
    if (i + 1 < shape.dimensions.size())
      os << ", ";
  }
  os << "], axes=[";
  for (size_t i = 0; i < shape.axes.size(); ++i)
  {
    os << shape.axes[i];
    if (i + 1 < shape.axes.size())
      os << ", ";
  }
  os << "], vertices=[";
  for (size_t i = 0; i < shape.vertices.size(); ++i)
  {
    os << shape.vertices[i];
    if (i + 1 < shape.vertices.size())
      os << ", ";
  }
  os << "], mesh_path='" << shape.mesh_path << "'";
  os << ", scale=" << shape.scale.transpose() << ")";
  return os;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_SHAPE_H_
