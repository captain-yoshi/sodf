#ifndef SODF_GEOMETRY_SHAPE_H_
#define SODF_GEOMETRY_SHAPE_H_

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <variant>
#include <type_traits>

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
| TriangularPrism   | width, depth, height, offset    | width, depth, height       |            | Base A=(0,0), B=(bw,0), C=(u,bh) in base plane.            |
| Cylinder          | radius, height                  | symmetry, ref base plane   |            |                                                            |
| Sphere            | radius                          | (none)                     |            |                                                            |
| Cone              | base_radius, top_radius, height | symmetry, ref base plane   |            | Frustum if top_radius > 0, cone if = 0.                    |
| SphericalSegment  | base_radius, top_radius, height | symmetry, ref base plane   |            | Cap if one radius = 0, segment if both > 0.                |
| Plane             | width, height (optional)        | normal, in-plane x-y       |            | Bounded plane if width & height > 0, infinite if = 0.      |


| Mesh              | Parameters       | Axes                    | Description / Notes                                      |
| ------------------------------------ | ----------------------- | -------------------------------------------------------- |
| External          | uri              | (none, defined in file) | Can be loaded via Assimp or simplify handeled externally |
| Inline            | vertices, faces  | (none)                  | Direct in-memory mesh                                    |
*/
// clang-format on

enum class ShapeType
{
  None,

  // 2D + 3D
  Line,

  // 2D
  Rectangle,
  Circle,
  Polygon,
  Triangle,

  // 3D
  Box,
  TriangularPrism,
  Cylinder,
  Sphere,
  Cone,
  SphericalSegment,
  Plane,
  Mesh,
};

struct TriangleMesh
{
  using Index = uint32_t;
  using Face = std::array<Index, 3>;

  std::vector<Eigen::Vector3d> V;  // vertcies
  std::vector<Face> F;             // CCW/outward
};

struct MeshRef
{
  std::string uri;  // asset identifier only (no scale)
};

using TriangleMeshPtr = std::shared_ptr<const TriangleMesh>;
using InlineMeshPtr = std::shared_ptr<const TriangleMesh>;

using MeshSource = std::variant<std::monostate,  // no mesh
                                MeshRef,         // external mesh by URI
                                InlineMeshPtr>;  // inline mesh (heap)
struct Shape
{
  ShapeType type = ShapeType::None;
  std::vector<double> dimensions;
  std::vector<Eigen::Vector3d> axes;
  std::vector<Eigen::Vector3d> vertices;  // line, polygon bases...
  Eigen::Vector3d scale{ 1, 1, 1 };       // per-instance scale
  MeshSource mesh;                        // choose internal or external
};

struct StackedShapeEntry
{
  geometry::Shape shape;

  // relative_transform[i] encodes a pure yaw+z transform from
  // top of segment (i-1) to base of segment i.
  Eigen::Isometry3d relative_transform;
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

  // From: R = (r2^2 + h^2 - r1^2) / (2h) -> isolate r2
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

inline std::ostream& operator<<(std::ostream& os, const MeshRef& mref)
{
  os << "MeshRef(uri='" << mref.uri << "')";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const TriangleMesh& M)
{
  os << "IndexedTriMesh(V=" << M.V.size() << ", F=" << M.F.size() << ")";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Shape& shape)
{
  os << "Shape(type=" << shape.type;

  // dimensions
  os << ", dimensions=[";
  for (size_t i = 0; i < shape.dimensions.size(); ++i)
  {
    os << shape.dimensions[i];
    if (i + 1 < shape.dimensions.size())
      os << ", ";
  }
  os << "]";

  // axes
  os << ", axes=[";
  for (size_t i = 0; i < shape.axes.size(); ++i)
  {
    const auto& a = shape.axes[i];
    os << "(" << a.x() << ", " << a.y() << ", " << a.z() << ")";
    if (i + 1 < shape.axes.size())
      os << ", ";
  }
  os << "]";

  // vertices
  os << ", vertices=[";
  // print up to first 4 vertices to keep logs readable
  const size_t max_show = 4;
  for (size_t i = 0; i < shape.vertices.size() && i < max_show; ++i)
  {
    const auto& v = shape.vertices[i];
    os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    if (i + 1 < std::min(shape.vertices.size(), max_show))
      os << ", ";
  }
  if (shape.vertices.size() > max_show)
  {
    os << ", ... (+" << (shape.vertices.size() - max_show) << ")";
  }
  os << "]";

  // mesh (variant)
  os << ", mesh=";
  std::visit(
      [&](const auto& alt) {
        using T = std::decay_t<decltype(alt)>;
        if constexpr (std::is_same_v<T, std::monostate>)
        {
          os << "None";
        }
        else if constexpr (std::is_same_v<T, MeshRef>)
        {
          os << alt;  // MeshRef printer
        }
        else if constexpr (std::is_same_v<T, std::unique_ptr<const TriangleMesh>>)
        {
          if (alt)
            os << *alt;
          else
            os << "InlineMesh(nullptr)";
        }
        else
        {
          os << "<unknown-mesh-alt>";
        }
      },
      shape.mesh);

  // scale
  os << ", scale=(" << shape.scale.x() << ", " << shape.scale.y() << ", " << shape.scale.z() << ")";

  os << ")";
  return os;
}

inline bool hasExternal(const Shape& s)
{
  return std::holds_alternative<MeshRef>(s.mesh);
}
inline bool hasInline(const Shape& s)
{
  return std::holds_alternative<InlineMeshPtr>(s.mesh);
}
inline const MeshRef* ext(const Shape& s)
{
  return std::get_if<MeshRef>(&s.mesh);
}
inline const TriangleMesh* inl(const Shape& s)
{
  if (const auto p = std::get_if<TriangleMeshPtr>(&s.mesh))
    return p->get();
  return nullptr;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_SHAPE_H_
