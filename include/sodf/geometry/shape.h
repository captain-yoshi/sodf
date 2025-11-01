#ifndef SODF_GEOMETRY_SHAPE_H_
#define SODF_GEOMETRY_SHAPE_H_

#include <array>
#include <vector>
#include <string>
#include <type_traits>

#include <Eigen/Geometry>

#include <sodf/geometry/mesh.h>

namespace sodf {
namespace geometry {

// Where to place the local origin (0,0,0) for shapes
enum class OriginPolicy
{
  Native = 0,      // Do not adjust; use vertices/URI frame as-authored.
                   // Cannot be used for primitives with dimensions.
  BaseCenter,      // Base-aligned (zE[0,H]), origin at base center (x_mid,y_mid,z=0).
  AABBCenter,      // Origin at full-shape AABB geometric center.
  VolumeCentroid,  // Origin at (volume/area) centroid (fallback: AABBCenter).

  // BaseMinCornerXY,    // Base at z=0; origin at base AABB min corner (x_min,y_min,0).
  // BaseMaxCornerXY,    // Base at z=0; origin at base AABB max corner (x_max,y_max,0).

};

enum class ShapeType
{
  None,

  // 2D + 3D
  Line,

  // 2D
  Rectangle,
  Circle,
  Triangle,
  Polygon,

  // 3D
  Plane,
  Box,
  TriangularPrism,
  Cylinder,
  Sphere,
  Cone,
  SphericalSegment,
  Mesh,
};

// clang-format off
/*
| ShapeType         | Dimensions                          | Axes (canonical X, Y, Z)   | Vertices           | Description / Notes                                      |
|-------------------|-------------------------------------|----------------------------|--------------------|----------------------------------------------------------|
| Line              | length                              | direction                  | 2 points (2D/3D)   | Works for 2D or 3D. Z vertices = 0 for 2D.               |
|                   |                                     |                            |                    | Bounded if length > 0, infinite if = 0.                  |
| Rectangle         | width, height                       | height, width, normal      | 4 points           |                                                          |
| Circle            | radius                              | reference x-y, normal      | -                  |                                                          |
| Triangle          | base, altitude, apex_offset         | altitude, base, normal     | 3 points           |                                                          |
| Polygon           | - (dimentionless)                   | x, y, normal               | N points           | Width and height are wrt. the centroid.                  |
| Plane             | width, height (optional)            | reference x-y, normal,     | -                  | Bounded plane if width & height > 0, infinite if = 0.    |
| Box               | width, depth, height                | depth, width, height       | -                  |                                                          |
| TriangularPrism   | base, altitude, height, apex_offset | altitude, base, height     | -                  | Base A=(0,0), B=(bw,0), C=(u,bh) in base plane.          |
| Cylinder          | radius, height                      | reference x-y, symmetry    | -                  |                                                          |
| Sphere            | radius                              | (none)                     | -                  |                                                          |
| Cone              | base_radius, top_radius, height     | reference x-y, symmetry    | -                  | Frustum if top_radius > 0, cone if = 0.                  |
| SphericalSegment  | base_radius, top_radius, height     | reference x-y, symmetry    | -                  | Cap if one radius = 0, segment if both > 0.              |
| Mesh              | - (dimensionless)                   | x, y, z                    | External URI or    | Use as-authored local frame, or override with AxisX/Y/Z; |
|                   |                                     |                            | TriangleMesh (V,F) | Supports indexed triangles; units = meters               |
*/
// clang-format on

struct Shape
{
  ShapeType type = ShapeType::None;
  std::vector<double> dimensions;
  std::vector<Eigen::Vector3d> axes;
  std::vector<Eigen::Vector3d> vertices;  // line, polygon bases...
  Eigen::Vector3d scale{ 1, 1, 1 };       // per-instance scale
  MeshSource mesh;                        // choose internal or external

  OriginPolicy origin = OriginPolicy::Native;
};

struct StackedShapeEntry
{
  geometry::Shape shape;

  // Pose of this shape expressed in the stack/base frame.
  Eigen::Isometry3d base_transform;
};

struct StackedShape
{
  std::vector<geometry::StackedShapeEntry> shapes;

  // Eigen::Vector3d axis_stack_direction;
  // Eigen::Vector3d axis_stack_reference;
};

inline bool isPrimitive(geometry::ShapeType t)
{
  using geometry::ShapeType;
  switch (t)
  {
    case ShapeType::None:
    case ShapeType::Mesh:
      return false;
    default:
      return true;
  }
}

inline bool isPrimitive2D(geometry::ShapeType t)
{
  using geometry::ShapeType;
  switch (t)
  {
    case ShapeType::Line:  // 2D or 3D; counted as 2D-capable
    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Polygon:
    case ShapeType::Triangle:
    case ShapeType::Plane:
      return true;
    default:
      return false;
  }
}

inline bool isPrimitive3D(geometry::ShapeType t)
{
  using geometry::ShapeType;
  switch (t)
  {
    case ShapeType::Line:  // 2D or 3D; counted as 3D-capable
    case ShapeType::Box:
    case ShapeType::TriangularPrism:
    case ShapeType::Cylinder:
    case ShapeType::Sphere:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      return true;
    default:
      return false;
  }
}

inline bool isPrimitive(const geometry::Shape& s)
{
  return isPrimitive(s.type);
}
inline bool isPrimitive2D(const geometry::Shape& s)
{
  return isPrimitive2D(s.type);
}
inline bool isPrimitive3D(const geometry::Shape& s)
{
  return isPrimitive3D(s.type);
}

inline bool primitiveHasDimensions(const geometry::Shape& s)
{
  return !s.dimensions.empty();
}
inline bool primitiveHasVertices(const geometry::Shape& s)
{
  return !s.vertices.empty();
}

// Dimension-less by SPEC: shapes that are not defined via `dimensions[]`.
inline bool isDimensionless(geometry::ShapeType t)
{
  using geometry::ShapeType;
  switch (t)
  {
    case ShapeType::Polygon:
    case ShapeType::Triangle:
    case ShapeType::Mesh:
    case ShapeType::None:
      return true;  // not dimension-led by design
    default:
      return false;  // supports/uses dimensions
  }
}

// Dimension-less by INSTANCE: either the type is dimension-less by spec,
// or this instance simply has no dimensions filled in.
inline bool isDimensionless(const geometry::Shape& s)
{
  if (isDimensionless(s.type))
    return true;                // spec says no dims
  return s.dimensions.empty();  // no dims provided in this instance
}

inline bool hasExternalMesh(const Shape& s)
{
  return std::holds_alternative<MeshRef>(s.mesh);
}
inline bool hasInlineMesh(const Shape& s)
{
  return std::holds_alternative<InlineMeshPtr>(s.mesh);
}
inline const MeshRef* getExternalMesh(const Shape& s)
{
  return std::get_if<MeshRef>(&s.mesh);
}
inline const TriangleMesh* getInlineMesh(const Shape& s)
{
  if (const auto p = std::get_if<TriangleMeshPtr>(&s.mesh))
    return p->get();
  return nullptr;
}

bool is2DShape(const Shape& shape);
double getShapeHeight(const Shape& shape);

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

inline double inferBaseRadiusFromHeight(double r2, double h)
{
  if (r2 < 0.0 || h <= 0.0)
    throw std::invalid_argument("Top radius must be non-negative and height > 0");

  // Mirror of inferTopRadiusFromHeight:
  // Use the same sphere radius estimate and advance by pm h/R in angle space.
  // R = (r^2 + h^2) / (2h)
  const double R = (r2 * r2 + h * h) / (2.0 * h);

  if (!(R > 0.0) || !std::isfinite(R))
    throw std::invalid_argument("Invalid curvature derived from top radius and height");

  // theta2 = asin(r2 / R);  phi1 = theta2 - h/R  -> r1 = R * sin(phi1)
  const double x = std::clamp(r2 / R, -1.0, 1.0);
  const double theta2 = std::asin(x);
  const double phi1 = theta2 - (h / R);

  const double r1 = R * std::sin(phi1);
  if (r1 < 0.0)
    return 0.0;  // guard tiny negatives from FP error
  return r1;
}

Eigen::Vector3d getShapeCentroid(const Shape& shape);
ShapeType shapeTypeFromString(const std::string& str);
std::string shapeTypeToString(ShapeType type);

// Helpers ---------------------------------------------------------------------

inline const char* originPolicyToString(OriginPolicy p)
{
  switch (p)
  {
    case OriginPolicy::Native:
      return "Native";
    case OriginPolicy::BaseCenter:
      return "BaseCenter";
    case OriginPolicy::AABBCenter:
      return "AABBCenter";
    case OriginPolicy::VolumeCentroid:
      return "VolumeCentroid";
    default:
      return "<unknown-origin>";
  }
}

// Canonical semantic labels for axes per shape type.
// Returns a span-like triplet of const char* for (X,Y,Z).
struct AxisLabels
{
  const char* X;
  const char* Y;
  const char* Z;
};

inline AxisLabels shapeAxisLabels(ShapeType t)
{
  using ST = ShapeType;
  switch (t)
  {
    case ST::Rectangle:  // axes = [Height (X), Width (Y), Normal (Z)]
    case ST::Triangle:
    case ST::Polygon:
      return { "Height", "Width", "Normal" };
    case ST::Circle:
      return { "RefX", "RefY", "Normal" };
    case ST::Plane:
      return { "RefX", "RefY", "Normal" };
    case ST::Box:  // axes = [Depth (X), Width (Y), Height (Z)]
      return { "Depth", "Width", "Height" };
    case ST::TriangularPrism:  // axes = [Altitude (X), Base (Y), Height (Z)]
      return { "Altitude", "Base", "Height" };
    case ST::Cylinder:  // axes = [RefX, RefY, Symmetry]
    case ST::Cone:
    case ST::SphericalSegment:
      return { "RefX", "RefY", "Symmetry" };
    case ST::Mesh:
      return { "X", "Y", "Z" };  // mesh frame as-authored unless overridden
    case ST::Line:
      return { "Dir", "(n/a)", "(n/a)" };
    case ST::Sphere:
      return { "(none)", "(none)", "(none)" };
    default:
      return { "X", "Y", "Z" };
  }
}

// Printers --------------------------------------------------------------------

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
  // Fixed formatting for compact, deterministic logs
  auto flags = os.flags();
  auto prec = os.precision();
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(6);

  os << "Shape(type=" << shape.type << ", origin=" << originPolicyToString(shape.origin) << ", scale=("
     << shape.scale.x() << ", " << shape.scale.y() << ", " << shape.scale.z() << ")";

  // dimensions
  os << ", dimensions=[";
  for (size_t i = 0; i < shape.dimensions.size(); ++i)
  {
    os << shape.dimensions[i];
    if (i + 1 < shape.dimensions.size())
      os << ", ";
  }
  os << "]";

  // axes with semantic labels
  const auto labels = shapeAxisLabels(shape.type);
  os << ", axes={";
  if (!shape.axes.empty())
  {
    // Print up to first 3 axes, labelled
    const size_t n = std::min<size_t>(shape.axes.size(), 3);
    for (size_t i = 0; i < n; ++i)
    {
      const auto& a = shape.axes[i];
      const char* name = (i == 0 ? labels.X : i == 1 ? labels.Y : labels.Z);
      os << name << "=(" << a.x() << ", " << a.y() << ", " << a.z() << ")";
      if (i + 1 < n)
        os << ", ";
    }
    if (shape.axes.size() > 3)
      os << ", ...+" << (shape.axes.size() - 3);
  }
  os << "}";

  // vertices (limit)
  os << ", vertices=[";
  const size_t max_show = 4;
  for (size_t i = 0; i < shape.vertices.size() && i < max_show; ++i)
  {
    const auto& v = shape.vertices[i];
    os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    if (i + 1 < std::min(shape.vertices.size(), max_show))
      os << ", ";
  }
  if (shape.vertices.size() > max_show)
    os << ", ...+" << (shape.vertices.size() - max_show);
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

  os << ")";
  // restore stream state
  os.flags(flags);
  os.precision(prec);
  return os;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_SHAPE_H_
