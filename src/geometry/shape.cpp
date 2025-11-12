#include <sodf/geometry/shape.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <iostream>

namespace sodf {
namespace geometry {

bool isPrimitive(geometry::ShapeType t)
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

bool isPrimitive2D(geometry::ShapeType t)
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

bool isPrimitive3D(geometry::ShapeType t)
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

bool isPrimitive(const geometry::Shape& s)
{
  return isPrimitive(s.type);
}
bool isPrimitive2D(const geometry::Shape& s)
{
  return isPrimitive2D(s.type);
}
bool isPrimitive3D(const geometry::Shape& s)
{
  return isPrimitive3D(s.type);
}

bool primitiveHasDimensions(const geometry::Shape& s)
{
  return !s.dimensions.empty();
}
bool primitiveHasVertices(const geometry::Shape& s)
{
  return !s.vertices.empty();
}

// Dimension-less by SPEC: shapes that are not defined via `dimensions[]`.
bool isDimensionless(geometry::ShapeType t)
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
bool isDimensionless(const geometry::Shape& s)
{
  if (isDimensionless(s.type))
    return true;                // spec says no dims
  return s.dimensions.empty();  // no dims provided in this instance
}

bool hasExternalMesh(const Shape& s)
{
  return std::holds_alternative<MeshRef>(s.mesh);
}
bool hasInlineMesh(const Shape& s)
{
  return std::holds_alternative<InlineMeshPtr>(s.mesh);
}
const MeshRef* getExternalMesh(const Shape& s)
{
  return std::get_if<MeshRef>(&s.mesh);
}
const TriangleMesh* getInlineMesh(const Shape& s)
{
  if (const auto p = std::get_if<TriangleMeshPtr>(&s.mesh))
    return p->get();
  return nullptr;
}

const Eigen::Vector3d& getShapePrimaryAxis(const Shape& shape)
{
  // Stable, by-reference fallback (never returns a dangling reference).
  static const Eigen::Vector3d kFallback(1.0, 0.0, 0.0);

  if (!shape.axes.empty())
    return shape.axes.front();

  // No axis provided: return a deterministic fallback.
  // For Mesh this is intentionally neutral; caller should set axes[0] explicitly
  // if a semantic axis is required.
  return kFallback;
}

const Eigen::Vector3d& getShapeUAxis(const Shape& shape)
{
  static const Eigen::Vector3d kFallback(0.0, 1.0, 0.0);  // +Y
  if (shape.axes.size() > 1)
    return shape.axes[1];
  return kFallback;
}

const Eigen::Vector3d& getShapeVAxis(const Shape& shape)
{
  static const Eigen::Vector3d kFallback(0.0, 0.0, 1.0);  // +Z
  if (shape.axes.size() > 2)
    return shape.axes[2];
  return kFallback;
}

Eigen::Vector3d getShapeCentroid(const Shape& shape)
{
  const auto& type = shape.type;
  const auto& vertices = shape.vertices;
  const auto& dimensions = shape.dimensions;

  switch (type)
  {
    case ShapeType::Line:
    {
      if (vertices.size() < 2)
        throw std::runtime_error("Line shape requires two vertices for centroid calculation.");
      return 0.5 * (vertices.at(0) + vertices.at(1));
    }

    case ShapeType::Triangle:
    case ShapeType::Polygon:
    {
      if (vertices.empty())
        throw std::runtime_error("Polygon/Triangle shape requires at least one vertex for centroid calculation.");
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (const auto& v : vertices)
        c += v;
      return c / static_cast<double>(vertices.size());
    }

    case ShapeType::TriangularPrism:
    {
      // dims = { base_length (along V), altitude (along U), height (along N), [apex_offset u (along U)] }
      if (dimensions.size() < 3)
        throw std::runtime_error("TriangularPrism requires dims {base, altitude, height[, apex_offset]}.");

      const double bw = dimensions[0];                                  // along V (Base axis)
      const double bh = dimensions[1];                                  // along U (Altitude axis)
      const double h = dimensions[2];                                   // along N (Height/extrusion)
      const double u = (dimensions.size() >= 4 ? dimensions[3] : 0.0);  // apex offset along U

      const Eigen::Vector3d& U = getShapeUAxis(shape);        // Altitude axis in base plane
      const Eigen::Vector3d& V = getShapeVAxis(shape);        // Base axis in base plane
      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);  // Extrusion/Height axis

      // Base triangle vertices in (U,V) plane:
      // A = 0, B = bw * V, C = u * U + bh * V  -> centroid = (A + B + C)/3
      const Eigen::Vector3d base_centroid = ((u / 3.0) * U) + (((bw + bh) / 3.0) * V);

      // Extrusion centroid offset = h/2 along N
      return base_centroid + (0.5 * h) * N;
    }

    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Plane:
    case ShapeType::Box:
    case ShapeType::Sphere:
      // By convention these primitives are centered at the origin in their canonical frames.
      return Eigen::Vector3d::Zero();

    case ShapeType::Cylinder:
    {
      // dims = { radius, height }, centroid at h/2 along the symmetry/extrusion axis
      if (dimensions.size() < 2)
        throw std::runtime_error("Cylinder requires dims {radius, height}.");
      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);
      return (0.5 * dimensions[1]) * N;
    }

    case ShapeType::Cone:
    {
      // dims = { base_radius, top_radius, height }
      if (dimensions.size() < 3)
        throw std::runtime_error("Cone requires dims {base_radius, top_radius, height}.");
      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);
      // Keep the prior heuristic: centroid ~ h/4 from base along axis (exact for a right cone).
      return (0.25 * dimensions[2]) * N;
    }

    case ShapeType::SphericalSegment:
    {
      // dims = { base_radius, top_radius, height }
      if (dimensions.size() < 3)
        throw std::runtime_error("SphericalSegment requires dims {base_radius, top_radius, height}.");

      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);
      const double r1 = dimensions[0];
      const double r2 = dimensions[1];
      const double h = dimensions[2];

      // Use the larger rim radius to estimate sphere R (matches your previous approach).
      const double R = std::max(r1, r2);
      // Centroid distance from the base plane along N (same closed form you had before):
      const double z = (3.0 * R - h) * h * h / (4.0 * (3.0 * R * R - 3.0 * R * h + h * h));
      return z * N;
    }

    case ShapeType::Mesh:
      throw std::runtime_error("getShapeCentroid() not implemented for Mesh.");

    default:
      throw std::runtime_error("getShapeCentroid() not implemented for this ShapeType.");
  }
}

double getShapeHeight(const Shape& s)
{
  using ST = ShapeType;

  auto need = [&](std::size_t n, const char* what) {
    if (s.dimensions.size() < n)
      throw std::runtime_error(std::string(what) + " expects at least " + std::to_string(n) + " dimensions");
  };

  switch (s.type)
  {
    // -------- 2D (height along canonical +Z) --------
    case ST::Rectangle:
      need(2, "Rectangle");
      return s.dimensions[1];  // size_z
    case ST::Plane:
      need(2, "Plane");
      return s.dimensions[1];  // size_z
    case ST::Circle:
      need(1, "Circle");
      return 2.0 * s.dimensions[0];  // diameter on Z
    case ST::Triangle:
      need(2, "Triangle");
      return s.dimensions[1];  // altitude_z
    case ST::Polygon:          // vertices in YZ; AABB Z-extent
    {
      if (s.vertices.empty())
        return 0.0;
      double zmin = std::numeric_limits<double>::infinity();
      double zmax = -zmin;
      for (const auto& p : s.vertices)
      {
        zmin = std::min(zmin, p.z());
        zmax = std::max(zmax, p.z());
      }
      return std::isfinite(zmin) && std::isfinite(zmax) ? (zmax - zmin) : 0.0;
    }
    case ST::Line:
    {
      if (s.vertices.size() == 2)
        return std::abs(s.vertices[1].z() - s.vertices[0].z());
      if (!s.dimensions.empty())
        return std::abs(s.dimensions[0]);  // fallback
      return 0.0;
    }

    // -------- 3D (height along canonical +X) --------
    case ST::Box:
      need(1, "Box");
      return s.dimensions[0];  // x
    case ST::TriangularPrism:
      need(1, "TriangularPrism");
      return s.dimensions[0];  // length_x
    case ST::Cylinder:
      need(2, "Cylinder");
      return s.dimensions[0];  // height_x (NEW order)
    case ST::Cone:
      need(3, "Cone");
      return s.dimensions[0];  // height_x (NEW order)
    case ST::SphericalSegment:
      need(3, "SphericalSegment");
      return s.dimensions[0];  // height_x (NEW order)
    case ST::Sphere:
      need(1, "Sphere");
      return 2.0 * s.dimensions[0];

    case ST::Mesh:
      // Unknown here unless inline verts available; keep your previous behavior
      throw std::runtime_error("getShapeHeight: unknown height for Mesh without inline vertices");

    case ST::None:
    default:
      throw std::runtime_error("getShapeHeight: unsupported shape type");
  }
}

double getShapeBaseRadius(const geometry::Shape& shape)
{
  using geometry::ShapeType;

  switch (shape.type)
  {
    case ShapeType::Cylinder:
    case ShapeType::Sphere:
    case ShapeType::Circle:
      // Single radius shapes: use "radius"
      return shape.dimensions.at(0);

    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      // Use base_radius if defined, else fallback to radius
      return shape.dimensions.at(0);

    case ShapeType::Box:
      return 0.5 * std::max({ shape.dimensions.at(0), shape.dimensions.at(1) });

    default:
      return 0.0;
  }
}

double getShapeTopRadius(const geometry::Shape& shape)
{
  using geometry::ShapeType;

  switch (shape.type)
  {
    case ShapeType::Cylinder:
    case ShapeType::Sphere:
    case ShapeType::Circle:
      // Single radius shapes: use "radius"
      return shape.dimensions.at(0);

    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      // Use top_radius if defined, else fallback to radius
      return shape.dimensions.at(1);

    case ShapeType::Box:
      return 0.5 * std::max({ shape.dimensions.at(0), shape.dimensions.at(1) });

    default:
      return 0.0;
  }
}

double getShapeMaxRadius(const geometry::Shape& shape)
{
  using geometry::ShapeType;

  switch (shape.type)
  {
    case ShapeType::Cylinder:
    case ShapeType::Sphere:
    case ShapeType::Circle:
      return shape.dimensions.at(0);  // radius

    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      return std::max(shape.dimensions.at(0), shape.dimensions.at(1));  // base_radius, top_radius

    case ShapeType::Box:
      return 0.5 * std::max({ shape.dimensions.at(0), shape.dimensions.at(1) });  // width, depth

    default:
      return 0.0;
  }
}

double getTopRadiusAtHeight(double base_radius, double top_radius, double total_height, double new_height)
{
  if (new_height <= 0.0 || new_height > total_height)
    return 0.0;

  // Compute z0 (sphere center relative to base)
  double z0 =
      (top_radius * top_radius - base_radius * base_radius + total_height * total_height) / (2.0 * total_height);
  double R = std::sqrt(base_radius * base_radius + z0 * z0);

  // Compute radial distance at given fill height
  double dz = z0 - new_height;
  double top_r = std::sqrt(std::max(0.0, R * R - dz * dz));

  return top_r;
}

geometry::Shape truncateShapeToHeight(const geometry::Shape& shape, double new_height)
{
  geometry::Shape result = shape;

  switch (shape.type)
  {
    case geometry::ShapeType::TriangularPrism:
    {
      if (result.dimensions.size() < 3)
        throw std::runtime_error("TriangularPrism expects at least 3 dimensions for truncation");
      if (new_height <= 0.0)
        throw std::runtime_error("TriangularPrism truncation: new height must be > 0");
      result.dimensions[2] = new_height;  // update depth; keep u unchanged
      break;
    }
    case geometry::ShapeType::Cylinder:
      result.dimensions.at(1) = new_height;
      break;

    case geometry::ShapeType::Cone:
    {
      double h = shape.dimensions.at(2);
      double base_r = shape.dimensions.at(0);
      double top_r = shape.dimensions.at(1);

      double t = new_height / h;
      double new_r = base_r + (top_r - base_r) * t;

      result.dimensions.at(1) = new_r;       // new top_radius
      result.dimensions.at(2) = new_height;  // new height
      break;
    }

    case geometry::ShapeType::SphericalSegment:
    {
      double h = shape.dimensions.at(2);
      double base_r = shape.dimensions.at(0);
      double top_r = shape.dimensions.at(1);

      double clipped_r = getTopRadiusAtHeight(base_r, top_r, h, new_height);

      result.dimensions.at(0) = base_r;
      result.dimensions.at(1) = clipped_r;
      result.dimensions.at(2) = new_height;

      std::cout << "[SphericalSegment Truncation]" << std::endl;
      std::cout << "  base_r     = " << base_r << std::endl;
      std::cout << "  top_r      = " << top_r << std::endl;
      std::cout << "  h          = " << h << std::endl;
      std::cout << "  new_height = " << new_height << std::endl;
      std::cout << "  new_r      = " << clipped_r << std::endl;
      break;
    }

    default:
      result.dimensions.clear();  // mark invalid if not supported
      break;
  }

  return result;
}

double inferSegmentHeightFromRadii(double r1, double r2)
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

double inferTopRadiusFromHeight(double r1, double h)
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

double inferBaseRadiusFromHeight(double r2, double h)
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

bool isValidSegment(double r1, double r2, double h, double epsilon)
{
  // Guard NaNs/infinities (optional but helpful).
  if (!std::isfinite(r1) || !std::isfinite(r2) || !std::isfinite(h))
    return false;

  // Treat tiny negatives as 0, but reject truly negative beyond epsilon.
  if (r1 < -epsilon || r2 < -epsilon)
    return false;

  // Height must be strictly positive beyond epsilon.
  if (h <= epsilon)
    return false;

  return true;
}

ShapeType shapeTypeFromString(const std::string& str)
{
  if (str == "Rectangle")
    return ShapeType::Rectangle;
  if (str == "Circle")
    return ShapeType::Circle;
  if (str == "Triangle")
    return ShapeType::Triangle;
  if (str == "Polygon")
    return ShapeType::Polygon;
  if (str == "Box")
    return ShapeType::Box;
  if (str == "TriangularPrism")
    return ShapeType::TriangularPrism;
  if (str == "Cylinder")
    return ShapeType::Cylinder;
  if (str == "Sphere")
    return ShapeType::Sphere;
  if (str == "Cone")
    return ShapeType::Cone;
  if (str == "SphericalSegment")
    return ShapeType::SphericalSegment;
  if (str == "Mesh")
    return ShapeType::Mesh;
  if (str == "Plane")
    return ShapeType::Plane;
  if (str == "Line")
    return ShapeType::Line;
  throw std::runtime_error("Unknown ShapeType: " + str);
}

std::string shapeTypeToString(ShapeType type)
{
  switch (type)
  {
    case ShapeType::Rectangle:
      return "Rectangle";
    case ShapeType::Circle:
      return "Circle";
    case ShapeType::Triangle:
      return "Triangle";
    case ShapeType::Polygon:
      return "Polygon";
    case ShapeType::Box:
      return "Box";
    case ShapeType::TriangularPrism:
      return "TriangularPrism";
    case ShapeType::Cylinder:
      return "Cylinder";
    case ShapeType::Sphere:
      return "Sphere";
    case ShapeType::Cone:
      return "Cone";
    case ShapeType::SphericalSegment:
      return "SphericalSegment";
    case ShapeType::Mesh:
      return "Mesh";
    case ShapeType::Plane:
      return "Plane";
    case ShapeType::Line:
      return "Line";
    default:
      throw std::runtime_error("Unknown ShapeType enum value");
  }
}

const char* originPolicyToString(OriginPolicy p)
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

AxisLabels shapeAxisLabels(ShapeType t)
{
  using ST = ShapeType;
  switch (t)
  {
    // 2D primitives (X = Normal, Y/Z = in-plane refs)
    case ST::Rectangle:  // axes = [Normal (X), SizeY (Y), SizeZ (Z)]
      return { "Normal", "RefY", "RefZ" };
    case ST::Triangle:  // axes = [Normal, Base, Altitude]
      return { "Normal", "Base", "Altitude" };
    case ST::Polygon:  // axes = [Normal, Y, Z]
      return { "Normal", "RefY", "RefZ" };
    case ST::Circle:  // axes = [Normal, RefY, RefZ]
      return { "Normal", "RefY", "RefZ" };
    case ST::Plane:  // axes = [Normal, RefY, RefZ]
      return { "Normal", "RefY", "RefZ" };

    // 3D primitives
    case ST::Box:  // axes = [X, Y, Z]
      return { "X", "Y", "Z" };
    case ST::TriangularPrism:  // axes = [Extrusion (X), Base (Y), Altitude (Z)]
      return { "Extrusion", "Base", "Altitude" };
    case ST::Cylinder:  // axes = [Symmetry, RefY, RefZ]
      return { "Symmetry", "RefY", "RefZ" };
    case ST::Cone:  // axes = [Symmetry, RefY, RefZ]
      return { "Symmetry", "RefY", "RefZ" };
    case ST::SphericalSegment:  // axes = [Symmetry, RefY, RefZ]
      return { "Symmetry", "RefY", "RefZ" };
    case ST::Sphere:  // no preferred axes
      return { "(none)", "(none)", "(none)" };

    // Line & Mesh
    case ST::Line:  // direction only
      return { "Dir", "(n/a)", "(n/a)" };
    case ST::Mesh:  // as-authored mesh frame
      return { "X", "Y", "Z" };

    // Fallback
    default:
      return { "X", "Y", "Z" };
  }
}

// Printers --------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, ShapeType type)
{
  return os << shapeTypeToString(type);
}

std::ostream& operator<<(std::ostream& os, const MeshRef& mref)
{
  os << "MeshRef(uri='" << mref.uri << "')";
  return os;
}

std::ostream& operator<<(std::ostream& os, const TriangleMesh& M)
{
  os << "IndexedTriMesh(V=" << M.V.size() << ", F=" << M.F.size() << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Shape& shape)
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
