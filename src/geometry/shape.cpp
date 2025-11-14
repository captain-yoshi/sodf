#include <sodf/geometry/shape.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <iostream>

namespace sodf {
namespace geometry {

const char* dim_role_name(DimRole r)
{
  switch (r)
  {
    case DimRole::Height:
      return "height";
    case DimRole::Radius:
      return "radius";
    case DimRole::BaseRadius:
      return "base_radius";
    case DimRole::TopRadius:
      return "top_radius";
    case DimRole::SizeY:
      return "size_y";
    case DimRole::SizeZ:
      return "size_z";
    case DimRole::Base:
      return "base";
    case DimRole::Altitude:
      return "altitude";
    case DimRole::ApexOffset:
      return "apex_offset";
    case DimRole::X:
      return "x";
    case DimRole::Y:
      return "y";
    case DimRole::Z:
      return "z";
    default:
      return "unknown";
  }
}

DimRole parse_dim_role(std::string_view key)
{
  static const std::unordered_map<std::string_view, DimRole> map = { // 3D
                                                                     { "height", DimRole::Height },
                                                                     { "radius", DimRole::Radius },
                                                                     { "base_radius", DimRole::BaseRadius },
                                                                     { "top_radius", DimRole::TopRadius },
                                                                     // 2D
                                                                     { "size_y", DimRole::SizeY },
                                                                     { "size_z", DimRole::SizeZ },
                                                                     { "base", DimRole::Base },
                                                                     { "altitude", DimRole::Altitude },
                                                                     { "apex_offset", DimRole::ApexOffset },
                                                                     // Cartesian (Box only)
                                                                     { "x", DimRole::X },
                                                                     { "y", DimRole::Y },
                                                                     { "z", DimRole::Z }
  };
  if (auto it = map.find(key); it != map.end())
    return it->second;
  return DimRole::Unknown;
}

int dim_index_for(ShapeType t, DimRole r)
{
  using ST = ShapeType;
  using DR = DimRole;
  switch (t)
  {
    // Box literally stores {x, y, z}
    case ST::Box:
      switch (r)
      {
        case DR::X:
        case DR::Height:  // alias for stacking
          return 0;
        case DR::Y:
          return 1;
        case DR::Z:
          return 2;
        default:
          return -1;
      }

    // 3D semantic shapes (no generic X/Y/Z)
    case ST::TriangularPrism:  // {length_x, base_y, altitude_z, apex_offset_y}
      switch (r)
      {
        case DR::Base:
          return 0;
        case DR::Altitude:
          return 1;
        case DR::ApexOffset:
          return 2;
        case DR::Height:
          return 3;  // height along canonical X
        default:
          return -1;
      }

    case ST::Cylinder:  // {height_x, radius}
      switch (r)
      {
        case DR::Radius:
          return 0;
        case DR::Height:
          return 1;
        default:
          return -1;
      }

    case ST::Cone:  // {height_x, base_radius, top_radius}
      switch (r)
      {
        case DR::BaseRadius:
          return 0;
        case DR::TopRadius:
          return 1;
        case DR::Height:
          return 2;
        default:
          return -1;
      }

    case ST::SphericalSegment:  // {height_x, base_radius, top_radius}
      switch (r)
      {
        case DR::BaseRadius:
          return 0;
        case DR::TopRadius:
          return 1;
        case DR::Height:
          return 2;
        default:
          return -1;
      }

    // 2D shapes
    case ST::Rectangle:  // {size_y, size_z}
    case ST::Plane:      // {size_y, size_z}
      switch (r)
      {
        case DR::SizeY:
          return 0;
        case DR::SizeZ:
        case DR::Height:
          return 1;  // also “2D height”
        default:
          return -1;
      }

    case ST::Circle:  // {radius}
      switch (r)
      {
        case DR::Radius:
          return 0;
        default:
          return -1;
      }

    case ST::Triangle:  // {base_y, altitude_z, apex_offset_y}
      switch (r)
      {
        case DR::Base:
          return 0;
        case DR::ApexOffset:
          return 1;
        case DR::Altitude:
        case DR::Height:
          return 2;
        default:
          return -1;
      }

    // Others: no fixed mapping
    default:
      return -1;
  }
}

bool supports_dim_role(ShapeType t, DimRole r)
{
  return dim_index_for(t, r) >= 0;
}

double& DimAccessor::at(std::size_t i)
{
  return dims.at(i);
}
const double& DimAccessor::at(std::size_t i) const
{
  return dims.at(i);
}

bool DimAccessor::has(DimRole r) const
{
  int i = dim_index_for(type, r);
  return i >= 0 && static_cast<std::size_t>(i) < dims.size();
}
bool DimAccessor::has(std::string_view key) const
{
  DimRole r = parse_dim_role(key);
  if (r == DimRole::Unknown)
    return false;
  return has(r);
}

double* DimAccessor::try_at(DimRole r)
{
  int i = dim_index_for(type, r);
  if (i < 0 || static_cast<std::size_t>(i) >= dims.size())
    return nullptr;
  return &dims[static_cast<std::size_t>(i)];
}
const double* DimAccessor::try_at(DimRole r) const
{
  int i = dim_index_for(type, r);
  if (i < 0 || static_cast<std::size_t>(i) >= dims.size())
    return nullptr;
  return &dims[static_cast<std::size_t>(i)];
}
double* DimAccessor::try_at(std::string_view key)
{
  DimRole r = parse_dim_role(key);
  return (r == DimRole::Unknown) ? nullptr : try_at(r);
}
const double* DimAccessor::try_at(std::string_view key) const
{
  DimRole r = parse_dim_role(key);
  return (r == DimRole::Unknown) ? nullptr : try_at(r);
}

double& DimAccessor::at(DimRole r)
{
  int i = dim_index_for(type, r);
  if (i < 0)
    throw std::out_of_range(std::string("dimension role '") + dim_role_name(r) + "' not supported for shape " +
                            shapeTypeToString(type));
  if (static_cast<std::size_t>(i) >= dims.size())
    throw std::out_of_range(std::string("dimension role '") + dim_role_name(r) + "' index " + std::to_string(i) +
                            " out of bounds for shape " + shapeTypeToString(type));
  return dims[static_cast<std::size_t>(i)];
}
const double& DimAccessor::at(DimRole r) const
{
  int i = dim_index_for(type, r);
  if (i < 0)
    throw std::out_of_range(std::string("dimension role '") + dim_role_name(r) + "' not supported for shape " +
                            shapeTypeToString(type));
  if (static_cast<std::size_t>(i) >= dims.size())
    throw std::out_of_range(std::string("dimension role '") + dim_role_name(r) + "' index " + std::to_string(i) +
                            " out of bounds for shape " + shapeTypeToString(type));
  return dims[static_cast<std::size_t>(i)];
}

double& DimAccessor::at(std::string_view key)
{
  DimRole r = parse_dim_role(key);
  if (r == DimRole::Unknown)
    throw std::out_of_range(std::string("unknown dimension key '") + std::string(key) + "'");
  return at(r);
}
const double& DimAccessor::at(std::string_view key) const
{
  DimRole r = parse_dim_role(key);
  if (r == DimRole::Unknown)
    throw std::out_of_range(std::string("unknown dimension key '") + std::string(key) + "'");
  return at(r);
}

DimAccessor dim(Shape& s)
{
  return DimAccessor{ s.type, s.dimensions };
}
DimAccessor dim(const Shape& s)
{
  return DimAccessor{ s.type, const_cast<std::vector<double>&>(s.dimensions) };
}

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

      const double bw = dim(shape).at(DimRole::Base);      // along V (Base axis)
      const double bh = dim(shape).at(DimRole::Altitude);  // along U (Altitude axis)
      const double h = dim(shape).at(DimRole::Height);     // along N (Height/extrusion)
      const double u = (dimensions.size() >= 4 ? dim(shape).at(DimRole::ApexOffset) : 0.0);  // apex offset along U

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
      return (0.5 * dim(shape).at(DimRole::Height)) * N;
    }

    case ShapeType::Cone:
    {
      // dims = { base_radius, top_radius, height }
      if (dimensions.size() < 3)
        throw std::runtime_error("Cone requires dims {base_radius, top_radius, height}.");
      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);
      // Keep the prior heuristic: centroid ~ h/4 from base along axis (exact for a right cone).
      return (0.25 * dim(shape).at(DimRole::Height)) * N;
    }

    case ShapeType::SphericalSegment:
    {
      // dims = { base_radius, top_radius, height }
      if (dimensions.size() < 3)
        throw std::runtime_error("SphericalSegment requires dims {base_radius, top_radius, height}.");

      const Eigen::Vector3d& N = getShapePrimaryAxis(shape);
      const double r1 = dim(shape).at(DimRole::BaseRadius);
      const double r2 = dim(shape).at(DimRole::TopRadius);
      const double h = dim(shape).at(DimRole::Height);

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
      return dim(s).at(DimRole::SizeZ);
    case ST::Plane:
      need(2, "Plane");
      return dim(s).at(DimRole::SizeZ);
    case ST::Circle:
      need(1, "Circle");
      return 2.0 * dim(s).at(DimRole::Radius);
    case ST::Triangle:
      need(2, "Triangle");
      return dim(s).at(DimRole::Altitude);

    case ST::Polygon:  // vertices in YZ; AABB Z-extent
    {
      if (s.vertices.empty())
        return 0.0;
      double zmin = std::numeric_limits<double>::infinity();
      double zmax = -std::numeric_limits<double>::infinity();
      for (const auto& p : s.vertices)
      {
        zmin = std::min(zmin, p.z());
        zmax = std::max(zmax, p.z());
      }
      return (std::isfinite(zmin) && std::isfinite(zmax)) ? (zmax - zmin) : 0.0;
    }

    case ST::Line:
    {
      if (s.vertices.size() == 2)
        return std::abs(s.vertices[1].z() - s.vertices[0].z());
      if (!s.dimensions.empty())
        return std::abs(s.dimensions[0]);  // fallback if encoded as length only
      return 0.0;
    }

    // -------- 3D (height along canonical +X) --------
    case ST::Box:
      need(1, "Box");
      return dim(s).at(DimRole::Height);  // alias of HeightX
    case ST::TriangularPrism:
      need(1, "TriangularPrism");
      return dim(s).at(DimRole::Height);  // prism’s X extent
    case ST::Cylinder:
      need(2, "Cylinder");
      return dim(s).at(DimRole::Height);  // {height_x, radius}
    case ST::Cone:
      need(3, "Cone");
      return dim(s).at(DimRole::Height);  // {height_x, Rb, Rt}
    case ST::SphericalSegment:
      need(3, "SphericalSegment");
      return dim(s).at(DimRole::Height);  // {height_x, Rb, Rt}
    case ST::Sphere:
      need(1, "Sphere");
      return 2.0 * dim(s).at(DimRole::Radius);

    case ST::Mesh:
      throw std::runtime_error("getShapeHeight: unknown height for Mesh without inline vertices");

    case ST::None:
    default:
      throw std::runtime_error("getShapeHeight: unsupported shape type");
  }
}

double getShapeBaseRadius(const Shape& s)
{
  using ST = ShapeType;

  switch (s.type)
  {
    case ST::Cylinder:  // {height_x, radius}
    case ST::Sphere:    // {radius}
    case ST::Circle:    // {radius}
      return dim(s).at(DimRole::Radius);

    case ST::Cone:              // {height_x, base_radius, top_radius}
    case ST::SphericalSegment:  // {height_x, base_radius, top_radius}
      return dim(s).at(DimRole::BaseRadius);

    case ST::Box:  // {x, y, z} → cross-section is Y/Z
    {
      const double wy = dim(s).at(DimRole::Y);
      const double wz = dim(s).at(DimRole::Z);
      return 0.5 * std::max(wy, wz);
    }

    default:
      return 0.0;
  }
}

double getShapeTopRadius(const Shape& s)
{
  using ST = ShapeType;

  switch (s.type)
  {
    case ST::Cylinder:
    case ST::Sphere:
    case ST::Circle:
      return dim(s).at(DimRole::Radius);

    case ST::Cone:
    case ST::SphericalSegment:
      return dim(s).at(DimRole::TopRadius);

    case ST::Box:
    {
      const double wy = dim(s).at(DimRole::Y);
      const double wz = dim(s).at(DimRole::Z);
      return 0.5 * std::max(wy, wz);
    }

    default:
      return 0.0;
  }
}

double getShapeMaxRadius(const Shape& s)
{
  using ST = ShapeType;

  switch (s.type)
  {
    case ST::Cylinder:
    case ST::Sphere:
    case ST::Circle:
      return dim(s).at(DimRole::Radius);

    case ST::Cone:
    case ST::SphericalSegment:
    {
      const double rb = dim(s).at(DimRole::BaseRadius);
      const double rt = dim(s).at(DimRole::TopRadius);
      return std::max(rb, rt);
    }

    case ST::Box:
    {
      const double wy = dim(s).at(DimRole::Y);
      const double wz = dim(s).at(DimRole::Z);
      return 0.5 * std::max(wy, wz);
    }

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

Shape truncateShapeToHeight(const Shape& shape, double new_height)
{
  Shape result = shape;

  switch (shape.type)
  {
    // Prism’s “height” is LengthX (canonical +X)
    case ShapeType::TriangularPrism:
    {
      if (!dim(result).has(DimRole::Height))
        throw std::runtime_error("TriangularPrism expects height");
      if (new_height <= 0.0)
        throw std::runtime_error("TriangularPrism truncation: new height must be > 0");

      dim(result).at(DimRole::Height) = new_height;
      break;
    }

    case ShapeType::Cylinder:  // {height_x, radius}
    {
      if (!dim(result).has(DimRole::Height))
        throw std::runtime_error("Cylinder expects height_x");
      dim(result).at(DimRole::Height) = new_height;
      break;
    }

    case ShapeType::Cone:  // {height_x, base_radius, top_radius}
    {
      const double h = dim(shape).at(DimRole::Height);
      const double rb = dim(shape).at(DimRole::BaseRadius);
      const double rt = dim(shape).at(DimRole::TopRadius);

      if (h <= 0.0)
        throw std::runtime_error("Cone truncation: invalid original height");

      const double t = std::clamp(new_height / h, 0.0, 1.0);
      const double new_r = rb + (rt - rb) * t;  // linear along height

      dim(result).at(DimRole::TopRadius) = new_r;
      dim(result).at(DimRole::Height) = new_height;
      break;
    }

    case ShapeType::SphericalSegment:  // {height_x, base_radius, top_radius}
    {
      const double h = dim(shape).at(DimRole::Height);
      const double rb = dim(shape).at(DimRole::BaseRadius);
      const double rt = dim(shape).at(DimRole::TopRadius);

      const double clipped_r = getTopRadiusAtHeight(rb, rt, h, new_height);

      dim(result).at(DimRole::BaseRadius) = rb;        // unchanged
      dim(result).at(DimRole::TopRadius) = clipped_r;  // new top
      dim(result).at(DimRole::Height) = new_height;    // new height
    }

    default:
      // Not supported → mark invalid
      result.dimensions.clear();
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
