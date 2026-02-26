#include <sodf/geometry/frame.h>
#include "sodf/xml/element_parser.h"
#include <sodf/xml/expression_parser.h>
#include <sodf/xml/utils.h>

namespace sodf {
namespace xml {

struct AxesTagNames
{
  const char* X;  // canonical X tag (e.g., "AxisHeight")
  const char* Y;  // canonical Y tag (e.g., "AxisWidth")
  const char* Z;  // canonical Z tag (e.g., "AxisNormal")
};

inline std::array<Eigen::Vector3d, 3> parseAxesCanonicalXYZ(const tinyxml2::XMLElement* elem,
                                                            const XMLParseContext& ctx, const AxesTagNames& tags,
                                                            double tol = 1e-9)
{
  using Eigen::Vector3d;

  auto get = [&](const char* tag) -> const tinyxml2::XMLElement* {
    return elem ? elem->FirstChildElement(tag) : nullptr;
  };

  const tinyxml2::XMLElement *ex = get(tags.X), *ey = get(tags.Y), *ez = get(tags.Z);
  const bool hasX = (ex != nullptr), hasY = (ey != nullptr), hasZ = (ez != nullptr);
  const int ln = elem ? elem->GetLineNum() : -1;
  const int provided = int(hasX) + int(hasY) + int(hasZ);

  // default canonical basis
  if (provided == 0)
  {
    return { Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ() };
  }

  if (provided < 2)
  {
    throw_xml_error(elem, ctx,
                    std::string("Provide at least two of <") + tags.X + ">, <" + tags.Y + ">, <" + tags.Z + ">");
  }

  auto nearly_zero = [&](const Vector3d& v) { return v.squaredNorm() < tol * tol; };
  auto is_unit = [&](const Vector3d& v) { return std::abs(v.norm() - 1.0) <= 1e2 * tol; };

  Vector3d X = hasX ? parseUnitVector(ex, ctx) : Vector3d::Zero();
  Vector3d Y = hasY ? parseUnitVector(ey, ctx) : Vector3d::Zero();
  Vector3d Z = hasZ ? parseUnitVector(ez, ctx) : Vector3d::Zero();

  if (provided == 3)
  {
    // Do NOT change user-provided axes. Only validate.
    if (!is_unit(X) || !is_unit(Y) || !is_unit(Z))
    {
      throw_xml_error(elem, ctx, "Axes must be unit-length");
    }

    if (std::abs(X.dot(Y)) > tol || std::abs(X.dot(Z)) > tol || std::abs(Y.dot(Z)) > tol)
    {
      throw_xml_error(elem, ctx, "Provided axes must be orthogonal");
    }

    const double det = X.dot(Y.cross(Z));
    if (det < 0.0 - 1e2 * tol)
    {
      throw_xml_error(elem, ctx,
                      "Provided axes are left-handed; "
                      "expected right-handed");
    }

    if (nearly_zero(Y.cross(Z)) || nearly_zero(Z.cross(X)) || nearly_zero(X.cross(Y)))
    {
      throw_xml_error(elem, ctx, "Degenerate axes");
    }

    return { X, Y, Z };
  }

  // Exactly two provided: derive the missing to enforce right-handed XYZ
  if (!hasZ)
  {
    Z = X.cross(Y);
    if (nearly_zero(Z))
    {
      Vector3d Yseed = geometry::computeOrthogonalAxis(X);
      Z = X.cross(Yseed);
    }
  }
  else if (!hasY)
  {
    Y = Z.cross(X);  // ensures X × Y = Z
    if (nearly_zero(Y))
    {
      Y = geometry::computeOrthogonalAxis(X);
    }
  }
  else  // !hasX
  {
    X = Y.cross(Z);  // ensures X × Y = Z
    if (nearly_zero(X))
    {
      X = geometry::computeOrthogonalAxis(Z);
    }
  }

  // Orthonormalize the *constructed* frame to kill roundoff; preserves handedness.
  Eigen::Matrix3d R = geometry::makeOrthonormalRightHanded(X, Y, Z);
  return { R.col(0), R.col(1), R.col(2) };
}

void parsePosition(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, Eigen::Vector3d& pos)
{
  pos.x() = evalNumberAttributeRequired(elem, "x", ctx);
  pos.y() = evalNumberAttributeRequired(elem, "y", ctx);
  pos.z() = evalNumberAttributeRequired(elem, "z", ctx);
}

void parseOrientationRPY(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, Eigen::Quaterniond& q)
{
  double roll = evalNumberAttributeRequired(elem, "roll", ctx);
  double pitch = evalNumberAttributeRequired(elem, "pitch", ctx);
  double yaw = evalNumberAttributeRequired(elem, "yaw", ctx);
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, Eigen::Quaterniond& q)
{
  q.x() = evalNumberAttributeRequired(elem, "x", ctx);
  q.y() = evalNumberAttributeRequired(elem, "y", ctx);
  q.z() = evalNumberAttributeRequired(elem, "z", ctx);
  q.w() = evalNumberAttributeRequired(elem, "w", ctx);
}

Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  Eigen::Vector3d pos{ 0, 0, 0 };
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  if (const auto* pos_elem = elem->FirstChildElement("Position"))
    parsePosition(pos_elem, ctx, pos);

  if (const auto* ori_elem = elem->FirstChildElement("Orientation"))
    parseOrientationRPY(ori_elem, ctx, q);
  else if (const auto* quat_elem = elem->FirstChildElement("Quaternion"))
  {
    parseQuaternion(quat_elem, ctx, q);
  }

  return Eigen::Translation3d(pos) * q;
}

geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  geometry::TransformNode frame;

  (void)tryEvalTextAttribute(elem, "parent", &frame.parent, ctx);

  frame.local = parseIsometry3D(elem, ctx);
  frame.rest_local = frame.local;
  frame.global = Eigen::Isometry3d::Identity();
  frame.dirty = true;

  return frame;
}

Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, double epsilon)
{
  Eigen::Vector3d vec;

  vec.x() = evalNumberAttributeRequired(elem, "x", ctx);
  vec.y() = evalNumberAttributeRequired(elem, "y", ctx);
  vec.z() = evalNumberAttributeRequired(elem, "z", ctx);

  double norm = vec.norm();
  if (std::abs(norm - 1.0) > epsilon)
  {
    {
      throw_xml_error(elem, ctx, "Expected unit vector but got norm=" + std::to_string(norm));
    }
  }
  return vec;
}

// (Note: fixed the spelling to "RightHand")
void validateAxesRightHandOrthonormal(const geometry::Shape& shape, const tinyxml2::XMLElement* elem,
                                      const XMLParseContext& ctx, double eps = 1e-9)
{
  const int ln = elem ? elem->GetLineNum() : -1;
  const std::size_t n = shape.axes.size();

  if (n != 3)
  {
    throw_xml_error(elem, ctx, "Shape must define exactly 3 axes (X, Y, Z); got " + std::to_string(n));
  }

  const Eigen::Vector3d& X = shape.axes[0];
  const Eigen::Vector3d& Y = shape.axes[1];
  const Eigen::Vector3d& Z = shape.axes[2];

  if (!geometry::areVectorsOrthonormal(X, Y, Z, eps))
  {
    throw_xml_error(elem, ctx, "Shape axes are not orthonormal");
  }

  if (!geometry::isRightHanded(X, Y, Z, eps))
  {
    throw_xml_error(elem, ctx, "Shape axes are not right-handed");
  }
}

const tinyxml2::XMLElement* reqChild(const tinyxml2::XMLElement* parent, const XMLParseContext& ctx, const char* tag,
                                     const char* context)
{
  if (!parent)
  {
    throw_xml_error(nullptr, ctx, std::string(context) + ": null parent element while requiring <" + tag + ">");
  }

  if (const auto* e = parent->FirstChildElement(tag))
    return e;

  throw_xml_error(parent, ctx, std::string(context) + " missing <" + tag + ">");
}

// ============================== 2D SHAPES ==============================

geometry::Shape parseRectangleShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;

  geometry::Shape s;
  s.type = geometry::ShapeType::Rectangle;

  // Axes (canonical): X = normal, Y = size_y, Z = size_z
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=normal, Y=size_y, Z=size_z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if ((dims != nullptr) == (verts != nullptr))
  {
    throw_xml_error(elem, ctx,
                    "Rectangle must specify exactly one of "
                    "<Dimensions> or <Vertices>");
  }

  if (dims)
  {
    const double sy = evalNumberAttributeRequired(dims, "size_y", ctx);
    const double sz = evalNumberAttributeRequired(dims, "size_z", ctx);
    s.dimensions = { sy, sz };
  }
  else
  {
    // Vertices expressed in (Y,Z) plane (X=0)
    std::size_t count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y", ctx);
      const double z = evalNumberAttributeRequired(v, "z", ctx);
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count != 4)
    {
      throw_xml_error(verts, ctx,
                      "Rectangle with <Vertices> "
                      "must contain exactly 4 <Vertex> entries");
    }

    s.dimensions.clear();
  }

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseCircleShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Circle;

  // Axes (canonical): X = normal, (Y,Z) = reference in-plane axes
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=normal, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = reqChild(elem, ctx, "Dimensions", "Circle");
  s.dimensions = { evalNumberAttributeRequired(dims, "radius", ctx) };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseTriangleShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Triangle;

  // Axes (canonical): X = normal, Y = base, Z = altitude
  const AxesTagNames tags{ "AxisNormal", "AxisBase", "AxisAltitude" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=normal, Y=base, Z=altitude]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if ((dims != nullptr) == (verts != nullptr))
  {
    throw_xml_error(elem, ctx,
                    "Triangle must specify exactly one of "
                    "<Dimensions> or <Vertices>");
  }

  if (dims)
  {
    const double base_y = evalNumberAttributeRequired(dims, "base_y", ctx);
    const double altitude_z = evalNumberAttributeRequired(dims, "altitude_z", ctx);
    const double apex_off_y = evalNumberAttributeRequired(dims, "apex_offset_y", ctx);
    s.dimensions = { base_y, apex_off_y, altitude_z };
  }
  else
  {
    // Vertices in (Y,Z) plane (X=0)
    int count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y", ctx);
      const double z = evalNumberAttributeRequired(v, "z", ctx);
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count != 3)
    {
      throw_xml_error(verts, ctx,
                      "Triangle with <Vertices> must contain "
                      "exactly 3 <Vertex> entries");
    }

    s.dimensions.clear();
  }

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parsePolygonShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Polygon;

  // Axes (canonical): X = normal, Y = in-plane y, Z = in-plane z
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=normal, Y, Z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* dims = elem->FirstChildElement("Dimensions"))
  {
    throw_xml_error(dims, ctx,
                    "Polygon must not define <Dimensions> "
                    "(dimensionless shape)");
  }

  const auto* verts = reqChild(elem, ctx, "Vertices", "Polygon");
  std::size_t count = 0;
  for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
  {
    const double y = evalNumberAttributeRequired(v, "y", ctx);
    const double z = evalNumberAttributeRequired(v, "z", ctx);
    s.vertices.emplace_back(0.0, y, z);  // X=0 (normal axis)
    ++count;
  }
  if (count < 3)
  {
    throw_xml_error(verts, ctx,
                    "Polygon requires at least 3 "
                    "<Vertex> entries");
  }

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parsePlaneShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Plane;

  // Axes (canonical): X = normal, Y = refY, Z = refZ
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=normal, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if (dims && verts)
  {
    throw_xml_error(elem, ctx,
                    "Plane must specify at most one of "
                    "<Dimensions> or <Vertices>");
  }

  if (dims)
  {
    const double sy = evalNumberAttributeRequired(dims, "size_y", ctx);
    const double sz = evalNumberAttributeRequired(dims, "size_z", ctx);
    if (!(sy > 0.0 && sz > 0.0))
    {
      throw_xml_error(dims, ctx,
                      "Plane <Dimensions> require positive "
                      "size_y and size_z");
    }
    s.dimensions = { sy, sz };
  }
  else if (verts)
  {
    std::size_t count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y", ctx);
      const double z = evalNumberAttributeRequired(v, "z", ctx);
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count < 3)
    {
      throw_xml_error(verts, ctx,
                      "Plane with <Vertices> requires "
                      "at least 3 <Vertex> entries");
    }

    s.dimensions.clear();
  }
  // else: infinite plane

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

// ============================== 3D SHAPES ==============================

geometry::Shape parseBoxShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Box;

  // Axes (canonical): X, Y, Z
  const AxesTagNames tags{ "AxisX", "AxisY", "AxisZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X, Y, Z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
  {
    throw_xml_error(verts, ctx, "Box must not define <Vertices>");
  }

  const auto* dims = reqChild(elem, ctx, "Dimensions", "Box");
  const double x = evalNumberAttributeRequired(dims, "x", ctx);
  const double y = evalNumberAttributeRequired(dims, "y", ctx);
  const double z = evalNumberAttributeRequired(dims, "z", ctx);
  s.dimensions = { x, y, z };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseTriangularPrismShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::TriangularPrism;

  // Axes (canonical): X = extrusion/length_x, Y = base, Z = altitude
  const AxesTagNames tags{ "AxisExtrusion", "AxisBase", "AxisAltitude" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=extrusion, Y=base, Z=altitude]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("TriangularPrism must not define <Vertices> at line " +
                             std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, ctx, "Dimensions", "TriangularPrism");
  const double length_x = evalNumberAttributeRequired(dims, "length_x", ctx);      // along X
  const double base_y = evalNumberAttributeRequired(dims, "base_y", ctx);          // along Y
  const double altitude_z = evalNumberAttributeRequired(dims, "altitude_z", ctx);  // along Z
  double apex_offset_y = 0.0;
  (void)tryEvalNumberAttribute(dims, "apex_offset_y", &apex_offset_y, ctx);

  if (!(length_x > 0.0 && base_y > 0.0 && altitude_z > 0.0))
  {
    throw_xml_error(dims, ctx,
                    "TriangularPrism requires positive "
                    "length_x, base_y, and altitude_z");
  }

  s.dimensions = { base_y, apex_offset_y, altitude_z, length_x };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseCylinderShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Cylinder;

  // Axes (canonical): X = symmetry/height axis, (Y,Z) = reference in-plane
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
  {
    throw_xml_error(verts, ctx, "Cylinder must not define <Vertices>");
  }

  const auto* dims = reqChild(elem, ctx, "Dimensions", "Cylinder");
  const double radius = evalNumberAttributeRequired(dims, "radius", ctx);
  const double height_x = evalNumberAttributeRequired(dims, "height_x", ctx);
  if (!(radius > 0.0 && height_x > 0.0))
  {
    throw_xml_error(dims, ctx,
                    "Cylinder requires positive "
                    "radius and height_x");
  }

  s.dimensions = { radius, height_x };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseSphereShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Sphere;

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("Sphere must not define <Vertices> at line " + std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, ctx, "Dimensions", "Sphere");
  const double radius = evalNumberAttributeRequired(dims, "radius", ctx);
  if (!(radius > 0.0))
    throw std::runtime_error("Sphere requires positive radius at line " + std::to_string(dims->GetLineNum()));

  s.dimensions = { radius };
  // No axes to validate (isotropic)
  return s;
}

geometry::Shape parseConeShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Cone;

  // Axes (canonical): X = symmetry/height, (Y,Z) = reference
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
  {
    throw_xml_error(verts, ctx, "Cone must not define <Vertices>");
  }

  const auto* dims = reqChild(elem, ctx, "Dimensions", "Cone");
  const double base_r = evalNumberAttributeRequired(dims, "base_radius", ctx);
  const double top_r = evalNumberAttributeRequired(dims, "top_radius", ctx);
  const double height_x = evalNumberAttributeRequired(dims, "height_x", ctx);

  if (!(height_x > 0.0) || base_r < 0.0 || top_r < 0.0)
  {
    throw_xml_error(dims, ctx,
                    "Cone requires base_radius ≥ 0, "
                    "top_radius ≥ 0, and height_x > 0");
  }
  if (base_r == 0.0 && top_r == 0.0)
  {
    throw_xml_error(dims, ctx,
                    "Cone invalid: base_radius and "
                    "top_radius cannot both be zero");
  }

  s.dimensions = { base_r, top_r, height_x };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseSphericalSegmentShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::SphericalSegment;

  // Axes (canonical): X = symmetry/height, (Y,Z) = reference
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
  {
    throw_xml_error(verts, ctx, "SphericalSegment must not define <Vertices>");
  }

  const auto* dims = reqChild(elem, ctx, "Dimensions", "SphericalSegment");

  const bool has_r1 = dims->Attribute("base_radius");
  const bool has_r2 = dims->Attribute("top_radius");
  const bool has_hx = dims->Attribute("height_x");

  int have = (has_r1 ? 1 : 0) + (has_r2 ? 1 : 0) + (has_hx ? 1 : 0);
  if (have < 2)
  {
    throw_xml_error(dims, ctx,
                    "SphericalSegment must provide at least two of "
                    "base_radius, top_radius, height_x");
  }

  double r1 = has_r1 ? evalNumberAttributeRequired(dims, "base_radius", ctx) : 0.0;
  double r2 = has_r2 ? evalNumberAttributeRequired(dims, "top_radius", ctx) : 0.0;
  double hx = has_hx ? evalNumberAttributeRequired(dims, "height_x", ctx) : 0.0;

  // If you have inference helpers, call them here (renamed for *_height_x if needed).
  if (have == 2)
  {
    if (!has_hx)
      hx = geometry::inferSegmentHeightFromRadii(r1, r2);
    else if (!has_r2)
      r2 = geometry::inferTopRadiusFromHeight(r1, hx);
    else /* !has_r1*/
      r1 = geometry::inferBaseRadiusFromHeight(r2, hx);
  }
  else
  {
    if (!geometry::isValidSegment(r1, r2, hx))
    {
      throw_xml_error(dims, ctx,
                      "SphericalSegment <Dimensions> are incompatible: "
                      "no valid sphere exists");
    }
  }

  s.dimensions = { r1, r2, hx };

  validateAxesRightHandOrthonormal(s, elem, ctx);
  return s;
}

geometry::Shape parseMeshShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::geometry;

  Shape s;
  s.type = ShapeType::Mesh;

  // ---- Axes (canonical: X=AxisX, Y=AxisY, Z=AxisZ)
  // Policy:
  //   - If 0 axes provided  → default to identity (I3).
  //   - If 1 axis provided  → error (must provide at least two to disambiguate).
  //   - If 2 or 3 provided  → derive the missing one right-handed (X×Y=Z), orthonormalize, and validate.
  const bool hasX = elem->FirstChildElement("AxisX") != nullptr;
  const bool hasY = elem->FirstChildElement("AxisY") != nullptr;
  const bool hasZ = elem->FirstChildElement("AxisZ") != nullptr;
  const int nAxesProvided = int(hasX) + int(hasY) + int(hasZ);

  if (nAxesProvided == 0)
  {
    // Identity frame
    s.axes = { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ() };
  }
  else if (nAxesProvided == 1)
  {
    throw_xml_error(elem, ctx,
                    "Mesh must provide either 0 axes "
                    "(defaults to identity) or at least 2 axes");
  }
  else
  {
    // 2 or 3 provided → parse/complete/validate
    const AxesTagNames tags{ "AxisX", "AxisY", "AxisZ" };
    auto axesXYZ = parseAxesCanonicalXYZ(elem, ctx, tags);  // [X, Y, Z], right-handed ONB
    s.axes.assign(axesXYZ.begin(), axesXYZ.end());
    validateAxesRightHandOrthonormal(s, elem, ctx);
  }

  // --- Scale (optional, defaults to 1,1,1) ---
  if (const auto* scale = elem->FirstChildElement("Scale"))
  {
    s.scale.x() = evalNumberAttribute(scale, "x", 1.0, ctx);
    s.scale.y() = evalNumberAttribute(scale, "y", 1.0, ctx);
    s.scale.z() = evalNumberAttribute(scale, "z", 1.0, ctx);
  }

  // --- Exactly one of <External> or <Inline> ---
  const auto* external = elem->FirstChildElement("External");
  const auto* internal = elem->FirstChildElement("Inline");
  const bool hasExternal = (external != nullptr);
  const bool hasInline = (internal != nullptr);
  if (hasExternal == hasInline)
  {
    throw_xml_error(elem, ctx,
                    "<Mesh> must contain exactly one of "
                    "<External> or <Inline>");
  }

  if (hasExternal)
  {
    // External URI form
    std::string uri = evalTextAttributeRequired(external, "uri", ctx);
    uri = resolve_realtive_uri(external, ctx.doc, uri);  // (uses your resolver; keep spelling as in your codebase)
    s.mesh = MeshRef{ std::move(uri) };
  }
  else
  {
    // Inline (TriangleMesh) form
    const auto* vs = internal->FirstChildElement("Vertices");
    const auto* fs = internal->FirstChildElement("Faces");
    if (!vs || !fs)
    {
      throw_xml_error(internal, ctx,
                      "<Inline> must contain <Vertices> "
                      "and <Faces>");
    }

    auto M = std::make_shared<TriangleMesh>();
    M->V.reserve(16);
    M->F.reserve(32);

    // <Vertices>
    for (const tinyxml2::XMLElement* v = vs->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double x = evalNumberAttributeRequired(v, "x", ctx);
      const double y = evalNumberAttributeRequired(v, "y", ctx);
      const double z = evalNumberAttributeRequired(v, "z", ctx);
      M->V.emplace_back(x, y, z);
    }
    if (M->V.empty())
    {
      throw_xml_error(vs, ctx, "<Vertices> is empty");
    }

    // <Faces> (0-based indices)
    for (const tinyxml2::XMLElement* f = fs->FirstChildElement("Face"); f; f = f->NextSiblingElement("Face"))
    {
      const uint32_t a = evalUIntAttributeRequired(f, "a", ctx);
      const uint32_t b = evalUIntAttributeRequired(f, "b", ctx);
      const uint32_t c = evalUIntAttributeRequired(f, "c", ctx);
      M->F.push_back({ a, b, c });
    }
    if (M->F.empty())
    {
      throw_xml_error(fs, ctx, "<Faces> is empty");
    }

    // Validate indices
    const uint32_t nV = static_cast<uint32_t>(M->V.size());
    for (const auto& tri : M->F)
    {
      if (tri[0] >= nV || tri[1] >= nV || tri[2] >= nV)
      {
        throw_xml_error(fs, ctx,
                        "Face index out of range "
                        "in <Inline> mesh");
      }
    }

    s.mesh = std::move(M);
  }

  return s;
}

geometry::Shape parseLineShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace tinyxml2;
  using Eigen::Vector3d;

  geometry::Shape s;
  s.type = geometry::ShapeType::Line;

  const XMLElement* axis_elem = elem->FirstChildElement("AxisDirection");
  const XMLElement* dims_elem = elem->FirstChildElement("Dimensions");
  const XMLElement* vtx_elem = elem->FirstChildElement("Vertex");

  const bool has_axis = (axis_elem != nullptr);
  const bool has_dims = (dims_elem != nullptr);
  const bool has_verts = (vtx_elem != nullptr);

  // --- Mode selection & exclusivity rules -----------------------------------
  // Valid options:
  //  A) AxisDirection + Dimensions(length)     (bounded segment)
  //  B) Exactly two <Vertex> elements          (2D or 3D segment)
  //
  // Invalid:
  //  - AxisDirection without Dimensions, or vice versa
  //  - Any Axis/Dimensions present together with Vertices
  if (has_verts)
  {
    if (has_axis || has_dims)
    {
      throw_xml_error(elem, ctx,
                      "Line: Vertices mode forbids "
                      "<AxisDirection> and <Dimensions>");
    }
  }
  else
  {
    // no vertices → must have BOTH axis and dims
    if (has_axis != has_dims)
    {
      throw_xml_error(elem, ctx,
                      "Line: Provide EITHER two <Vertex> tags "
                      "OR BOTH <AxisDirection> and "
                      "<Dimensions length=\"...\">");
    }
    if (!has_axis && !has_dims)
    {
      throw_xml_error(elem, ctx,
                      "Line: Missing specification. "
                      "Provide either vertices or "
                      "(AxisDirection + Dimensions)");
    }
  }

  // --- Mode A: Axis + Dimensions --------------------------------------------
  if (!has_verts)
  {
    // Parse axis (single unit vector only; no 3-axis frame)
    const Vector3d axis = parseUnitVector(axis_elem, ctx);
    s.axes.clear();
    s.axes.push_back(axis);

    // Require <Dimensions length="..."> and length > 0
    if (!dims_elem->FindAttribute("length"))
    {
      throw_xml_error(dims_elem, ctx,
                      "Line: <Dimensions> must include "
                      "a 'length' attribute");
    }
    const double length = evalNumberAttributeRequired(dims_elem, "length", ctx);
    if (!(length > 0.0))
    {
      throw_xml_error(dims_elem, ctx, "Line: length must be positive");
    }

    // Store length. Do NOT synthesize vertices.
    s.dimensions.clear();
    s.dimensions.push_back(length);

    // Optional sanity: ensure the single axis is unit (parseUnitVector already enforces).
    return s;
  }

  // --- Mode B: Vertices (exactly 2) -----------------------------------------
  // Read exactly two <Vertex> with (x,y[,z]); 2D if z omitted on both.
  std::array<const XMLElement*, 2> Vxml{};
  int count = 0;
  for (const XMLElement* v = vtx_elem; v && count < 2; v = v->NextSiblingElement("Vertex"))
  {
    Vxml[count++] = v;
  }
  if (count != 2 || Vxml[1] == nullptr)
  {
    throw_xml_error(elem, ctx, "Line: Must have exactly two <Vertex> tags");
  }

  s.vertices.reserve(2);
  for (int i = 0; i < 2; ++i)
  {
    const XMLElement* v = Vxml[i];
    const double x = evalNumberAttributeRequired(v, "x", ctx);
    const double y = evalNumberAttributeRequired(v, "y", ctx);
    double z = 0.0;
    if (v->FindAttribute("z"))
      tryEvalNumberAttribute(v, "z", &z, ctx);
    s.vertices.emplace_back(x, y, z);
  }

  // Either both specify z or neither (enforce 2D vs 3D consistently)
  const bool z0 = (Vxml[0]->FindAttribute("z") != nullptr);
  const bool z1 = (Vxml[1]->FindAttribute("z") != nullptr);
  if (z0 != z1)
  {
    throw_xml_error(elem, ctx,
                    "Line: Either both vertices must "
                    "specify 'z' or neither does");
  }

  // In vertices mode, axes and dimensions are irrelevant.
  s.axes.clear();
  s.dimensions.clear();

  return s;
}

void validateOriginAuthoring(const geometry::Shape& s, bool origin_present, const tinyxml2::XMLElement* elem,
                             const XMLParseContext& ctx)
{
  using geometry::OriginPolicy;
  using geometry::ShapeType;

  const std::string line = std::to_string(elem->GetLineNum());

  // (1) Mesh: origin must be unset or Native
  if (s.type == ShapeType::Mesh)
  {
    if (!origin_present)
      return;  // OK
    if (s.origin == OriginPolicy::Native)
      return;  // OK

    throw_xml_error(elem, ctx, "Mesh 'origin' must be 'Native' or omitted");
  }

  // Non-mesh primitives only below
  if (!isPrimitive(s.type))
    return;  // ShapeType::None etc.

  const bool has_dims = primitiveHasDimensions(s);
  const bool has_verts = primitiveHasVertices(s);

  // (3) 3D primitives cannot have vertices -> must use Mesh
  if (s.type != ShapeType::Line && isPrimitive3D(s.type) && has_verts)
  {
    throw_xml_error(elem, ctx,
                    "3D primitives cannot use <Vertices>. "
                    "Use type='Mesh' instead");
  }

  // (2) Primitives with dimensions: origin required and must be != Native
  if (has_dims)
  {
    if (!origin_present)
    {
      throw_xml_error(elem, ctx,
                      "Primitive with <Dimensions> requires "
                      "explicit 'origin' attribute");
    }
    if (s.origin == OriginPolicy::Native)
    {
      throw_xml_error(elem, ctx,
                      "Primitive with <Dimensions> must have "
                      "origin != 'Native'");
    }
    return;  // OK
  }

  // 2D vertex-authored primitives: only Native or omitted
  if (has_verts)
  {
    if (!origin_present)
      return;  // OK
    if (s.origin == OriginPolicy::Native)
      return;  // OK
    throw_xml_error(elem, ctx,
                    "Vertex-authored primitive allows only "
                    "origin='Native' or no attribute");
  }

  // If neither dims nor vertices are provided for a primitive, it's ill-formed
  throw_xml_error(elem, ctx,
                  "Primitive is missing authoring data "
                  "(<Dimensions> or <Vertices>)");
}

std::optional<geometry::OriginPolicy> parseOriginPolicyAttr(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  const char* raw = elem->Attribute("origin");
  if (!raw)
    return std::nullopt;

  using geometry::OriginPolicy;
  std::string_view v(raw);

  if (v == "Native")
    return OriginPolicy::Native;
  if (v == "BaseCenter")
    return OriginPolicy::BaseCenter;
  if (v == "AABBCenter")
    return OriginPolicy::AABBCenter;
  if (v == "VolumeCentroid")
    return OriginPolicy::VolumeCentroid;

  throw_xml_error(elem, ctx,
                  std::string("Unknown origin policy '") + std::string(v) +
                      "' (allowed: Native, BaseCenter, "
                      "AABBCenter, VolumeCentroid)");
}

geometry::Shape parseShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  const std::string type_str = evalTextAttributeRequired(elem, "type", ctx);
  geometry::ShapeType type = geometry::shapeTypeFromString(type_str.c_str());

  geometry::Shape s;
  switch (type)
  {
    case geometry::ShapeType::Rectangle:
      s = parseRectangleShape(elem, ctx);
      break;
    case geometry::ShapeType::Circle:
      s = parseCircleShape(elem, ctx);
      break;
    case geometry::ShapeType::Triangle:
      s = parseTriangleShape(elem, ctx);
      break;
    case geometry::ShapeType::Polygon:
      s = parsePolygonShape(elem, ctx);
      break;
    case geometry::ShapeType::Box:
      s = parseBoxShape(elem, ctx);
      break;
    case geometry::ShapeType::TriangularPrism:
      s = parseTriangularPrismShape(elem, ctx);
      break;
    case geometry::ShapeType::Cylinder:
      s = parseCylinderShape(elem, ctx);
      break;
    case geometry::ShapeType::Sphere:
      s = parseSphereShape(elem, ctx);
      break;
    case geometry::ShapeType::Cone:
      s = parseConeShape(elem, ctx);
      break;
    case geometry::ShapeType::SphericalSegment:
      s = parseSphericalSegmentShape(elem, ctx);
      break;
    case geometry::ShapeType::Plane:
      s = parsePlaneShape(elem, ctx);
      break;
    case geometry::ShapeType::Mesh:
      s = parseMeshShape(elem, ctx);
      break;
    case geometry::ShapeType::Line:
      s = parseLineShape(elem, ctx);
      break;
    default:
      throw_xml_error(elem, ctx, std::string("Unknown shape type '") + type_str + "'");
  }

  // Parse optional origin attribute
  const auto origin_attr = parseOriginPolicyAttr(elem, ctx);
  const bool origin_present = origin_attr.has_value();
  if (origin_present)
  {
    s.origin = *origin_attr;  // default was Native; overwrite if present
  }

  // Enforce your rules
  validateOriginAuthoring(s, origin_present, elem, ctx);

  return s;
}

geometry::StackedShape parseStackedShape(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  geometry::StackedShape stack;

  // const tinyxml2::XMLElement* dir_elem = stacked_elem->FirstChildElement("AxisStackDirection");
  // if (!dir_elem)
  //   throw std::runtime_error("<StackedShape> missing <AxisStackDirection> element at line " +
  //                            std::to_string(stacked_elem->GetLineNum()));
  // stack.axis_stack_direction = parseUnitVector(dir_elem).normalized();

  // const tinyxml2::XMLElement* ref_elem = stacked_elem->FirstChildElement("AxisStackReference");
  // if (!ref_elem)
  //   throw std::runtime_error("<StackedShape> missing <AxisStackReference> element at line " +
  //                            std::to_string(stacked_elem->GetLineNum()));
  // stack.axis_stack_reference = parseUnitVector(ref_elem).normalized();

  // if (!geometry::areVectorsOrthonormal(stack.axis_stack_direction, stack.axis_stack_reference))
  //   throw std::runtime_error(std::string("Axes for StackedShape must be mutually orthonormal at line ") +
  //                            std::to_string(stacked_elem->GetLineNum()));

  // --- shapes: optional <Transform>; defaults to Identity (absolute in stack frame) ---
  for (const tinyxml2::XMLElement* shape_elem = elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    geometry::Shape shape = parseShape(shape_elem, ctx);

    const tinyxml2::XMLElement* trans_elem = shape_elem->FirstChildElement("Transform");
    Eigen::Isometry3d T_abs = Eigen::Isometry3d::Identity();
    if (trans_elem)
      T_abs = parseIsometry3D(trans_elem, ctx);  // absolute w.r.t. stack/base frame

    // Store absolute transform as-is (field name may still be 'relative_transform')
    stack.shapes.push_back({ shape, T_abs });
  }

  return stack;
}

components::Button parseButton(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)

{
  using namespace sodf::components;

  Button btn;
  btn.type = buttonTypeFromString(evalTextAttributeRequired(elem, "type", ctx).c_str());

  const tinyxml2::XMLElement* link_elem = elem->FirstChildElement("Link");
  if (!link_elem)
  {
    throw_xml_error(elem, ctx, "Button missing <Link>");
  }

  btn.link_id = evalTextAttributeRequired(link_elem, "ref", ctx);

  const tinyxml2::XMLElement* joint_elem = elem->FirstChildElement("Joint");
  if (!joint_elem)
  {
    throw_xml_error(elem, ctx, "Button missing <Joint>");
  }

  btn.joint_id = evalTextAttributeRequired(joint_elem, "ref", ctx);

  if (const auto* detent_elem = elem->FirstChildElement("DetentSpacing"))
    tryEvalNumberAttribute(detent_elem, "value", &btn.detent_spacing, ctx);

  if (const auto* rest_elem = elem->FirstChildElement("RestPositions"))
    for (const auto* pos_elem = rest_elem->FirstChildElement("Position"); pos_elem;
         pos_elem = pos_elem->NextSiblingElement("Position"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(pos_elem, "value", &v, ctx))
        btn.rest_positions.push_back(v);
    }

  if (const auto* stiff_elem = elem->FirstChildElement("StiffnessProfile"))
    for (const auto* s_elem = stiff_elem->FirstChildElement("Stiffness"); s_elem;
         s_elem = s_elem->NextSiblingElement("Stiffness"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(s_elem, "value", &v, ctx))
        btn.stiffness_profile.push_back(v);
    }

  return btn;
}

components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::components;
  VirtualButton vbtn;

  const auto* shape_elem = elem->FirstChildElement("Shape");
  if (!shape_elem)
  {
    throw_xml_error(elem, ctx, "VirtualButton missing <Shape>");
  }

  vbtn.shape_id = evalTextAttributeRequired(shape_elem, "id", ctx);

  const auto* axis_elem = elem->FirstChildElement("AxisPress");
  if (!axis_elem)
  {
    throw_xml_error(elem, ctx, "VirtualButton missing <AxisPress>");
  }

  const double x = evalNumberAttributeRequired(axis_elem, "x", ctx);
  const double y = evalNumberAttributeRequired(axis_elem, "y", ctx);
  const double z = evalNumberAttributeRequired(axis_elem, "z", ctx);
  vbtn.press_axis = Eigen::Vector3d(x, y, z);
  if (!geometry::isUnitVector(vbtn.press_axis))
  {
    throw_xml_error(axis_elem, ctx, "AxisPress must define a unit vector");
  }

  if (const auto* label_elem = elem->FirstChildElement("Label"))
    vbtn.label = evalTextAttribute(label_elem, "text", "", ctx);

  if (const auto* img_elem = elem->FirstChildElement("Image"))
    vbtn.image_uri = evalTextAttribute(img_elem, "uri", "", ctx);

  return vbtn;
}

components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = elem->FirstChildElement("Type");
  const auto* act_e = elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
  {
    throw_xml_error(elem, ctx, "Joint missing <Type> or <Actuation>");
  }

  joint.type = jointTypeFromString(evalTextAttributeRequired(type_e, "value", ctx).c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value", ctx).c_str());
  joint.initialize_for_type();

  const auto* axis_elem = elem->FirstChildElement("Axis");
  if (!axis_elem)
  {
    throw_xml_error(elem, ctx, "Joint missing <Axis>");
  }

  joint.axes.col(0) = Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x", ctx),
                                      evalNumberAttributeRequired(axis_elem, "y", ctx),
                                      evalNumberAttributeRequired(axis_elem, "z", ctx));
  if (!geometry::isUnitVector(joint.axes.col(0)))
  {
    throw_xml_error(axis_elem, ctx, "<Axis> must define a unit vector");
  }

  const auto* pos_e = elem->FirstChildElement("Position");
  if (!pos_e)
  {
    throw_xml_error(elem, ctx, "Single-DOF joint missing <Position>");
  }

  joint.position(0) = evalNumberAttributeRequired(pos_e, "value", ctx);

  if (const auto* vel_e = elem->FirstChildElement("Velocity"))
    joint.velocity(0) = evalNumberAttributeRequired(vel_e, "value", ctx);
  if (const auto* eff_e = elem->FirstChildElement("Effort"))
    joint.effort(0) = evalNumberAttributeRequired(eff_e, "value", ctx);

  if (const auto* limit_e = elem->FirstChildElement("Limit"))
  {
    auto opt = [&](const tinyxml2::XMLElement* e, const char* attr, double& out) {
      return e && tryEvalNumberAttribute(e, attr, &out, ctx);
    };

    if (const auto* pos = limit_e->FirstChildElement("Position"))
    {
      (void)opt(pos, "min", joint.limit.min_position(0));
      (void)opt(pos, "max", joint.limit.max_position(0));
    }
    if (const auto* vel = limit_e->FirstChildElement("Velocity"))
    {
      (void)opt(vel, "min", joint.limit.min_velocity(0));
      (void)opt(vel, "max", joint.limit.max_velocity(0));
    }
    if (const auto* acc = limit_e->FirstChildElement("Acceleration"))
    {
      (void)opt(acc, "min", joint.limit.min_acceleration(0));
      (void)opt(acc, "max", joint.limit.max_acceleration(0));
    }
    if (const auto* jerk = limit_e->FirstChildElement("Jerk"))
    {
      (void)opt(jerk, "min", joint.limit.min_jerk(0));
      (void)opt(jerk, "max", joint.limit.max_jerk(0));
    }
    if (const auto* eff = limit_e->FirstChildElement("Effort"))
    {
      (void)opt(eff, "min", joint.limit.min_effort(0));
      (void)opt(eff, "max", joint.limit.max_effort(0));
    }
  }

  if (const auto* dyn_e = elem->FirstChildElement("Dynamics"))
  {
    if (const auto* s = dyn_e->FirstChildElement("Stiffness"))
      joint.dynamics.stiffness(0) = evalNumberAttributeRequired(s, "value", ctx);
    if (const auto* d = dyn_e->FirstChildElement("Damping"))
      joint.dynamics.damping(0) = evalNumberAttributeRequired(d, "value", ctx);
    if (const auto* r = dyn_e->FirstChildElement("RestPosition"))
      joint.dynamics.rest_position[0] =
          r->Attribute("value") ? std::optional<double>{ evalNumberAttributeRequired(r, "value", ctx) } : std::nullopt;
    if (const auto* i = dyn_e->FirstChildElement("Inertia"))
      joint.dynamics.inertia(0) = evalNumberAttributeRequired(i, "value", ctx);
  }
  return joint;
}

components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = elem->FirstChildElement("Type");
  const auto* act_e = elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
  {
    throw_xml_error(elem, ctx, "Joint missing <Type> or <Actuation>");
  }

  const std::string type_str = evalTextAttributeRequired(type_e, "value", ctx);

  joint.type = jointTypeFromString(type_str.c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value", ctx).c_str());
  joint.initialize_for_type();  // <-- initialize first
  const int dof = joint.dof();  //     then query DOF

  const auto* axes_elem = elem->FirstChildElement("Axes");
  if (!axes_elem)
  {
    throw_xml_error(elem, ctx, "Multi-DOF joint missing <Axes>");
  }

  int axis_i = 0;
  for (const auto* axis_elem = axes_elem->FirstChildElement("Axis"); axis_elem && axis_i < dof;
       axis_elem = axis_elem->NextSiblingElement("Axis"), ++axis_i)
  {
    joint.axes.col(axis_i) = Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x", ctx),
                                             evalNumberAttributeRequired(axis_elem, "y", ctx),
                                             evalNumberAttributeRequired(axis_elem, "z", ctx));
  }
  if (axis_i != dof)
  {
    throw_xml_error(axes_elem, ctx,
                    "Expected " + std::to_string(dof) + " <Axis> elements for joint type '" + type_str + "'");
  }

  // Validate axes
  switch (joint.type)
  {
    case JointType::FIXED:
      // no axes required
      break;

    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      if (!geometry::isUnitVector(joint.axes.col(0)))
      {
        throw_xml_error(axes_elem, ctx, "Axis for '" + type_str + "' joint must be a unit vector");
      }
      break;

    case JointType::SPHERICAL:
    case JointType::PLANAR:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
      {
        throw_xml_error(axes_elem, ctx, "Axes for '" + type_str + "' joint must be mutually orthonormal");
      }
      if (!geometry::isRightHanded(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
      {
        throw_xml_error(axes_elem, ctx, "Axes for '" + type_str + "' joint must be right-handed");
      }
      break;

    case JointType::FLOATING:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::areVectorsOrthonormal(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
      {
        throw_xml_error(axes_elem, ctx, "FLOATING joint requires two orthonormal triplets");
      }

      if (!geometry::isRightHanded(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::isRightHanded(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
      {
        throw_xml_error(axes_elem, ctx, "FLOATING joint axes must form right-handed triplets");
      }

      break;

    default:
      throw_xml_error(axes_elem, ctx, "Unknown JointType during axes validation");
  }

  // Fill state vectors
  auto fill_vector = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag,
                        const XMLParseContext& ctx) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value", ctx);
  };
  fill_vector(joint.position, elem->FirstChildElement("Positions"), "Position", ctx);
  fill_vector(joint.velocity, elem->FirstChildElement("Velocities"), "Velocity", ctx);
  fill_vector(joint.effort, elem->FirstChildElement("Efforts"), "Effort", ctx);

  // Limits
  auto fill_limits = [](Eigen::VectorXd& min_v, Eigen::VectorXd& max_v, const tinyxml2::XMLElement* limits_elem,
                        const char* tag, const XMLParseContext& ctx) {
    int i = 0;
    for (const auto* e = limits_elem ? limits_elem->FirstChildElement(tag) : nullptr; e && i < min_v.size();
         e = e->NextSiblingElement(tag), ++i)
    {
      min_v(i) = evalNumberAttributeRequired(e, "min", ctx);
      max_v(i) = evalNumberAttributeRequired(e, "max", ctx);
    }
  };
  const auto* limits_elem = elem->FirstChildElement("Limits");
  if (limits_elem)
  {
    fill_limits(joint.limit.min_position, joint.limit.max_position, limits_elem, "Position", ctx);
    fill_limits(joint.limit.min_velocity, joint.limit.max_velocity, limits_elem, "Velocity", ctx);
    fill_limits(joint.limit.min_acceleration, joint.limit.max_acceleration, limits_elem, "Acceleration", ctx);
    fill_limits(joint.limit.min_jerk, joint.limit.max_jerk, limits_elem, "Jerk", ctx);
    fill_limits(joint.limit.min_effort, joint.limit.max_effort, limits_elem, "Effort", ctx);
  }

  // Dynamics
  auto fill_dyn = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag,
                     const XMLParseContext& ctx) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value", ctx);
  };
  const auto* dyn_elem = elem->FirstChildElement("Dynamics");
  if (dyn_elem)
  {
    fill_dyn(joint.dynamics.stiffness, dyn_elem, "Stiffness", ctx);
    fill_dyn(joint.dynamics.damping, dyn_elem, "Damping", ctx);
    fill_dyn(joint.dynamics.inertia, dyn_elem, "Inertia", ctx);

    int i = 0;
    for (const auto* r = dyn_elem->FirstChildElement("RestPosition"); r && i < joint.dynamics.rest_position.size();
         r = r->NextSiblingElement("RestPosition"), ++i)
      joint.dynamics.rest_position[i] = evalNumberAttributeRequired(r, "value", ctx);
  }

  return joint;
}

components::Joint parseJoint(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  if (!elem)
  {
    throw_xml_error(nullptr, ctx, "parseJoint: null <Joint> element");
  }

  const bool hasAxes = elem->FirstChildElement("Axes") != nullptr;

  const bool hasAxis = elem->FirstChildElement("Axis") != nullptr;

  // Heuristic:
  //  - Multi-DOF -> <Axes>
  //  - Single-DOF -> <Axis>

  if (hasAxes)
  {
    return parseMultiDofJoint(elem, ctx);
  }

  if (hasAxis)
  {
    return parseSingleDofJoint(elem, ctx);
  }

  throw_xml_error(elem, ctx,
                  "Joint element must contain either "
                  "<Axis> (single-DOF) or "
                  "<Axes> (multi-DOF)");
}

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  using namespace components;

  if (!elem)
  {
    throw_xml_error(nullptr, ctx, "parseParallelGrasp: null <ParallelGrasp> element");
  }

  ParallelGrasp grasp;

  // ---------------------------------------------------------------------------
  // Required <GraspType>
  // ---------------------------------------------------------------------------

  const auto* gt = elem->FirstChildElement("GraspType");

  if (!gt)
  {
    throw_xml_error(elem, ctx, "ParallelGrasp: <GraspType> is required");
  }

  const std::string gtv = evalTextAttributeRequired(gt, "value", ctx);

  if (gtv == "Internal")
  {
    grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
  }
  else if (gtv == "External")
  {
    grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
  }
  else
  {
    throw_xml_error(gt, ctx,
                    "ParallelGrasp: <GraspType> value must be "
                    "'Internal' or 'External'");
  }

  // ---------------------------------------------------------------------------
  // Optional <GapSize>
  // ---------------------------------------------------------------------------

  if (const auto* gap = elem->FirstChildElement("GapSize"))
  {
    grasp.gap_size = evalNumberAttributeRequired(gap, "value", ctx);
  }

  // ---------------------------------------------------------------------------
  // Required axes
  // ---------------------------------------------------------------------------

  const auto* gap_axis_elem = elem->FirstChildElement("GapAxis");

  if (!gap_axis_elem)
  {
    throw_xml_error(elem, ctx, "ParallelGrasp missing <GapAxis>");
  }

  grasp.gap_axis = parseUnitVector(gap_axis_elem, ctx);

  const auto* rot_axis_elem = elem->FirstChildElement("RotationalAxis");

  if (!rot_axis_elem)
  {
    throw_xml_error(elem, ctx, "ParallelGrasp missing <RotationalAxis>");
  }

  grasp.rotation_axis = parseUnitVector(rot_axis_elem, ctx);

  // ---------------------------------------------------------------------------
  // Optional <RotationalSymmetry>
  // ---------------------------------------------------------------------------

  if (const auto* sym = elem->FirstChildElement("RotationalSymmetry"))
  {
    grasp.rotational_symmetry = evalUIntAttributeRequired(sym, "value", ctx);
  }
  else
  {
    grasp.rotational_symmetry = 1;
  }

  // ---------------------------------------------------------------------------
  // Optional <RealSurfaces>
  // ---------------------------------------------------------------------------

  grasp.contact_shape_ids.clear();

  if (const auto* reals = elem->FirstChildElement("RealSurfaces"))
  {
    for (const auto* surf = reals->FirstChildElement("Shape"); surf; surf = surf->NextSiblingElement("Shape"))
    {
      grasp.contact_shape_ids.push_back(evalTextAttributeRequired(surf, "id", ctx));
    }
  }

  // ---------------------------------------------------------------------------
  // Optional <VirtualSurface>
  // ---------------------------------------------------------------------------

  if (const auto* vs = elem->FirstChildElement("VirtualSurface"))
  {
    const auto* shape = vs->FirstChildElement("Shape");

    if (!shape)
    {
      throw_xml_error(vs, ctx, "<VirtualSurface> missing <Shape>");
    }

    grasp.canonical_surface = parseShape(shape, ctx);
  }

  return grasp;
}

}  // namespace xml
}  // namespace sodf
