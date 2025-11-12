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

inline std::array<Eigen::Vector3d, 3> parseAxesCanonicalXYZ(const tinyxml2::XMLElement* elem, const AxesTagNames& tags,
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
    throw std::runtime_error(std::string("Provide at least two of <") + tags.X + ">, <" + tags.Y + ">, <" + tags.Z +
                             "> at line " + std::to_string(ln));

  auto nearly_zero = [&](const Vector3d& v) { return v.squaredNorm() < tol * tol; };
  auto is_unit = [&](const Vector3d& v) { return std::abs(v.norm() - 1.0) <= 1e2 * tol; };

  Vector3d X = hasX ? parseUnitVector(ex) : Vector3d::Zero();
  Vector3d Y = hasY ? parseUnitVector(ey) : Vector3d::Zero();
  Vector3d Z = hasZ ? parseUnitVector(ez) : Vector3d::Zero();

  if (provided == 3)
  {
    // Do NOT change user-provided axes. Only validate.
    if (!is_unit(X) || !is_unit(Y) || !is_unit(Z))
      throw std::runtime_error("Axes must be unit-length at line " + std::to_string(ln));

    if (std::abs(X.dot(Y)) > tol || std::abs(X.dot(Z)) > tol || std::abs(Y.dot(Z)) > tol)
      throw std::runtime_error("Provided axes must be orthogonal at line " + std::to_string(ln));

    const double det = X.dot(Y.cross(Z));
    if (det < 0.0 - 1e2 * tol)
      throw std::runtime_error("Provided axes are left-handed; expected right-handed at line " + std::to_string(ln));

    if (nearly_zero(Y.cross(Z)) || nearly_zero(Z.cross(X)) || nearly_zero(X.cross(Y)))
      throw std::runtime_error("Degenerate axes at line " + std::to_string(ln));

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

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos)
{
  pos.x() = evalNumberAttributeRequired(element, "x");
  pos.y() = evalNumberAttributeRequired(element, "y");
  pos.z() = evalNumberAttributeRequired(element, "z");
}

void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  double roll = evalNumberAttributeRequired(element, "roll");
  double pitch = evalNumberAttributeRequired(element, "pitch");
  double yaw = evalNumberAttributeRequired(element, "yaw");
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  q.x() = evalNumberAttributeRequired(element, "x");
  q.y() = evalNumberAttributeRequired(element, "y");
  q.z() = evalNumberAttributeRequired(element, "z");
  q.w() = evalNumberAttributeRequired(element, "w");
}

Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* transform_elem)
{
  Eigen::Vector3d pos{ 0, 0, 0 };
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  if (const auto* pos_elem = transform_elem->FirstChildElement("Position"))
    parsePosition(pos_elem, pos);

  if (const auto* ori_elem = transform_elem->FirstChildElement("Orientation"))
    parseOrientationRPY(ori_elem, q);
  else if (const auto* quat_elem = transform_elem->FirstChildElement("Quaternion"))
  {
    parseQuaternion(quat_elem, q);
  }

  return Eigen::Translation3d(pos) * q;
}

geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* transform_elem)
{
  geometry::TransformNode frame;
  (void)tryEvalTextAttribute(transform_elem, "parent", &frame.parent);

  frame.local = parseIsometry3D(transform_elem);
  frame.global = Eigen::Isometry3d::Identity();
  frame.dirty = true;

  return frame;
}

Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* element, double epsilon)
{
  Eigen::Vector3d vec;

  vec.x() = evalNumberAttributeRequired(element, "x");
  vec.y() = evalNumberAttributeRequired(element, "y");
  vec.z() = evalNumberAttributeRequired(element, "z");

  double norm = vec.norm();
  if (std::abs(norm - 1.0) > epsilon)
  {
    std::ostringstream oss;
    oss << "Expected unit vector but got vector with norm " << norm << " at line " << element->GetLineNum();
    throw std::runtime_error(oss.str());
  }
  return vec;
}

// (Note: fixed the spelling to "RightHand")
void validateAxesRightHandOrthonormal(const geometry::Shape& shape, const tinyxml2::XMLElement* elem, double eps = 1e-9)
{
  const int ln = elem ? elem->GetLineNum() : -1;
  const std::size_t n = shape.axes.size();

  if (n != 3)
  {
    throw std::runtime_error("Shape must define exactly 3 axes (X, Y, Z) at line " + std::to_string(ln) + " (got " +
                             std::to_string(n) + ")");
  }

  const Eigen::Vector3d& X = shape.axes[0];
  const Eigen::Vector3d& Y = shape.axes[1];
  const Eigen::Vector3d& Z = shape.axes[2];

  if (!geometry::areVectorsOrthonormal(X, Y, Z, eps))
  {
    throw std::runtime_error("Shape axes are not orthonormal at line " + std::to_string(ln));
  }

  if (!geometry::isRightHanded(X, Y, Z, eps))
  {
    throw std::runtime_error("Shape axes are not right-handed at line " + std::to_string(ln));
  }
}

const tinyxml2::XMLElement* reqChild(const tinyxml2::XMLElement* parent, const char* tag, const char* ctx)
{
  if (const auto* e = parent->FirstChildElement(tag))
    return e;
  throw std::runtime_error(std::string(ctx) + " missing <" + tag + "> at line " + std::to_string(parent->GetLineNum()));
}

// ============================== 2D SHAPES ==============================

geometry::Shape parseRectangleShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;

  geometry::Shape s;
  s.type = geometry::ShapeType::Rectangle;

  // Axes (canonical): X = normal, Y = size_y, Z = size_z
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=normal, Y=size_y, Z=size_z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if ((dims != nullptr) == (verts != nullptr))
    throw std::runtime_error("Rectangle must specify exactly one of <Dimensions> or <Vertices> at line " +
                             std::to_string(ln));

  if (dims)
  {
    const double sy = evalNumberAttributeRequired(dims, "size_y");
    const double sz = evalNumberAttributeRequired(dims, "size_z");
    s.dimensions = { sy, sz };
  }
  else
  {
    // Vertices expressed in (Y,Z) plane (X=0)
    std::size_t count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y");
      const double z = evalNumberAttributeRequired(v, "z");
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count != 4)
      throw std::runtime_error("Rectangle with <Vertices> must have exactly 4 entries at line " +
                               std::to_string(verts->GetLineNum()));
    s.dimensions.clear();
  }

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseCircleShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Circle;

  // Axes (canonical): X = normal, (Y,Z) = reference in-plane axes
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=normal, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = reqChild(elem, "Dimensions", "Circle");
  s.dimensions = { evalNumberAttributeRequired(dims, "radius") };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseTriangleShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Triangle;

  // Axes (canonical): X = normal, Y = base, Z = altitude
  const AxesTagNames tags{ "AxisNormal", "AxisBase", "AxisAltitude" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=normal, Y=base, Z=altitude]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if ((dims != nullptr) == (verts != nullptr))
    throw std::runtime_error("Triangle must specify exactly one of <Dimensions> or <Vertices> at line " +
                             std::to_string(ln));

  if (dims)
  {
    const double base_y = evalNumberAttributeRequired(dims, "base_y");
    const double altitude_z = evalNumberAttributeRequired(dims, "altitude_z");
    const double apex_off_y = evalNumberAttributeRequired(dims, "apex_offset_y");
    s.dimensions = { base_y, apex_off_y, altitude_z };
  }
  else
  {
    // Vertices in (Y,Z) plane (X=0)
    int count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y");
      const double z = evalNumberAttributeRequired(v, "z");
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count != 3)
      throw std::runtime_error("Triangle with <Vertices> must have exactly 3 <Vertex> entries at line " +
                               std::to_string(verts->GetLineNum()));
    s.dimensions.clear();
  }

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parsePolygonShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Polygon;

  // Axes (canonical): X = normal, Y = in-plane y, Z = in-plane z
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=normal, Y, Z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* dims = elem->FirstChildElement("Dimensions"))
    throw std::runtime_error("Polygon must not define <Dimensions> (dimensionless) at line " +
                             std::to_string(dims->GetLineNum()));

  const auto* verts = reqChild(elem, "Vertices", "Polygon");
  std::size_t count = 0;
  for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
  {
    const double y = evalNumberAttributeRequired(v, "y");
    const double z = evalNumberAttributeRequired(v, "z");
    s.vertices.emplace_back(0.0, y, z);  // X=0 (normal axis)
    ++count;
  }
  if (count < 3)
    throw std::runtime_error("Polygon requires at least 3 <Vertex> entries at line " +
                             std::to_string(verts->GetLineNum()));

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parsePlaneShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Plane;

  // Axes (canonical): X = normal, Y = refY, Z = refZ
  const AxesTagNames tags{ "AxisNormal", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=normal, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  const auto* dims = elem->FirstChildElement("Dimensions");
  const auto* verts = elem->FirstChildElement("Vertices");
  const int ln = elem ? elem->GetLineNum() : -1;

  if (dims && verts)
    throw std::runtime_error("Plane must specify at most one of <Dimensions> or <Vertices> at line " +
                             std::to_string(ln));

  if (dims)
  {
    const double sy = evalNumberAttributeRequired(dims, "size_y");
    const double sz = evalNumberAttributeRequired(dims, "size_z");
    if (!(sy > 0.0 && sz > 0.0))
      throw std::runtime_error("Plane <Dimensions> require positive size_y and size_z at line " +
                               std::to_string(dims->GetLineNum()));
    s.dimensions = { sy, sz };
  }
  else if (verts)
  {
    std::size_t count = 0;
    for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double y = evalNumberAttributeRequired(v, "y");
      const double z = evalNumberAttributeRequired(v, "z");
      s.vertices.emplace_back(0.0, y, z);
      ++count;
    }
    if (count < 3)
      throw std::runtime_error("Plane with <Vertices> requires at least 3 <Vertex> entries at line " +
                               std::to_string(verts->GetLineNum()));
    s.dimensions.clear();
  }
  // else: infinite plane

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

// ============================== 3D SHAPES ==============================

geometry::Shape parseBoxShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Box;

  // Axes (canonical): X, Y, Z
  const AxesTagNames tags{ "AxisX", "AxisY", "AxisZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X, Y, Z]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("Box must not define <Vertices> at line " + std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "Box");
  const double x = evalNumberAttributeRequired(dims, "x");
  const double y = evalNumberAttributeRequired(dims, "y");
  const double z = evalNumberAttributeRequired(dims, "z");
  s.dimensions = { x, y, z };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseTriangularPrismShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::TriangularPrism;

  // Axes (canonical): X = extrusion/length_x, Y = base, Z = altitude
  const AxesTagNames tags{ "AxisExtrusion", "AxisBase", "AxisAltitude" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=extrusion, Y=base, Z=altitude]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("TriangularPrism must not define <Vertices> at line " +
                             std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "TriangularPrism");
  const double length_x = evalNumberAttributeRequired(dims, "length_x");      // along X
  const double base_y = evalNumberAttributeRequired(dims, "base_y");          // along Y
  const double altitude_z = evalNumberAttributeRequired(dims, "altitude_z");  // along Z
  double apex_offset_y = 0.0;
  (void)tryEvalNumberAttribute(dims, "apex_offset_y", &apex_offset_y);

  if (!(length_x > 0.0 && base_y > 0.0 && altitude_z > 0.0))
    throw std::runtime_error("TriangularPrism requires positive length_x, base_y, altitude_z at line " +
                             std::to_string(dims->GetLineNum()));

  s.dimensions = { base_y, apex_offset_y, altitude_z, length_x };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseCylinderShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Cylinder;

  // Axes (canonical): X = symmetry/height axis, (Y,Z) = reference in-plane
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("Cylinder must not define <Vertices> at line " + std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "Cylinder");
  const double radius = evalNumberAttributeRequired(dims, "radius");
  const double height_x = evalNumberAttributeRequired(dims, "height_x");
  if (!(radius > 0.0 && height_x > 0.0))
    throw std::runtime_error("Cylinder requires positive radius and height_x at line " +
                             std::to_string(dims->GetLineNum()));

  s.dimensions = { radius, height_x };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseSphereShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Sphere;

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("Sphere must not define <Vertices> at line " + std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "Sphere");
  const double radius = evalNumberAttributeRequired(dims, "radius");
  if (!(radius > 0.0))
    throw std::runtime_error("Sphere requires positive radius at line " + std::to_string(dims->GetLineNum()));

  s.dimensions = { radius };
  // No axes to validate (isotropic)
  return s;
}

geometry::Shape parseConeShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::Cone;

  // Axes (canonical): X = symmetry/height, (Y,Z) = reference
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("Cone must not define <Vertices> at line " + std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "Cone");
  const double base_r = evalNumberAttributeRequired(dims, "base_radius");
  const double top_r = evalNumberAttributeRequired(dims, "top_radius");
  const double height_x = evalNumberAttributeRequired(dims, "height_x");

  if (!(height_x > 0.0) || base_r < 0.0 || top_r < 0.0)
    throw std::runtime_error("Cone requires base_radius ≥ 0, top_radius ≥ 0, height_x > 0 at line " +
                             std::to_string(dims->GetLineNum()));
  if (base_r == 0.0 && top_r == 0.0)
    throw std::runtime_error("Cone invalid: base_radius and top_radius cannot both be zero at line " +
                             std::to_string(dims->GetLineNum()));

  s.dimensions = { base_r, top_r, height_x };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseSphericalSegmentShape(const tinyxml2::XMLElement* elem)
{
  using namespace sodf::geometry;
  geometry::Shape s;
  s.type = geometry::ShapeType::SphericalSegment;

  // Axes (canonical): X = symmetry/height, (Y,Z) = reference
  const AxesTagNames tags{ "AxisSymmetry", "AxisRefY", "AxisRefZ" };
  auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X=symmetry, Y=refY, Z=refZ]
  s.axes.assign(axesXYZ.begin(), axesXYZ.end());

  if (const auto* verts = elem->FirstChildElement("Vertices"))
    throw std::runtime_error("SphericalSegment must not define <Vertices> at line " +
                             std::to_string(verts->GetLineNum()));

  const auto* dims = reqChild(elem, "Dimensions", "SphericalSegment");

  const bool has_r1 = dims->Attribute("base_radius");
  const bool has_r2 = dims->Attribute("top_radius");
  const bool has_hx = dims->Attribute("height_x");

  int have = (has_r1 ? 1 : 0) + (has_r2 ? 1 : 0) + (has_hx ? 1 : 0);
  if (have < 2)
    throw std::runtime_error(
        "SphericalSegment must provide at least two of base_radius, top_radius, height_x at line " +
        std::to_string(dims->GetLineNum()));

  double r1 = has_r1 ? evalNumberAttributeRequired(dims, "base_radius") : 0.0;
  double r2 = has_r2 ? evalNumberAttributeRequired(dims, "top_radius") : 0.0;
  double hx = has_hx ? evalNumberAttributeRequired(dims, "height_x") : 0.0;

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
      throw std::runtime_error("SphericalSegment <Dimensions> are incompatible: no valid sphere exists at line " +
                               std::to_string(dims->GetLineNum()));
  }

  s.dimensions = { r1, r2, hx };

  validateAxesRightHandOrthonormal(s, elem);
  return s;
}

geometry::Shape parseMeshShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
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
    const int ln = elem ? elem->GetLineNum() : -1;
    throw std::runtime_error("Mesh must provide either 0 axes (defaults to identity) or at least 2 axes at line " +
                             std::to_string(ln));
  }
  else
  {
    // 2 or 3 provided → parse/complete/validate
    const AxesTagNames tags{ "AxisX", "AxisY", "AxisZ" };
    auto axesXYZ = parseAxesCanonicalXYZ(elem, tags);  // [X, Y, Z], right-handed ONB
    s.axes.assign(axesXYZ.begin(), axesXYZ.end());
    validateAxesRightHandOrthonormal(s, elem);
  }

  // --- Scale (optional, defaults to 1,1,1) ---
  if (const auto* scale = elem->FirstChildElement("Scale"))
  {
    s.scale.x() = evalNumberAttribute(scale, "x", 1.0);
    s.scale.y() = evalNumberAttribute(scale, "y", 1.0);
    s.scale.z() = evalNumberAttribute(scale, "z", 1.0);
  }

  // --- Exactly one of <External> or <Inline> ---
  const auto* external = elem->FirstChildElement("External");
  const auto* internal = elem->FirstChildElement("Inline");
  const bool hasExternal = (external != nullptr);
  const bool hasInline = (internal != nullptr);
  if (hasExternal == hasInline)
  {
    throw std::runtime_error(
        std::string("<Shape type='Mesh'> must contain exactly one of <External> or <Inline> at line ") +
        std::to_string(elem->GetLineNum()));
  }

  if (hasExternal)
  {
    // External URI form
    std::string uri = evalTextAttributeRequired(external, "uri");
    uri = resolve_realtive_uri(external, doc, uri);  // (uses your resolver; keep spelling as in your codebase)
    s.mesh = MeshRef{ std::move(uri) };
  }
  else
  {
    // Inline (TriangleMesh) form
    const auto* vs = internal->FirstChildElement("Vertices");
    const auto* fs = internal->FirstChildElement("Faces");
    if (!vs || !fs)
    {
      throw std::runtime_error(std::string("<Inline> must contain <Vertices> and <Faces> at line ") +
                               std::to_string(internal->GetLineNum()));
    }

    auto M = std::make_shared<TriangleMesh>();
    M->V.reserve(16);
    M->F.reserve(32);

    // <Vertices>
    for (const tinyxml2::XMLElement* v = vs->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    {
      const double x = evalNumberAttributeRequired(v, "x");
      const double y = evalNumberAttributeRequired(v, "y");
      const double z = evalNumberAttributeRequired(v, "z");
      M->V.emplace_back(x, y, z);
    }
    if (M->V.empty())
    {
      throw std::runtime_error(std::string("<Vertices> is empty at line ") + std::to_string(vs->GetLineNum()));
    }

    // <Faces> (0-based indices)
    for (const tinyxml2::XMLElement* f = fs->FirstChildElement("Face"); f; f = f->NextSiblingElement("Face"))
    {
      const uint32_t a = evalUIntAttributeRequired(f, "a");
      const uint32_t b = evalUIntAttributeRequired(f, "b");
      const uint32_t c = evalUIntAttributeRequired(f, "c");
      M->F.push_back({ a, b, c });
    }
    if (M->F.empty())
    {
      throw std::runtime_error(std::string("<Faces> is empty at line ") + std::to_string(fs->GetLineNum()));
    }

    // Validate indices
    const uint32_t nV = static_cast<uint32_t>(M->V.size());
    for (const auto& tri : M->F)
    {
      if (tri[0] >= nV || tri[1] >= nV || tri[2] >= nV)
      {
        throw std::runtime_error(std::string("Face index out of range in <Inline> mesh at line ") +
                                 std::to_string(fs->GetLineNum()));
      }
    }

    s.mesh = std::move(M);
  }

  return s;
}

geometry::Shape parseLineShape(const tinyxml2::XMLElement* elem)
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

  const int ln = elem ? elem->GetLineNum() : -1;

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
      throw std::runtime_error("Line: Vertices mode forbids <AxisDirection> and <Dimensions> at line " +
                               std::to_string(ln));
    }
  }
  else
  {
    // no vertices → must have BOTH axis and dims
    if (has_axis != has_dims)
    {
      throw std::runtime_error(
          "Line: Provide EITHER two <Vertex> tags OR BOTH <AxisDirection> and <Dimensions length=\"...\"> at line " +
          std::to_string(ln));
    }
    if (!has_axis && !has_dims)
    {
      throw std::runtime_error("Line: Missing specification. Provide either vertices or (AxisDirection + Dimensions).");
    }
  }

  // --- Mode A: Axis + Dimensions --------------------------------------------
  if (!has_verts)
  {
    // Parse axis (single unit vector only; no 3-axis frame)
    const Vector3d axis = parseUnitVector(axis_elem);
    s.axes.clear();
    s.axes.push_back(axis);

    // Require <Dimensions length="..."> and length > 0
    if (!dims_elem->FindAttribute("length"))
    {
      throw std::runtime_error("Line: <Dimensions> must include a 'length' attribute at line " +
                               std::to_string(dims_elem->GetLineNum()));
    }
    const double length = evalNumberAttributeRequired(dims_elem, "length");
    if (!(length > 0.0))
    {
      throw std::runtime_error("Line: length must be positive at line " + std::to_string(dims_elem->GetLineNum()));
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
    throw std::runtime_error("Line: Must have exactly two <Vertex> tags at line " + std::to_string(ln));
  }

  s.vertices.reserve(2);
  for (int i = 0; i < 2; ++i)
  {
    const XMLElement* v = Vxml[i];
    const double x = evalNumberAttributeRequired(v, "x");
    const double y = evalNumberAttributeRequired(v, "y");
    double z = 0.0;
    if (v->FindAttribute("z"))
      tryEvalNumberAttribute(v, "z", &z);
    s.vertices.emplace_back(x, y, z);
  }

  // Either both specify z or neither (enforce 2D vs 3D consistently)
  const bool z0 = (Vxml[0]->FindAttribute("z") != nullptr);
  const bool z1 = (Vxml[1]->FindAttribute("z") != nullptr);
  if (z0 != z1)
  {
    throw std::runtime_error("Line: Either both vertices specify 'z' or neither does (line " + std::to_string(ln) +
                             ").");
  }

  // In vertices mode, axes and dimensions are irrelevant.
  s.axes.clear();
  s.dimensions.clear();

  return s;
}

void validateOriginAuthoring(const geometry::Shape& s, bool origin_present, const tinyxml2::XMLElement* elem)
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
    throw std::runtime_error("Mesh 'origin' must be 'Native' or omitted (line " + line + ")");
  }

  // Non-mesh primitives only below
  if (!isPrimitive(s.type))
    return;  // ShapeType::None etc.

  const bool has_dims = primitiveHasDimensions(s);
  const bool has_verts = primitiveHasVertices(s);

  // (3) 3D primitives cannot have vertices -> must use Mesh
  if (s.type != ShapeType::Line && isPrimitive3D(s.type) && has_verts)
  {
    throw std::runtime_error("3D primitives cannot use <Vertices> (except Line). Use type='Mesh' instead (line " +
                             line + ")");
  }

  // (2) Primitives with dimensions: origin required and must be != Native
  if (has_dims)
  {
    if (!origin_present)
    {
      throw std::runtime_error("Primitive with <Dimensions> requires explicit 'origin' attribute (line " + line + ")");
    }
    if (s.origin == OriginPolicy::Native)
    {
      throw std::runtime_error("Primitive with <Dimensions> must have origin != 'Native' (line " + line + ")");
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
    throw std::runtime_error("Vertex-authored primitive allows only origin='Native' or no attribute (line " + line +
                             ")");
  }

  // If neither dims nor vertices are provided for a primitive, it's ill-formed
  throw std::runtime_error("Primitive is missing authoring data (<Dimensions> or <Vertices>) (line " + line + ")");
}

std::optional<geometry::OriginPolicy> parseOriginPolicyAttr(const tinyxml2::XMLElement* elem)
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

  throw std::runtime_error(std::string("Unknown origin policy '") + std::string(v) + "' at line " +
                           std::to_string(elem->GetLineNum()));
}

geometry::Shape parseShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
{
  const std::string type_str = evalTextAttributeRequired(elem, "type");
  geometry::ShapeType type = geometry::shapeTypeFromString(type_str.c_str());

  geometry::Shape s;
  switch (type)
  {
    case geometry::ShapeType::Rectangle:
      s = parseRectangleShape(elem);
      break;
    case geometry::ShapeType::Circle:
      s = parseCircleShape(elem);
      break;
    case geometry::ShapeType::Triangle:
      s = parseTriangleShape(elem);
      break;
    case geometry::ShapeType::Polygon:
      s = parsePolygonShape(elem);
      break;
    case geometry::ShapeType::Box:
      s = parseBoxShape(elem);
      break;
    case geometry::ShapeType::TriangularPrism:
      s = parseTriangularPrismShape(elem);
      break;
    case geometry::ShapeType::Cylinder:
      s = parseCylinderShape(elem);
      break;
    case geometry::ShapeType::Sphere:
      s = parseSphereShape(elem);
      break;
    case geometry::ShapeType::Cone:
      s = parseConeShape(elem);
      break;
    case geometry::ShapeType::SphericalSegment:
      s = parseSphericalSegmentShape(elem);
      break;
    case geometry::ShapeType::Plane:
      s = parsePlaneShape(elem);
      break;
    case geometry::ShapeType::Mesh:
      s = parseMeshShape(doc, elem);
      break;
    case geometry::ShapeType::Line:
      s = parseLineShape(elem);
      break;
    default:
      throw std::runtime_error("Unknown shape type in parseShapeElement at line " + std::to_string(elem->GetLineNum()));
  }

  // Parse optional origin attribute
  const auto origin_attr = parseOriginPolicyAttr(elem);
  const bool origin_present = origin_attr.has_value();
  if (origin_present)
  {
    s.origin = *origin_attr;  // default was Native; overwrite if present
  }

  // Enforce your rules
  validateOriginAuthoring(s, origin_present, elem);

  return s;
}

geometry::StackedShape parseStackedShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* stacked_elem)
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
  for (const tinyxml2::XMLElement* shape_elem = stacked_elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    geometry::Shape shape = parseShape(doc, shape_elem);

    const tinyxml2::XMLElement* trans_elem = shape_elem->FirstChildElement("Transform");
    Eigen::Isometry3d T_abs = Eigen::Isometry3d::Identity();
    if (trans_elem)
      T_abs = parseIsometry3D(trans_elem);  // absolute w.r.t. stack/base frame

    // Store absolute transform as-is (field name may still be 'relative_transform')
    stack.shapes.push_back({ shape, T_abs });
  }

  return stack;
}

components::Button parseButton(const tinyxml2::XMLElement* btn_elem)
{
  using namespace sodf::components;

  Button btn;
  btn.type = buttonTypeFromString(evalTextAttributeRequired(btn_elem, "type").c_str());

  const tinyxml2::XMLElement* link_elem = btn_elem->FirstChildElement("Link");
  if (!link_elem)
    throw std::runtime_error("Button missing <Link> at line " + std::to_string(btn_elem->GetLineNum()));
  btn.link_id = evalTextAttributeRequired(link_elem, "ref");

  const tinyxml2::XMLElement* joint_elem = btn_elem->FirstChildElement("Joint");
  if (!joint_elem)
    throw std::runtime_error("Button missing <Joint> at line " + std::to_string(btn_elem->GetLineNum()));
  btn.joint_id = evalTextAttributeRequired(joint_elem, "ref");

  if (const auto* detent_elem = btn_elem->FirstChildElement("DetentSpacing"))
    tryEvalNumberAttribute(detent_elem, "value", &btn.detent_spacing);

  if (const auto* rest_elem = btn_elem->FirstChildElement("RestPositions"))
    for (const auto* pos_elem = rest_elem->FirstChildElement("Position"); pos_elem;
         pos_elem = pos_elem->NextSiblingElement("Position"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(pos_elem, "value", &v))
        btn.rest_positions.push_back(v);
    }

  if (const auto* stiff_elem = btn_elem->FirstChildElement("StiffnessProfile"))
    for (const auto* s_elem = stiff_elem->FirstChildElement("Stiffness"); s_elem;
         s_elem = s_elem->NextSiblingElement("Stiffness"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(s_elem, "value", &v))
        btn.stiffness_profile.push_back(v);
    }

  return btn;
}

components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem)
{
  using namespace sodf::components;
  VirtualButton vbtn;

  const auto* shape_elem = vb_elem->FirstChildElement("Shape");
  if (!shape_elem)
    throw std::runtime_error("VirtualButton missing <Shape> at line " + std::to_string(vb_elem->GetLineNum()));
  vbtn.shape_id = evalTextAttributeRequired(shape_elem, "id");

  const auto* axis_elem = vb_elem->FirstChildElement("AxisPress");
  if (!axis_elem)
    throw std::runtime_error("VirtualButton missing <AxisPress> at line " + std::to_string(vb_elem->GetLineNum()));
  const double x = evalNumberAttributeRequired(axis_elem, "x");
  const double y = evalNumberAttributeRequired(axis_elem, "y");
  const double z = evalNumberAttributeRequired(axis_elem, "z");
  vbtn.press_axis = Eigen::Vector3d(x, y, z);
  if (!geometry::isUnitVector(vbtn.press_axis))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  if (const auto* label_elem = vb_elem->FirstChildElement("Label"))
    vbtn.label = evalTextAttribute(label_elem, "text", "");

  if (const auto* img_elem = vb_elem->FirstChildElement("Image"))
    vbtn.image_uri = evalTextAttribute(img_elem, "uri", "");

  return vbtn;
}

components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = joint_elem->FirstChildElement("Type");
  const auto* act_e = joint_elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
    throw std::runtime_error("Joint missing <Type> or <Actuation>");

  joint.type = jointTypeFromString(evalTextAttributeRequired(type_e, "value").c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value").c_str());
  joint.initialize_for_type();

  const auto* axis_elem = joint_elem->FirstChildElement("Axis");
  if (!axis_elem)
    throw std::runtime_error("Missing <Axis>");
  joint.axes.col(0) =
      Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x"), evalNumberAttributeRequired(axis_elem, "y"),
                      evalNumberAttributeRequired(axis_elem, "z"));
  if (!geometry::isUnitVector(joint.axes.col(0)))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  const auto* pos_e = joint_elem->FirstChildElement("Position");
  if (!pos_e)
    throw std::runtime_error("Single-DOF joint missing <Position>");
  joint.position(0) = evalNumberAttributeRequired(pos_e, "value");

  if (const auto* vel_e = joint_elem->FirstChildElement("Velocity"))
    joint.velocity(0) = evalNumberAttributeRequired(vel_e, "value");
  if (const auto* eff_e = joint_elem->FirstChildElement("Effort"))
    joint.effort(0) = evalNumberAttributeRequired(eff_e, "value");

  if (const auto* limit_e = joint_elem->FirstChildElement("Limit"))
  {
    auto opt = [&](const tinyxml2::XMLElement* e, const char* attr, double& out) {
      return e && tryEvalNumberAttribute(e, attr, &out);
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

  if (const auto* dyn_e = joint_elem->FirstChildElement("Dynamics"))
  {
    if (const auto* s = dyn_e->FirstChildElement("Stiffness"))
      joint.dynamics.stiffness(0) = evalNumberAttributeRequired(s, "value");
    if (const auto* d = dyn_e->FirstChildElement("Damping"))
      joint.dynamics.damping(0) = evalNumberAttributeRequired(d, "value");
    if (const auto* r = dyn_e->FirstChildElement("RestPosition"))
      joint.dynamics.rest_position[0] =
          r->Attribute("value") ? std::optional<double>{ evalNumberAttributeRequired(r, "value") } : std::nullopt;
    if (const auto* i = dyn_e->FirstChildElement("Inertia"))
      joint.dynamics.inertia(0) = evalNumberAttributeRequired(i, "value");
  }
  return joint;
}

components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = joint_elem->FirstChildElement("Type");
  const auto* act_e = joint_elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
    throw std::runtime_error("Joint missing <Type> or <Actuation>");

  joint.type = jointTypeFromString(evalTextAttributeRequired(type_e, "value").c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value").c_str());
  joint.initialize_for_type();  // <-- initialize first
  const int dof = joint.dof();  //     then query DOF

  const auto* axes_elem = joint_elem->FirstChildElement("Axes");
  if (!axes_elem)
    throw std::runtime_error("Multi-DOF joint missing <Axes> at line " + std::to_string(joint_elem->GetLineNum()));

  int axis_i = 0;
  for (const auto* axis_elem = axes_elem->FirstChildElement("Axis"); axis_elem && axis_i < dof;
       axis_elem = axis_elem->NextSiblingElement("Axis"), ++axis_i)
  {
    joint.axes.col(axis_i) =
        Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x"), evalNumberAttributeRequired(axis_elem, "y"),
                        evalNumberAttributeRequired(axis_elem, "z"));
  }
  if (axis_i != dof)
    throw std::runtime_error("There must be " + std::to_string(dof) + " <Axis> tags for a " +
                             evalTextAttributeRequired(type_e, "value") + " joint at line " +
                             std::to_string(axes_elem->GetLineNum()));

  // Validate axes
  switch (joint.type)
  {
    case JointType::FIXED:
      // no axes required
      break;

    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      if (!geometry::isUnitVector(joint.axes.col(0)))
        throw std::runtime_error(std::string("Axis for ") + evalTextAttributeRequired(type_e, "value") +
                                 " joint must be a unit vector at line " + std::to_string(axes_elem->GetLineNum()));
      break;

    case JointType::SPHERICAL:
    case JointType::PLANAR:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
        throw std::runtime_error(std::string("Axes for ") + evalTextAttributeRequired(type_e, "value") +
                                 " joint must be mutually orthonormal at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      if (!geometry::isRightHanded(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
        throw std::runtime_error(std::string("Axes for ") + evalTextAttributeRequired(type_e, "value") +
                                 " joint must be mutually right handed at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;

    case JointType::FLOATING:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::areVectorsOrthonormal(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
        throw std::runtime_error("Axes for FLOATING joint must be two mutually orthonormal triplets at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      if (!geometry::isRightHanded(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::isRightHanded(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
        throw std::runtime_error("Axes for FLOATING joint must be two mutually right handed triplets at line " +
                                 std::to_string(axes_elem->GetLineNum()));

      break;

    default:
      throw std::runtime_error("Unknown JointType for axes validation at line " +
                               std::to_string(axes_elem->GetLineNum()));
  }

  // Fill state vectors
  auto fill_vector = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value");
  };
  fill_vector(joint.position, joint_elem->FirstChildElement("Positions"), "Position");
  fill_vector(joint.velocity, joint_elem->FirstChildElement("Velocities"), "Velocity");
  fill_vector(joint.effort, joint_elem->FirstChildElement("Efforts"), "Effort");

  // Limits
  auto fill_limits = [](Eigen::VectorXd& min_v, Eigen::VectorXd& max_v, const tinyxml2::XMLElement* limits_elem,
                        const char* tag) {
    int i = 0;
    for (const auto* e = limits_elem ? limits_elem->FirstChildElement(tag) : nullptr; e && i < min_v.size();
         e = e->NextSiblingElement(tag), ++i)
    {
      min_v(i) = evalNumberAttributeRequired(e, "min");
      max_v(i) = evalNumberAttributeRequired(e, "max");
    }
  };
  const auto* limits_elem = joint_elem->FirstChildElement("Limits");
  if (limits_elem)
  {
    fill_limits(joint.limit.min_position, joint.limit.max_position, limits_elem, "Position");
    fill_limits(joint.limit.min_velocity, joint.limit.max_velocity, limits_elem, "Velocity");
    fill_limits(joint.limit.min_acceleration, joint.limit.max_acceleration, limits_elem, "Acceleration");
    fill_limits(joint.limit.min_jerk, joint.limit.max_jerk, limits_elem, "Jerk");
    fill_limits(joint.limit.min_effort, joint.limit.max_effort, limits_elem, "Effort");
  }

  // Dynamics
  auto fill_dyn = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value");
  };
  const auto* dyn_elem = joint_elem->FirstChildElement("Dynamics");
  if (dyn_elem)
  {
    fill_dyn(joint.dynamics.stiffness, dyn_elem, "Stiffness");
    fill_dyn(joint.dynamics.damping, dyn_elem, "Damping");
    fill_dyn(joint.dynamics.inertia, dyn_elem, "Inertia");

    int i = 0;
    for (const auto* r = dyn_elem->FirstChildElement("RestPosition"); r && i < joint.dynamics.rest_position.size();
         r = r->NextSiblingElement("RestPosition"), ++i)
      joint.dynamics.rest_position[i] = evalNumberAttributeRequired(r, "value");
  }

  return joint;
}

components::Joint parseJoint(const tinyxml2::XMLElement* joint_elem)
{
  // Heuristic: multi-DOF if <Axes> exists, single-DOF if <Axis> exists
  if (joint_elem->FirstChildElement("Axes"))
    return parseMultiDofJoint(joint_elem);
  if (joint_elem->FirstChildElement("Axis"))
    return parseSingleDofJoint(joint_elem);
  throw std::runtime_error("Joint element is missing <Axis> or <Axes>");
}

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
{
  using namespace components;
  ParallelGrasp grasp;

  const auto* gt = elem->FirstChildElement("GraspType");
  if (!gt)
    throw std::runtime_error("ParallelGrasp: <GraspType> is required.");
  const std::string gtv = evalTextAttributeRequired(gt, "value");
  if (gtv == "Internal")
    grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
  else if (gtv == "External")
    grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
  else
    throw std::runtime_error("ParallelGrasp: <GraspType> value must be 'Internal' or 'External'");

  if (const auto* gap = elem->FirstChildElement("GapSize"))
    grasp.gap_size = evalNumberAttributeRequired(gap, "value");
  grasp.gap_axis = parseUnitVector(elem->FirstChildElement("GapAxis"));
  grasp.rotation_axis = parseUnitVector(elem->FirstChildElement("RotationalAxis"));

  if (const auto* sym = elem->FirstChildElement("RotationalSymmetry"))
    grasp.rotational_symmetry = evalUIntAttributeRequired(sym, "value");  // <- integer semantics
  else
    grasp.rotational_symmetry = 1;

  grasp.contact_shape_ids.clear();
  if (const auto* reals = elem->FirstChildElement("RealSurfaces"))
    for (const auto* surf = reals->FirstChildElement("Shape"); surf; surf = surf->NextSiblingElement("Shape"))
      grasp.contact_shape_ids.push_back(evalTextAttributeRequired(surf, "id"));

  if (const auto* vs = elem->FirstChildElement("VirtualSurface"))
  {
    const auto* shape = vs->FirstChildElement("Shape");
    if (!shape)
      throw std::runtime_error("<VirtualSurface> missing <Shape> at line " + std::to_string(vs->GetLineNum()));
    grasp.canonical_surface = parseShape(doc, shape);
  }
  return grasp;
}

}  // namespace xml
}  // namespace sodf
