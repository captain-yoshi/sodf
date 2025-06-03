#include <sodf/geometry/shape.h>

namespace sodf {
namespace geometry {

bool is2DShape(const Shape& shape)
{
  switch (shape.type)
  {
    case ShapeType::Line:
      // Check both vertices have z == 0 (within tolerance)
      if (shape.vertices.size() == 2)
      {
        const double tol = 1e-8;
        return std::abs(shape.vertices[0].z()) < tol && std::abs(shape.vertices[1].z()) < tol;
      }
      return false;
    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Polygon:
    case ShapeType::Triangle:
      return true;
    default:
      return false;
  }
}

Eigen::Vector3d getShapeNormalAxis(const Shape& shape)
{
  switch (shape.type)
  {
    case ShapeType::Line:
    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Triangle:
    case ShapeType::Polygon:
    case ShapeType::Plane:
      // 2D shapes and planes: normal is stored in axes[0]
      return shape.axes.at(0).normalized();

    case ShapeType::Box:
    case ShapeType::Mesh:
      // No single "normal", but for a box you might pick +Z of the box, or require a face index
      throw std::runtime_error("getNormal() not defined for Box/Mesh without additional info.");

    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      // "Normal" usually refers to symmetry axis (axes[0])
      return shape.axes.at(0).normalized();

    case ShapeType::Sphere:
      throw std::runtime_error("getNormal() not defined for Sphere (normal depends on query point).");

    default:
      throw std::runtime_error("getNormal() not implemented for this ShapeType.");
  }
}

Eigen::Vector3d getShapeCentroid(const Shape& shape)
{
  auto& type = shape.type;
  auto& vertices = shape.vertices;
  auto& dimensions = shape.dimensions;
  auto& axes = shape.axes;

  switch (type)
  {
    case ShapeType::Line:
      if (vertices.size() < 2)
        throw std::runtime_error("Line shape requires two vertices for centroid calculation.");
      return 0.5 * (vertices.at(0) + vertices.at(1));

    case ShapeType::Triangle:
    case ShapeType::Polygon:
      if (vertices.empty())
        throw std::runtime_error("Polygon/Triangle shape requires at least one vertex for centroid calculation.");
      {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto& v : vertices)
          c += v;
        return c / vertices.size();
      }

    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Plane:
    case ShapeType::Box:
    case ShapeType::Sphere:
      return Eigen::Vector3d::Zero();

    case ShapeType::Cylinder:
      // axes[0]: symmetry axis, dimensions[1]: height
      if (axes.size() < 1 || dimensions.size() < 2)
        throw std::runtime_error("Cylinder shape requires symmetry axis and 2 dimensions (radius, height).");
      return axes[0].normalized() * (dimensions[1] / 2.0);

    case ShapeType::Cone:
      // axes[0]: symmetry axis, dimensions[2]: height
      if (axes.size() < 1 || dimensions.size() < 3)
        throw std::runtime_error(
            "Cone shape requires symmetry axis and 3 dimensions (base_radius, top_radius, height).");
      return axes[0].normalized() * (dimensions[2] / 4.0);

    case ShapeType::SphericalSegment:
      // axes[0]: symmetry axis, dimensions[0]=base_radius, dimensions[1]=top_radius, dimensions[2]=height
      if (axes.size() < 1 || dimensions.size() < 3)
        throw std::runtime_error(
            "SphericalSegment requires symmetry axis and 3 dimensions (base_radius, top_radius, height).");
      {
        double R = dimensions[0] > 0 ? dimensions[0] : dimensions[1];
        double h = dimensions[2];
        // (A more precise formula may be needed for your use-case)
        double z = (3.0 * R - h) * h * h / (4.0 * (3.0 * R * R - 3.0 * R * h + h * h));
        return axes[0].normalized() * z;
      }

    case ShapeType::Mesh:
      throw std::runtime_error("getShapeCentroid() not implemented for Mesh (requires mesh analysis).");

    default:
      throw std::runtime_error("getShapeCentroid() not implemented for this ShapeType.");
  }
}

// --- Helper: extract height of shape for stacking ---
double shapeHeight(const Shape& shape)
{
  // Add more shape types as needed
  switch (shape.type)
  {
    case ShapeType::SphericalSegment:
    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::Box:
      // Assumes height is always last dimension (see your convention)
      return shape.dimensions.back();
    // For others, add as appropriate
    default:
      throw std::runtime_error("Automatic stacking: unknown height for this shape type");
  }
}

// --- Helper: extract symmetry axis from axes vector (index 0 per convention) ---
Eigen::Vector3d getShapeSymmetryAxis(const Shape& shape)
{
  if (shape.axes.empty())
    throw std::runtime_error("Shape missing axes info for stacking");
  return shape.axes[0].normalized();
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

}  // namespace geometry
}  // namespace sodf
