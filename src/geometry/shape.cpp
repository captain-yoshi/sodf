#include <sodf/geometry/shape.h>

#include <iostream>

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

    case ShapeType::TriangularPrism:
    {
      // Option B: dimensions = { bw, bh, depth, u }
      // axes = { base_x, base_y, depth_axis }
      if (dimensions.size() < 4 || axes.size() < 3)
        throw std::runtime_error("TriangularPrism requires dims{bw,bh,depth,u} and axes{base_x,base_y,depth_axis}");

      const double bw = dimensions[0];
      const double bh = dimensions[1];
      const double d = dimensions[2];
      const double u = dimensions[3];

      const Eigen::Vector3d& base_x = axes[0];
      const Eigen::Vector3d& base_y = axes[1];
      const Eigen::Vector3d& depth_axis = axes[2];

      // Base triangle centroid in the base plane:
      // ( (0,0) + (bw,0) + (u,bh) ) / 3
      Eigen::Vector2d c2((0.0 + bw + u) / 3.0, (0.0 + 0.0 + bh) / 3.0);
      return c2.x() * base_x.normalized() + c2.y() * base_y.normalized() + (d * 0.5) * depth_axis.normalized();
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
    case ShapeType::TriangularPrism:
      if (shape.dimensions.size() < 3)
        throw std::runtime_error("TriangularPrism expects at least 3 dimensions");
      return shape.dimensions[2];

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

double shapeBaseRadius(const geometry::Shape& shape)
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

double shapeTopRadius(const geometry::Shape& shape)
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

double shapeMaxRadius(const geometry::Shape& shape)
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

const Eigen::Vector3d& getShapeNormalAxis(const Shape& shape)
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
      return shape.axes.at(0);

    case ShapeType::Box:
    case ShapeType::Mesh:
      // No single "normal", but for a box you might pick +Z of the box, or require a face index
      throw std::runtime_error("getNormal() not defined for Box/Mesh without additional info.");

    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      // "Normal" usually refers to symmetry axis (axes[0])
      return shape.axes.at(0);

    case ShapeType::Sphere:
      throw std::runtime_error("getNormal() not defined for Sphere (normal depends on query point).");

    default:
      throw std::runtime_error("getNormal() not implemented for this ShapeType.");
  }
}

const Eigen::Vector3d& getShapeReferenceAxis(const geometry::Shape& shape)
{
  using ShapeType = geometry::ShapeType;
  switch (shape.type)
  {
    case ShapeType::TriangularPrism:
      // Base plane exists; expose in-plane "reference" axis (axes[1] = base_y)
      if (shape.axes.size() < 2)
        throw std::runtime_error("TriangularPrism requires axes[1] (base_y) for reference axis.");
      return shape.axes[1];
    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      if (shape.axes.size() < 2)
        throw std::runtime_error("Shape requires reference axis (axes[1]) but not found.");
      return shape.axes[1];

    default:
      throw std::runtime_error("getShapeReferenceAxis() is only valid for shapes that define a base plane.");
  }
}

const Eigen::Vector3d& getShapeSymmetryAxis(const geometry::Shape& shape)
{
  using ShapeType = geometry::ShapeType;
  switch (shape.type)
  {
    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      if (shape.axes.size() < 1)
        throw std::runtime_error("Shape requires symmetry axis (axes[0]) but none provided.");
      return shape.axes[0];

    default:
      throw std::runtime_error("getShapeSymmetryAxis() is only valid for axisymmetric 3D shapes.");
  }
}

double getTopRadiusAtHeight(double base_radius, double top_radius, double total_height, double fill_height)
{
  if (fill_height <= 0.0 || fill_height > total_height)
    return 0.0;

  // Compute z0 (sphere center relative to base)
  double z0 =
      (top_radius * top_radius - base_radius * base_radius + total_height * total_height) / (2.0 * total_height);
  double R = std::sqrt(base_radius * base_radius + z0 * z0);

  // Compute radial distance at given fill height
  double dz = z0 - fill_height;
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

}  // namespace geometry
}  // namespace sodf
