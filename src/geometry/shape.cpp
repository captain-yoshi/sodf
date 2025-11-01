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
  const auto& type = shape.type;
  const auto& vertices = shape.vertices;
  const auto& dimensions = shape.dimensions;
  const auto& axes = shape.axes;

  switch (type)
  {
    case ShapeType::Line:
      if (vertices.size() < 2)
        throw std::runtime_error("Line shape requires two vertices for centroid calculation.");
      return 0.5 * (vertices.at(0) + vertices.at(1));

    case ShapeType::Triangle:
    case ShapeType::Polygon:
    {
      if (vertices.empty())
        throw std::runtime_error("Polygon/Triangle shape requires at least one vertex for centroid calculation.");
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (const auto& v : vertices) c += v;
      return c / vertices.size();
    }

    case ShapeType::TriangularPrism:
    {
      // dims = { base, altitude, height, apex_offset }, axes = [Altitude (X), Base (Y), Height (Z)]
      if (dimensions.size() < 3 || axes.size() < 3)
        throw std::runtime_error("TriangularPrism requires dims{base,altitude,height[,u]} and 3 axes.");
      const double bw = dimensions[0];   // base length along Base(Y)
      const double bh = dimensions[1];   // altitude along Altitude(X)
      const double h  = dimensions[2];   // prism height along Height(Z)
      const double u  = (dimensions.size() >= 4 ? dimensions[3] : 0.0); // apex offset along Altitude(X)

      const Eigen::Vector3d& Alt = axes[0]; // X
      const Eigen::Vector3d& Bas = axes[1]; // Y
      const Eigen::Vector3d& Hei = axes[2]; // Z

      // Base triangle centroid in base plane: A=(0,0), B=(bw,0) along Bas, C=(u,bh) (Alt,Bas)
      Eigen::Vector2d c2((0.0 + bw + 0.0) / 3.0, (0.0 + 0.0 + 0.0) / 3.0); // (we’ll build explicitly below)
      // Build centroid vector in the base plane:
      Eigen::Vector3d base_centroid = ( (bw / 3.0) * Bas.normalized()
                                      + (u  / 3.0) * Alt.normalized()
                                      + (bh / 3.0) * Alt.normalized() /* Alt component of C */ )
                                      - (u / 3.0) * Alt.normalized(); // cancel earlier placeholder
      // More directly: centroid = (A + B + C)/3 = ( (0*Alt+0*Bas) + (0*Alt+bw*Bas) + (u*Alt+bh*Bas) )/3
      base_centroid = ( (bw * Bas) + (u * Alt) + (bh * Bas) ) / 3.0;

      // Add half of prism height along Z:
      return base_centroid + 0.5 * h * Hei.normalized();
    }

    case ShapeType::Rectangle:
    case ShapeType::Circle:
    case ShapeType::Plane:
    case ShapeType::Box:
    case ShapeType::Sphere:
      return Eigen::Vector3d::Zero();

    case ShapeType::Cylinder:
    {
      // axes = [RefX, RefY, Symmetry(Z)], dims = {radius, height}
      if (axes.size() < 3 || dimensions.size() < 2)
        throw std::runtime_error("Cylinder requires axes[2]=symmetry and dims {radius,height}.");
      const Eigen::Vector3d& Sym = axes[2];
      return 0.5 * dimensions[1] * Sym.normalized();
    }

    case ShapeType::Cone:
    {
      // axes = [RefX, RefY, Symmetry(Z)], dims = {base_radius, top_radius, height}
      if (axes.size() < 3 || dimensions.size() < 3)
        throw std::runtime_error("Cone requires axes[2]=symmetry and dims {base_radius,top_radius,height}.");
      const Eigen::Vector3d& Sym = axes[2];
      // Centroid of a (possibly frustum) along axis ~ h/4 from base for a cone; keep your previous heuristic:
      return 0.25 * dimensions[2] * Sym.normalized();
    }

    case ShapeType::SphericalSegment:
    {
      // axes = [RefX, RefY, Symmetry(Z)], dims = {base_radius, top_radius, height}
      if (axes.size() < 3 || dimensions.size() < 3)
        throw std::runtime_error("SphericalSegment requires axes[2]=symmetry and dims {base_radius,top_radius,height}.");
      const Eigen::Vector3d& Sym = axes[2];
      double R = dimensions[0] > 0 ? dimensions[0] : dimensions[1];
      double h = dimensions[2];
      double z = (3.0 * R - h) * h * h / (4.0 * (3.0 * R * R - 3.0 * R * h + h * h)); // as you had
      return z * Sym.normalized();
    }

    case ShapeType::Mesh:
      throw std::runtime_error("getShapeCentroid() not implemented for Mesh.");

    default:
      throw std::runtime_error("getShapeCentroid() not implemented for this ShapeType.");
  }
}


// --- Helper: extract height of shape for stacking ---
double getShapeHeight(const Shape& shape)
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

const Eigen::Vector3d& getShapeNormalAxis(const Shape& s)
{
  using ST = ShapeType;
  switch (s.type)
  {
    // 2D surfaces + plane: Normal is Z (axes[2])
    case ST::Rectangle:
    case ST::Circle:
    case ST::Triangle:
    case ST::Polygon:
    case ST::Plane:
      return s.axes.at(2);

    // Revolute shapes: symmetry axis is Z (axes[2])
    case ST::Cylinder:
    case ST::Cone:
    case ST::SphericalSegment:
      return s.axes.at(2);

    // Box: take +Height (Z) as the “normal” for a face-aligned notion
    case ST::Box:
      return s.axes.at(2);

    // No unique surface normal:
    case ST::Line:
    case ST::Sphere:
    case ST::Mesh:
      throw std::runtime_error("getShapeNormalAxis undefined for this ShapeType.");

    default:
      throw std::runtime_error("getShapeNormalAxis: unsupported ShapeType.");
  }
}

const Eigen::Vector3d& getShapeReferenceAxis(const geometry::Shape& shape)
{
  using ShapeType = geometry::ShapeType;
  switch (shape.type)
  {
    case ShapeType::TriangularPrism:
      if (shape.axes.size() < 2)
        throw std::runtime_error("TriangularPrism requires axes[1] (Base) for reference axis.");
      return shape.axes[1]; // Base (Y)

    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::SphericalSegment:
      if (shape.axes.size() < 2)
        throw std::runtime_error("Axisymmetric shape requires axes[1] (RefY) for reference axis.");
      return shape.axes[1]; // RefY

    default:
      throw std::runtime_error("getShapeReferenceAxis() only valid for shapes with a defined base plane.");
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
      if (shape.axes.size() < 3)
        throw std::runtime_error("Axisymmetric shape requires axes[2] (symmetry Z).");
      return shape.axes[2];

    default:
      throw std::runtime_error("getShapeSymmetryAxis() only valid for axisymmetric 3D shapes.");
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
