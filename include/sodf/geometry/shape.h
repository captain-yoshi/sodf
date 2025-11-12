#ifndef SODF_GEOMETRY_SHAPE_H_
#define SODF_GEOMETRY_SHAPE_H_

#include <array>
#include <vector>
#include <string>
#include <type_traits>
#include <iostream>

#include <Eigen/Geometry>

#include <sodf/geometry/mesh.h>

namespace sodf {
namespace geometry {

// Where to place the local origin (0,0,0) for shapes
enum class OriginPolicy
{
  Native = 0,      // Do not adjust; use vertices/URI frame as-authored.
                   // Cannot be used for primitives with dimensions.
  BaseCenter,      // Base-aligned (xE[0,H]), origin at base center (x=0,y_mid,z_mid).
  AABBCenter,      // Origin at full-shape AABB geometric center.
  VolumeCentroid,  // Origin at (volume/area) centroid (fallback: AABBCenter).

  // BaseMinCornerYZ,    // Base at x=0; origin at base AABB min corner (0,y_min,z_min).
  // BaseMaxCornerYZ,    // Base at x=0; origin at base AABB max corner (0,y_max,z_max).

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
| ShapeType         | Dimensions                                  | Axes (canonical X, Y, Z)   | Vertices           | Description / Notes                                      |
|-------------------|---------------------------------------------|----------------------------|--------------------|----------------------------------------------------------|
| Line              | length                                      | direction, N/A, N/A        | 2 points (2D/3D)   | Works for 2D or 3D. Z vertices = 0 for 2D.               |
|                   |                                             |                            |                    | Bounded if length > 0, infinite if = 0.                  |
| Rectangle         | size_y, size_z                              | normal, reference y-z      | 4 points           |                                                          |
| Circle            | radius                                      | normal, reference y-z      | -                  |                                                          |
| Triangle          | base_y, altitude_z, apex_offset_y           | normal, base, altitude     | 3 points           |                                                          |
| Polygon           | - (dimentionless)                           | normal, reference y-z      | N points           | Width and height are wrt. the centroid.                  |
| Plane             | size_y, size_z                              | normal, reference y-z      | -                  | Bounded plane if width & height > 0, infinite if = 0.    |
| Box               | x, y, z                                     | x, y, z                    | -                  |                                                          |
| TriangularPrism   | length_x, base_y, altitude_z, apex_offset_y | extrusion, base, altitude  | -                  | Base A=(0,0), B=(bw,0), C=(u,bh) in base plane.          |
| Cylinder          | height_x, radius                            | symmetry, reference y-z    | -                  |                                                          |
| Sphere            | radius                                      | (none)                     | -                  |                                                          |
| Cone              | height_x,base_radius, top_radius            | symmetry, reference y-z    | -                  | Frustum if top_radius > 0, cone if = 0.                  |
| SphericalSegment  | height_x,base_radius, top_radius            | symmetry, reference y-z    | -                  | Cap if one radius = 0, segment if both > 0.              |
| Mesh              | - (dimensionless)                           | x, y, z                    | External URI or    | Use as-authored local frame, or override with AxisX/Y/Z; |
|                   |                                             |                            | TriangleMesh (V,F) | Supports indexed triangles; units = meters               |
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

bool isPrimitive(geometry::ShapeType t);
bool isPrimitive2D(geometry::ShapeType t);
bool isPrimitive3D(geometry::ShapeType t);

bool isPrimitive(const geometry::Shape& s);
bool isPrimitive2D(const geometry::Shape& s);
bool isPrimitive3D(const geometry::Shape& s);

bool primitiveHasDimensions(const geometry::Shape& s);
bool primitiveHasVertices(const geometry::Shape& s);

bool is2DShape(const Shape& shape);

// Dimension-less by SPEC: shapes that are not defined via `dimensions[]`.
bool isDimensionless(geometry::ShapeType t);

// Dimension-less by INSTANCE
bool isDimensionless(const geometry::Shape& s);

bool hasExternalMesh(const Shape& s);
bool hasInlineMesh(const Shape& s);
const MeshRef* getExternalMesh(const Shape& s);
const TriangleMesh* getInlineMesh(const Shape& s);

// Already-declared API (left unchanged) --------------------------------------

// Returns the primary axis for the shape:
// - 2D: the surface normal (canonical axes[0])
// - 3D: the symmetry / extrusion axis (canonical axes[0])
// - Mesh: there is no canonical choice; if provided, returns axes[0],
//         otherwise returns a stable +X fallback.
//
// Contract: callers that rely on a specific direction SHOULD set axes[0].
// For Mesh, consider precomputing your own principal/extrusion axis (e.g. AABB
// longest axis or PCA) and storing it in axes[0] when constructing the Shape.
const Eigen::Vector3d& getShapePrimaryAxis(const Shape& shape);

// In-plane / reference axes (the "other" two):
// - For 2D: these are the two in-plane directions.
// - For 3D of revolution/extrusion: these are the two reference directions perpendicular to symmetry.
// - For Mesh: there is no canonical choice; if not provided we fall back to world Y/Z.
//
// Naming: we use "U" and "V" to avoid implying a specific semantic like Width/Height.
const Eigen::Vector3d& getShapeUAxis(const Shape& shape);
const Eigen::Vector3d& getShapeVAxis(const Shape& shape);

double getShapeHeight(const Shape& shape);
double getShapeBaseRadius(const geometry::Shape& shape);
double getShapeTopRadius(const geometry::Shape& shape);
double getShapeMaxRadius(const geometry::Shape& shape);

double getTopRadiusAtHeight(double base_radius, double top_radius, double total_height, double new_height);
geometry::Shape truncateShapeToHeight(const geometry::Shape& shape, double new_height);

bool isValidSegment(double r1, double r2, double h, double epsilon = 1e-12);

double inferSegmentHeightFromRadii(double r1, double r2);
double inferTopRadiusFromHeight(double r1, double h);
double inferBaseRadiusFromHeight(double r2, double h);

Eigen::Vector3d getShapeCentroid(const Shape& shape);
ShapeType shapeTypeFromString(const std::string& str);
std::string shapeTypeToString(ShapeType type);

// Helpers ---------------------------------------------------------------------

const char* originPolicyToString(OriginPolicy p);

// Canonical semantic labels for axes per shape type.
// Returns a span-like triplet of const char* for (X,Y,Z).
struct AxisLabels
{
  const char* X;
  const char* Y;
  const char* Z;
};

AxisLabels shapeAxisLabels(ShapeType t);

// Printers --------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, ShapeType type);
std::ostream& operator<<(std::ostream& os, const MeshRef& mref);
std::ostream& operator<<(std::ostream& os, const TriangleMesh& M);
std::ostream& operator<<(std::ostream& os, const Shape& shape);

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_SHAPE_H_
