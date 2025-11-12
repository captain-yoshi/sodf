#ifndef SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
#define SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_

#include <array>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sodf/physics/domain_shape_base.h>
#include <sodf/geometry/mesh.h>

namespace sodf {
namespace physics {

// -----------------------------------------------------------------------------
// Canonical convention (applies to all shapes here):
//   • Canonical X is the fill “height” axis.
//   • Y and Z span cross-sections (lateral directions).
//   • Dimensions are interpreted in the canonical frame as:
//       - Box:              X = height, Y/Z = lateral extents
//       - Cylinder:         X = height, radius in YZ
//       - Cone / Frustum:   X = height, base/top radii in YZ
//       - SphericalSegment: X = height, end radii in YZ
//   • Tilt-aware mesh uses a gravity frame where +X_g points “up”
//     (opposite to gravity), but **still** treats X_g as the canonical height axis.
//   • Some historical member names mention “z”; they now refer to the canonical
//     height axis (X / X_g). Inline accessors are provided for canonical names.
// -----------------------------------------------------------------------------

// ============================== Analytic shapes ==============================

class FluidBoxShape : public DomainShapeBase
{
public:
  // width  = extent along +Y, length = extent along +Z, max_fill_height = extent along +X
  FluidBoxShape(double width, double length, double max_fill_height);

  // V ↦ h  (height along +X)
  double getFillHeight(double V) const override;
  // h ↦ V  (height along +X)
  double getFillVolume(double h) const override;

  const double width_;   // along +Y
  const double length_;  // along +Z
};

class FluidCylinderShape : public DomainShapeBase
{
public:
  // radius in YZ, max_fill_height along +X
  FluidCylinderShape(double radius, double max_fill_height);

  double getFillHeight(double V) const override;  // V ↦ h (+X)
  double getFillVolume(double h) const override;  // h (+X) ↦ V

  const double radius_;  // in YZ
};

class FluidConeShape : public DomainShapeBase
{
public:
  // r0 at x=0 (base), r1 at x=H (top), H along +X
  FluidConeShape(double r0, double r1, double H);

  double getFillHeight(double V) const override;  // V ↦ h (+X)
  double getFillVolume(double h) const override;  // h (+X) ↦ V

  const double base_radius_;  // at x=0
  const double top_radius_;   // at x=H
};

class FluidSphericalSegmentShape : public DomainShapeBase
{
public:
  // Circular section radii in YZ at x=0 (base) and x=H (top)
  FluidSphericalSegmentShape(double r_base, double r_top, double H);

  double getFillHeight(double V) const override;  // V ↦ h (+X)
  double getFillVolume(double h) const override;  // h (+X) ↦ V

  const double base_radius_;
  const double top_radius_;
};

// ============================== Convex mesh (tilt-aware) ==============================
//
// Stateless, immutable convex mesh fluid domain.
// Canonical: in the gravity frame, +X_g is the height axis (up).
//
class FluidConvexMeshShape : public DomainShapeBase, public TiltAwareInterface
{
public:
  using Index = geometry::TriangleMesh::Index;
  using Tri = geometry::TriangleMesh::Face;

  // Construct from LOCAL mesh (authoring frame). In this class, LOCAL +X is
  // the canonical height axis for axis-aligned ops (no-gravity or trivial env).
  FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> triangles_local);

  // Axis-aligned fallbacks (treat local +X as “up”)
  double getFillHeight(double V) const override;  // V ↦ h (+X local)
  double getFillVolume(double h) const override;  // h (+X local) ↦ V

  // TiltAwareInterface (gravity-aware): +X_g is “up” in gravity frame
  double getFillVolumeWithEnv(double h_world, const FillEnv& env) const override;
  double getFillHeightWithEnv(double V, const FillEnv& env) const override;

  bool buildFilledVolumeAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  bool buildFilledVolumeAtVolumeWithEnv(double V, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  // Gravity-aligned working space (world → gravity)
  struct GravitySpace
  {
    // Rotation such that +X_g points “up” (opposite g_down_world)
    Eigen::Matrix3d R_wg;

    // NOTE (legacy field names): values below are measured ALONG THE HEIGHT AXIS (+X_g),
    // even though the names contain “z”. Use the canonical-X accessors for clarity.
    double base_z_g = 0.0;  // == base_x_g()
    double z_min = 0.0;     // == x_min_g()
    double z_max = 0.0;     // == x_max_g()

    // Canonical-X accessors (preferred)
    inline double base_x_g() const
    {
      return base_z_g;
    }
    inline double x_min_g() const
    {
      return z_min;
    }
    inline double x_max_g() const
    {
      return z_max;
    }

    // Height span above the base plane (along +X_g)
    inline double Hmax() const
    {
      return std::max(0.0, x_max_g() - base_x_g());
    }
  };

private:
  // ---- immutable caches & helpers -----------------------------------------

  void tetrahedralizeLocal_();
  void computeCapacityFromLocal_();

  // Build world→gravity rotation with +X_g aligned to “up”
  static Eigen::Matrix3d makeWorldToGravity_(const Eigen::Vector3d& g_down_world);
  GravitySpace makeGravitySpace_(const FillEnv& env) const;

  // Cross-section polygon at a given height (along +X_g)
  bool computeSectionPolygonAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& poly_world,
                                            const FillEnv& env) const;

private:
  // immutable mesh data (LOCAL authoring frame, canonical +X is height)
  std::vector<Eigen::Vector3d> verts_local_;
  std::vector<Tri> tris_;
  Eigen::Vector3d interior_local_{ Eigen::Vector3d::Zero() };

  struct Tet
  {
    Index a, b, c, d;
  };
  std::vector<Tet> tets_local_;
};

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
