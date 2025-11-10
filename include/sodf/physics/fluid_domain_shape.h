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
// Canonical convention used by all fluid domain shapes in this file:
//
//   • X is the "height" (up) axis.
//   • For primitives, dimensions are interpreted as:
//       - Box:    x = height, y/z = lateral extents
//       - Cylinder: x = height, radius in (y,z)
//       - Cone/frustum: x = height, radii in (y,z)
//   • Tilt-aware mesh code builds a gravity frame where +X_g points "up".
//   • Some historical member names still mention 'z'; those refer to the
//     *height direction* (canonical X) for backward compatibility.
// -----------------------------------------------------------------------------

// Analytic fluid shapes (axis-aligned, +X up in local)
class FluidBoxShape : public DomainShapeBase
{
public:
  // width = Y extent, length = Z extent, max_fill_height = X extent
  FluidBoxShape(double width, double length, double max_fill_height);

  // V ↦ h (height along +X)
  double getFillHeight(double V) const override;
  // h (along +X) ↦ V
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
  // Radii of circular sections in YZ at x=0 (base) and x=H (top)
  FluidSphericalSegmentShape(double r_base, double r_top, double H);

  double getFillHeight(double V) const override;  // V ↦ h (+X)
  double getFillVolume(double h) const override;  // h (+X) ↦ V

  const double base_radius_;
  const double top_radius_;
};

// Stateless, immutable convex mesh fluid domain (tilt-aware via TiltAwareInterface)
// Canonical: +X_g is gravity "up" inside the gravity frame.
class FluidConvexMeshShape : public DomainShapeBase, public TiltAwareInterface
{
public:
  using Index = geometry::TriangleMesh::Index;
  using Tri = geometry::TriangleMesh::Face;

  // Construct from LOCAL mesh (immutable, authoring frame; X is canonical height).
  FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> triangles_local);

  // Axis-aligned fallbacks (treat local +X as world "up")
  double getFillHeight(double V) const override;  // V ↦ h (+X)
  double getFillVolume(double h) const override;  // h (+X) ↦ V

  // TiltAwareInterface stateless API (gravity-aware; +X_g is up)
  double getFillVolumeWithEnv(double h_world, const FillEnv& env) const override;
  double getFillHeightWithEnv(double V, const FillEnv& env) const override;

  bool buildFilledVolumeAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  bool buildFilledVolumeAtVolumeWithEnv(double V, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  struct GravitySpace
  {
    // World→Gravity rotation where +X_g points "up" (opposite g_down_world)
    Eigen::Matrix3d R_wg;

    // NOTE: historical names: the values below are measured ALONG THE
    // GRAVITY HEIGHT AXIS (canonical X_g), even though the member names say 'z'.
    double base_z_g = 0.0;  // (R_wg * p_base_world).x()
    double z_min = 0.0;     // min along +X_g of mesh vertices
    double z_max = 0.0;     // max along +X_g of mesh vertices

    // Height span above the base plane (along +X_g)
    double Hmax() const
    {
      return std::max(0.0, z_max - base_z_g);
    }
  };

private:
  // immutable caches & helpers
  void tetrahedralizeLocal_();
  void computeCapacityFromLocal_();

  // Build world→gravity rotation with +X_g aligned to "up"
  static Eigen::Matrix3d makeWorldToGravity_(const Eigen::Vector3d& g_down_world);
  GravitySpace makeGravitySpace_(const FillEnv& env) const;

  // Orders a convex loop in the plane perpendicular to the height axis.
  // (Name kept for ABI stability; operates in the gravity Y–Z plane.)
  void orderConvexLoopXY(std::vector<Eigen::Vector3d>& pts) const;

  // Cross-section polygon at a given height (along +X_g)
  bool computeSectionPolygonAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& poly_world,
                                            const FillEnv& env) const;

  // (internal helper kept declared; implemented in .cpp)
  double clippedFanVolumeZleq_(const GravitySpace& gs, const FillEnv& env, double h_g) const;

private:
  // immutable mesh data (LOCAL authoring frame)
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
