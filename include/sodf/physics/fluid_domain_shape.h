
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

// Analytic fluid shapes (axis-aligned, +Z up in local)
class FluidBoxShape : public DomainShapeBase
{
public:
  FluidBoxShape(double width, double length, double max_fill_height);

  double getFillHeight(double V) const override;
  double getFillVolume(double h) const override;

  const double width_;
  const double length_;
};

class FluidCylinderShape : public DomainShapeBase
{
public:
  FluidCylinderShape(double radius, double max_fill_height);

  double getFillHeight(double V) const override;
  double getFillVolume(double h) const override;

  const double radius_;
};

class FluidConeShape : public DomainShapeBase
{
public:
  FluidConeShape(double r0, double r1, double H);

  double getFillHeight(double V) const override;
  double getFillVolume(double h) const override;

  const double base_radius_;
  const double top_radius_;
};

class FluidSphericalSegmentShape : public DomainShapeBase
{
public:
  FluidSphericalSegmentShape(double r_base, double r_top, double H);

  double getFillHeight(double V) const override;
  double getFillVolume(double h) const override;

  const double base_radius_;
  const double top_radius_;
};

// Stateless, immutable convex mesh fluid domain (tilt-aware via TiltAwareInterface)
class FluidConvexMeshShape : public DomainShapeBase, public TiltAwareInterface
{
public:
  using Index = geometry::TriangleMesh::Index;
  using Tri = geometry::TriangleMesh::Face;

  // Construct from LOCAL mesh (immutable).
  FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> triangles_local);

  // Axis-aligned fallbacks (treat local +Z as world +Z)
  double getFillHeight(double V) const override;
  double getFillVolume(double h) const override;

  // TiltAwareInterface stateless API
  double getFillVolumeWithEnv(double h_world, const FillEnv& env) const override;
  double getFillHeightWithEnv(double V, const FillEnv& env) const override;

  bool buildFilledVolumeAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  bool buildFilledVolumeAtVolumeWithEnv(double V, std::vector<Eigen::Vector3d>& tri_list_world,
                                        const FillEnv& env) const override;

  struct GravitySpace
  {
    Eigen::Matrix3d R_wg;   // world to gravity
    double base_z_g = 0.0;  // (R_wg * p_base_world).z()
    double z_min = 0.0;     // raw mesh min z (no base shift)
    double z_max = 0.0;     // raw mesh max z (no base shift)

    // height span above the base plane
    double Hmax() const
    {
      return std::max(0.0, z_max - base_z_g);
    }
  };

private:
  // immutable caches & helpers
  void tetrahedralizeLocal_();
  void computeCapacityFromLocal_();

  static Eigen::Matrix3d makeWorldToGravity_(const Eigen::Vector3d& g_down_world);
  GravitySpace makeGravitySpace_(const FillEnv& env) const;

  void orderConvexLoopXY(std::vector<Eigen::Vector3d>& pts) const;

  bool computeSectionPolygonAtHeightWithEnv(double h_world, std::vector<Eigen::Vector3d>& poly_world,
                                            const FillEnv& env) const;

  double clippedFanVolumeZleq_(const GravitySpace& gs, const FillEnv& env, double h_g) const;

private:
  // immutable mesh data (LOCAL)
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
