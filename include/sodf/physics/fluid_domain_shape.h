#ifndef SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
#define SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_

#include <sodf/physics/domain_shape.h>
#include <Eigen/Geometry>

namespace sodf {
namespace physics {

// Forward declarations
class FluidBoxShape;
class FluidCylinderShape;
class FluidConeShape;
class FluidSphericalSegmentShape;

/**
 * @brief Computes the fill height (level) in a stacked fluid domain for a given fill volume.
 */
double getFillHeight(const FluidBoxShape& domain, double fill_volume);
double getFillHeight(const FluidCylinderShape& domain, double fill_volume);
double getFillHeight(const FluidConeShape& domain, double fill_volume);
double getFillHeight(const FluidSphericalSegmentShape& domain, double fill_volume);

double getFillVolume(const FluidBoxShape& domain, double fill_height);
double getFillVolume(const FluidCylinderShape& domain, double fill_height);
double getFillVolume(const FluidConeShape& domain, double fill_height);
double getFillVolume(const FluidSphericalSegmentShape& domain, double fill_height);

/**
 * @brief Rectangular Prism ("Box") fluid domain shape.
 *
 * Models a right rectangular prism (box) with given width, length, and height.
 * Provides closed-form solutions for fill height/volume relationships.
 */
class FluidBoxShape : public DomainShape
{
public:
  FluidBoxShape(double width, double length, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double width_;
  const double length_;
};

/**
 * @brief Cylinder fluid domain shape (right circular cylinder).
 *
 * Models a cylinder with constant radius and height. Implements analytic formulas for fill height/volume.
 */
class FluidCylinderShape : public DomainShape
{
public:
  FluidCylinderShape(double radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double radius_;
};

/**
 * @brief Fluid domain cone or frustum (truncated cone) shape.
 *
 * Represents the volume and geometry of a cone or frustum defined by a base radius,
 * a top radius, and a height. The shape can be a perfect cone (if either the base
 * or top radius is zero), or a frustum (if both radii are nonzero).
 *
 * The base is at z = 0 (with radius base_radius_) and the top is at z = max_fill_height (with radius top_radius_).
 * If base_radius_ < top_radius_, the cone is "upside-down" (widest at the top).
 * If base_radius_ > top_radius_, the cone tapers upwards (widest at the base).
 *
 * The class supports calculation of the fill height required to reach a specified volume,
 * as well as the volume up to a specified fill height.
 */
class FluidConeShape : public DomainShape
{
public:
  FluidConeShape(double base_radius, double top_radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double base_radius_;  // Radius at base (z = 0)
  const double top_radius_;   // Radius at top (z = max_fill_height)
};

/**
 * @brief Fluid domain spherical segment shape (portion of a sphere "cut" by two planes).
 *
 * Represents a 3D shape corresponding to a segment of a sphere,
 * defined by the base radius, top radius, and segment height.
 *
 * The segment is modeled such that:
 *   - The flat, circular base of the segment is at z = 0.
 *   - The "tip" (the furthest point from the base) is at z = max_fill_height.
 *   - The full sphere's center lies along the symmetry axis.
 *
 * The geometric convention is that the segment is "standing" with the base at the bottom (z = 0),
 * and the tip at z = max_fill_height.
 */
class FluidSphericalSegmentShape : public DomainShape
{
public:
  FluidSphericalSegmentShape(double base_radius, double top_radius, double max_fill_height);

  double getFillHeight(double fill_volume) const override;
  double getFillVolume(double fill_height) const override;

  const double base_radius_;  // radius at the base (z = 0)
  const double top_radius_;   // radius at the top (z = max_fill_height)
};

FluidSphericalSegmentShape FromBaseTop(double r1, double r2);
FluidSphericalSegmentShape FromBaseHeight(double r1, double h);
FluidSphericalSegmentShape FromBaseSphereRadius(double r1, double R);

/// Signed tetra volume (a,b,c,d).
double signedTetraVolume(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                         const Eigen::Vector3d& d);

/// Volume of a tetra clipped by plane z = h (keep z ≤ h), all in gravity space.
double clippedTetraVolumeZleq(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                              const Eigen::Vector3d& p3, double h, double epsilon = 1e-12);

/**
 * @brief Fluid domain for an arbitrary closed convex triangle mesh under gravity.
 *
 * - getFillVolume(h): exact volume below height h via tetra + half-space clipping (z ≤ h in gravity space)
 * - getFillHeight(V): robust monotone inversion via bisection
 *
 * "Gravity space": a frame where +Z is aligned with gravity. The chosen base-plane point is z=0.
 */
class FluidConvexMeshShape : public DomainShape
{
public:
  struct Tri
  {
    int a, b, c;
  };  // triangle indices (CCW/outward for a closed convex mesh)

  /**
   * @brief Construct from WORLD-space mesh with known gravity and base-plane point.
   * @param vertices_world Closed, convex, manifold mesh vertices in world space.
   * @param tris           Triangle index list (CCW outward).
   * @param gravity_world  Gravity direction in world coords (need not be unit; will be normalized).
   * @param base_plane_point_world A point that defines where h=0 lives (e.g., tray inner bottom).
   *
   * This ctor computes max_fill_height in the initializer list so the DomainShape base is born correct.
   */
  FluidConvexMeshShape(const std::vector<Eigen::Vector3d>& vertices_world, const std::vector<Tri>& tris,
                       const Eigen::Vector3d& gravity_world, const Eigen::Vector3d& base_plane_point_world);

  /**
   * @brief Construct from LOCAL mesh only. Call setState(...) later to set pose/gravity/base.
   */
  FluidConvexMeshShape(std::vector<Eigen::Vector3d> vertices_local, std::vector<Tri> tris_local);

  // --- DomainShape virtuals ---
  double getFillVolume(double fill_height) const override;
  double getFillHeight(double fill_volume) const override;

  // --- Runtime state updates (e.g., object orientation changes) ---
  void setWorldPose(const Eigen::Isometry3d& T_wo);
  void setGravityWorld(const Eigen::Vector3d& g_world);
  void setBasePointWorld(const Eigen::Vector3d& p_world);

  /// Convenience: set all at once (pose, gravity, base-point) then recompute.
  void setState(const Eigen::Isometry3d& T_wo, const Eigen::Vector3d& g_world, const Eigen::Vector3d& p_base_world);

  // Accessors (gravity-space, after preprocessing)
  const std::vector<Eigen::Vector3d>& verticesG() const
  {
    return verts_g_;
  }
  const std::vector<Tri>& triangles() const
  {
    return tris_;
  }
  const Eigen::Matrix3d& R_world_to_g() const
  {
    return R_world_to_g_;
  }

  // Compute the free-surface (single convex polygon) at given *height* (meters).
  // Returns false if the plane does not intersect the mesh.
  bool computeSectionPolygonAtHeight(double height_g, std::vector<Eigen::Vector3d>& out_world) const;

  // Convenience: same, but from *volume*. Internally uses getFillHeight(volume).
  bool computeSectionPolygonAtVolume(double fill_volume, std::vector<Eigen::Vector3d>& out_world) const;

  // Build a watertight triangle list for the liquid volume at height h (gravity space).
  // Output: world-space triangles in triplets of points. Returns false if empty.
  bool buildFilledVolumeAtHeight(double height_g, std::vector<Eigen::Vector3d>& tri_list_world) const;

  // Convenience: same from volume
  bool buildFilledVolumeAtVolume(double fill_volume, std::vector<Eigen::Vector3d>& tri_list_world) const;

private:
  static Eigen::Matrix3d makeWorldToGravity(const Eigen::Vector3d& g_world);

  /// Compute max fill height from world-space inputs (used in initializer list).
  static double computeMaxFillHeightAt(const std::vector<Eigen::Vector3d>& verts_w, const Eigen::Vector3d& g_world,
                                       const Eigen::Vector3d& base_world);

  /// Build tetrahedra by fanning each triangle with an interior point (LOCAL space indices).
  void tetrahedralizeLocal_();

  /// Compute polyhedron volume from local-space tetra fan (rigid-transform invariant).
  void computeCapacityFromLocal_();

  /// Recompute gravity-space data (R, verts_g_, z extents, max_fill_height_) from current state.
  void updateGravitySpace_();

  // Clip a convex polygon by z <= h in gravity space (Sutherland–Hodgman).
  static void clipPolyZleq(double h, const std::vector<Eigen::Vector3d>& in, std::vector<Eigen::Vector3d>& out);

  // Fan-triangulate a convex polygon (gravity space) into world-space triangles and append them.
  void appendTriangulatedPolyToWorld(const std::vector<Eigen::Vector3d>& poly_g,
                                     std::vector<Eigen::Vector3d>& tri_list_world, bool reverse_winding = false) const;

  // Collect unique intersection points of the convex mesh with plane z = h (gravity space).
  void intersectWithPlaneZ(double h, std::vector<Eigen::Vector3d>& out_pts_g) const;

  // Order convex points (in gravity space) CCW to form a proper polygon.
  static void orderConvexLoopXY(std::vector<Eigen::Vector3d>& pts);

private:
  // ---------- Persistent (LOCAL) mesh data ----------
  std::vector<Eigen::Vector3d> verts_local_;
  std::vector<Tri> tris_;
  Eigen::Vector3d interior_local_{ Eigen::Vector3d::Zero() };  // centroid
  struct Tet
  {
    int a, b, c, d;
  };                             // indices into verts_local_ plus interior as last
  std::vector<Tet> tets_local_;  // built in local space (indices consistent forever)

  // ---------- Runtime state (WORLD) ----------
  Eigen::Isometry3d T_wo_{ Eigen::Isometry3d::Identity() };
  Eigen::Vector3d g_world_{ 0, 0, 1 };
  Eigen::Vector3d p_base_world_{ 0, 0, 0 };

  // ---------- Cached gravity-space data ----------
  Eigen::Matrix3d R_world_to_g_{ Eigen::Matrix3d::Identity() };
  std::vector<Eigen::Vector3d> verts_g_;  // gravity-space verts, includes interior as last
  double z_min_ = 0.0;
  double z_max_ = 0.0;

  // Numeric guard
  static constexpr double kEps = 1e-12;
};

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_FLUID_DOMAIN_SHAPE_H_
