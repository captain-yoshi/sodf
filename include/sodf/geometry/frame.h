#ifndef SODF_GEOMETRY_FRAME_H_
#define SODF_GEOMETRY_FRAME_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sodf/geometry/shape.h>

namespace sodf {
namespace geometry {

// ---- Core helpers -----------------------------------------------------------

double toRadians(double degrees);
double toDegrees(double radians);

/*
yaw:
    A yaw is a counterclockwise rotation of alpha about the  z-axis. The rotation matrix is given by

    R_z

    |cos(alpha) -sin(alpha) 0|
    |sin(apha)   cos(alpha) 0|
    |    0            0     1|

pitch:
    R_y
    A pitch is a counterclockwise rotation of  beta about the  y-axis. The rotation matrix is given by

    |cos(beta)  0   sin(beta)|
    |0          1       0    |
    |-sin(beta) 0   cos(beta)|

roll:
    A roll is a counterclockwise rotation of  gamma about the  x-axis. The rotation matrix is given by
    R_x
    |1          0           0|
    |0 cos(gamma) -sin(gamma)|
    |0 sin(gamma)  cos(gamma)|



    It is important to note that   R_z R_y R_x performs the roll first, then the pitch, and finally the yaw
    Roration matrix: R_z*R_y*R_x
*/
// https://ros-developer.com/2017/11/18/roll-pitch-yaw-using-eigen-kdl-frame/
Eigen::Isometry3d RPY(double roll, double pitch, double yaw);

/**
 * @brief Pick any unit vector perpendicular to 'axis' (stable across alignments).
 * @param axis Arbitrary 3D vector; can be non-unit. If degenerate, UnitX is returned.
 * @return Unit vector orthogonal to 'axis'.
 */
Eigen::Vector3d computeOrthogonalAxis(const Eigen::Vector3d& axis);

/**
 * @brief Orthonormalize (Gram–Schmidt) X,Y,Z and enforce right-handedness.
 *        If inputs are degenerate, robust fallbacks are used.
 * @return Rotation matrix with columns X,Y,Z and det=+1.
 */
Eigen::Matrix3d makeOrthonormalRightHanded(const Eigen::Vector3d& Xraw, const Eigen::Vector3d& Yraw,
                                           const Eigen::Vector3d& Zraw);

// ---- Public API -------------------------------------------------------------

/**
 * @brief Build a rotation from 3 (possibly non-orthonormal) axes.
 *        Columns map local X,Y,Z -> world.
 */
Eigen::Matrix3d computeOrientationFromAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                                           const Eigen::Vector3d& z_axis);

/**
 * @brief Build a rotation given Z (primary) and an optional X seed.
 *        If x_seed is degenerate or nearly parallel to z, a stable orthogonal is chosen.
 */
Eigen::Matrix3d computeOrientationFromZ(const Eigen::Vector3d& z_axis,
                                        const Eigen::Vector3d& x_seed = Eigen::Vector3d::Zero());

bool decompose_abs_twist_lift_wrt_axes(const Eigen::Isometry3d& T_abs, const Eigen::Vector3d& z_hat_in,
                                       const Eigen::Vector3d& x_hint_in, double& yaw_out, double& z_base_out,
                                       double tilt_tol, double xy_tol);

/**
 * @brief Isometry with +Z aligned to 'axis' and translation 'pos'.
 */
Eigen::Isometry3d buildIsometryFromZAxis(const Eigen::Vector3d& pos, const Eigen::Vector3d& axis);

/**
 * @brief Isometry with +Z aligned to z_axis and X seeded by x_axis, at translation 'pos'.
 */
Eigen::Isometry3d buildIsometryFromZXAxes(const Eigen::Vector3d& pos, const Eigen::Vector3d& z_axis,
                                          const Eigen::Vector3d& x_axis);

/**
 * @brief Rotation that maps a "shape frame" (z_shape, x_shape) to a "stack frame" (z_stack, x_stack).
 *        Non-orthonormal inputs are orthonormalized consistently first.
 */
Eigen::Matrix3d buildAxisAlignment(const Eigen::Vector3d& z_shape, const Eigen::Vector3d& x_shape,
                                   const Eigen::Vector3d& z_stack, const Eigen::Vector3d& x_stack);

// Produces a right-handed, orthonormal basis (X,Y,Z) from s.axes following the above convention.
// Uses getShapePrimaryAxis / getShapeUAxis / getShapeVAxis with stable fallbacks;
// `tol` is a small threshold to detect/repair degeneracies.
void buildCanonicalAxes(const sodf::geometry::Shape& s, Eigen::Vector3d& X, Eigen::Vector3d& Y, Eigen::Vector3d& Z,
                        double tol = 1e-9);

// Returns the canonical in-plane basis (U,V) for 2D shapes, i.e., (Y,Z) under the convention above.
// Special-cases Line as (direction, any stable orthogonal). Throws for non-2D shapes.
std::pair<Eigen::Vector3d, Eigen::Vector3d> pickCanonicalUV(const sodf::geometry::Shape& s);

// Map (u,v,0) from a 2D local parameterization to world, given explicit in-plane U and V directions.
// U and V need not be orthonormal; when orthonormalize=true we build a right-handed frame (U, V, N=U×V).
std::vector<Eigen::Vector3d> local2DVerticesToWorld(...);

std::vector<Eigen::Vector3d> local2DVerticesToWorld(const std::vector<Eigen::Vector3d>& uv_vertices,
                                                    const Eigen::Vector3d& U_axis, const Eigen::Vector3d& V_axis,
                                                    const Eigen::Isometry3d& pose,
                                                    const Eigen::Vector2d& uv_scale = Eigen::Vector2d(1.0, 1.0),
                                                    bool orthonormalize = true);

// optional 2D origin policy (AABBCenter, BaseCenter, VolumeCentroid, Native).
// Requires: <sodf/geometry/origin.h> in the .cpp for applyOriginPolicy2DVertices.
std::vector<Eigen::Vector3d> shape2DVerticesToWorld(const sodf::geometry::Shape& s, const Eigen::Isometry3d& pose,
                                                    bool apply_policy = true, bool orthonormalize = true);

// ---- Validation utilities ---------------------------------------------------

bool isUnitVector(const Eigen::Vector3d& v, double tol = 1e-8);

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-8);

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                           double tol = 1e-8);

bool isRightHanded(const Eigen::Vector3d& X, const Eigen::Vector3d& Y, const Eigen::Vector3d& Z, double tol = 1e-8);

bool isRightHandedOrthonormal(const Eigen::Vector3d& X, const Eigen::Vector3d& Y, const Eigen::Vector3d& Z,
                              double tol = 1e-8);

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_FRAME_H_
