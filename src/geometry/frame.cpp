#include <sodf/geometry/frame.h>
#include <sodf/geometry/origin.h>

#include <cmath>
#include <stdexcept>

namespace sodf {
namespace geometry {

double toRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double toDegrees(double radians)
{
  return radians * 180.0 / M_PI;
}

Eigen::Isometry3d RPY(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());

  return Eigen::Isometry3d(Eigen::Quaterniond(y * p * r));
}

Eigen::Vector3d computeOrthogonalAxis(const Eigen::Vector3d& axis)
{
  // Normalize input (handles zero safely below).
  Eigen::Vector3d a = axis;
  const double an = a.norm();
  if (!(an > 0.0) || !std::isfinite(an))
  {
    return Eigen::Vector3d::UnitX();  // degenerate input
  }
  a /= an;

  // Choose a fallback least aligned with 'axis' to improve conditioning.
  Eigen::Vector3d fallback = (std::abs(a.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();

  Eigen::Vector3d x = a.cross(fallback);
  double n = x.norm();
  if (n <= 1e-12 || !std::isfinite(n))
  {
    // axis ~ fallback; try another
    fallback = Eigen::Vector3d::UnitX();
    x = a.cross(fallback);
    n = x.norm();
  }

  if (n > 0.0 && std::isfinite(n))
    return x / n;

  // final fallback (only if axis was truly pathological)
  return Eigen::Vector3d::UnitX();
}

Eigen::Matrix3d makeOrthonormalRightHanded(const Eigen::Vector3d& Xraw, const Eigen::Vector3d& Yraw,
                                           const Eigen::Vector3d& Zraw)
{
  Eigen::Vector3d X = Xraw;
  Eigen::Vector3d Y = Yraw;
  Eigen::Vector3d Z = Zraw;

  // Normalize X (fallback if degenerate)
  double nx = X.norm();
  if (nx > 0.0 && std::isfinite(nx))
    X /= nx;
  else
    X = Eigen::Vector3d::UnitX();

  // Make Y orthogonal to X, then normalize (fallback if degenerate)
  Y -= X * (X.dot(Y));
  double ny = Y.norm();
  if (ny > 0.0 && std::isfinite(ny))
    Y /= ny;
  else
    Y = computeOrthogonalAxis(X);  // guaranteed unit + orthogonal to X

  // Make Z orthogonal to span{X,Y}, then normalize
  Z -= X * (X.dot(Z)) + Y * (Y.dot(Z));
  double nz = Z.norm();
  if (!(nz > 0.0) || !std::isfinite(nz))
    Z = X.cross(Y);  // exact orthogonal completion
  else
    Z /= nz;

  // Ensure right-handedness (determinant +1). If det < 0, flip Z.
  Eigen::Matrix3d R;
  R.col(0) = X;
  R.col(1) = Y;
  R.col(2) = Z;

  if (R.determinant() < 0.0)
    R.col(2) = -R.col(2);

  return R;
}

// ---- Public API -------------------------------------------------------------

Eigen::Matrix3d computeOrientationFromAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                                           const Eigen::Vector3d& z_axis)
{
  return makeOrthonormalRightHanded(x_axis, y_axis, z_axis);
}

Eigen::Matrix3d computeOrientationFromZ(const Eigen::Vector3d& z_axis, const Eigen::Vector3d& x_seed)
{
  constexpr double kTiny = 1e-12;
  constexpr double kAlmostParallel = 0.999;  // cos(theta) ≳ 0.999 ⇒ θ ≲ 2.6°

  auto safe_norm = [&](const Eigen::Vector3d& v) -> double {
    const double n = v.norm();
    return (std::isfinite(n) ? n : 0.0);
  };

  // --- Z: primary axis
  Eigen::Vector3d Z = (safe_norm(z_axis) > kTiny) ? z_axis.normalized() : Eigen::Vector3d::UnitZ();

  // --- X: project seed onto plane ⟂ Z; if degenerate, pick a stable orthogonal
  Eigen::Vector3d X = x_seed;
  if (safe_norm(X) <= kTiny || std::abs(X.normalized().dot(Z)) > kAlmostParallel)
  {
    X = computeOrthogonalAxis(Z);  // stable fallback from your frame.h
  }
  else
  {
    // Gram–Schmidt: remove Z component from X
    X -= Z * (X.dot(Z));
    const double nx = safe_norm(X);
    if (nx <= kTiny)
      X = computeOrthogonalAxis(Z);
    else
      X /= nx;
  }

  // --- Y: complete right-handed triad, then re-tighten X
  Eigen::Vector3d Y = Z.cross(X);
  const double ny = safe_norm(Y);
  if (ny <= kTiny)
  {
    // Extremely rare: numerical collapse — pick a new orthogonal X and recompute
    X = computeOrthogonalAxis(Z);
    Y = Z.cross(X);
  }
  Y.normalize();

  // Re-orthogonalize X to counter tiny drift from normalization
  X = Y.cross(Z);  // guaranteed ⟂ both Y and Z, right-handed

  // Final, right-handed, orthonormal basis in columns
  return makeOrthonormalRightHanded(X, Y, Z);
}

bool decompose_abs_twist_lift_wrt_axes(const Eigen::Isometry3d& T_abs, const Eigen::Vector3d& z_hat_in,
                                       const Eigen::Vector3d& x_hint_in, double& yaw_out, double& z_base_out,
                                       double tilt_tol, double xy_tol)
{
  const Eigen::Vector3d z_hat = z_hat_in.normalized();
  const Eigen::Matrix3d B = computeOrientationFromZ(z_hat, x_hint_in);  // cols: x_hat,y_hat,z_hat

  const Eigen::Matrix3d Rw = T_abs.linear();
  const Eigen::Vector3d tw = T_abs.translation();

  // Lateral shift must be ~0 wrt z_hat
  const double z_comp = z_hat.dot(tw);
  const Eigen::Vector3d t_perp = tw - z_hat * z_comp;
  if (t_perp.norm() > xy_tol)
  {
    // std::cout << "meshifyStackedShapePrimitives: lateral shift " << t_perp.norm() << " exceeds tolerance (" << xy_tol
    //           << ")." << std::endl;
    return false;
  }

  // No tilt of the stack axis
  if ((Rw * z_hat - z_hat).norm() > tilt_tol)
  {
    // std::cout << "meshifyStackedShapePrimitives: tilt of stack axis exceeds tolerance (" << tilt_tol << " rad)."
    //           << std::endl;
    return false;
  }

  // Yaw about z_hat
  const Eigen::Matrix3d Rloc = B.transpose() * Rw * B;
  yaw_out = std::atan2(Rloc(1, 0), Rloc(0, 0));
  z_base_out = z_comp;
  return true;
}

Eigen::Isometry3d buildIsometryFromZAxis(const Eigen::Vector3d& pos, const Eigen::Vector3d& axis)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = computeOrientationFromZ(axis);
  T.translation() = pos;
  return T;
}

Eigen::Isometry3d buildIsometryFromZXAxes(const Eigen::Vector3d& pos, const Eigen::Vector3d& z_axis,
                                          const Eigen::Vector3d& x_axis)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = computeOrientationFromZ(z_axis, x_axis);
  T.translation() = pos;
  return T;
}

Eigen::Matrix3d buildAxisAlignment(const Eigen::Vector3d& z_shape, const Eigen::Vector3d& x_shape,
                                   const Eigen::Vector3d& z_stack, const Eigen::Vector3d& x_stack)
{
  Eigen::Matrix3d R_shape = computeOrientationFromZ(z_shape, x_shape);
  Eigen::Matrix3d R_stack = computeOrientationFromZ(z_stack, x_stack);
  // Map: shape -> world via R_shape, world -> stack via R_stack^T
  // We want R such that v_stack = R * v_shape
  return R_stack * R_shape.transpose();
}

void buildCanonicalAxes(const sodf::geometry::Shape& s, Eigen::Vector3d& X, Eigen::Vector3d& Y, Eigen::Vector3d& Z,
                        double tol)
{
  using Eigen::Vector3d;
  using ST = sodf::geometry::ShapeType;

  auto ensure_in_plane = [&](const Vector3d& v, const Vector3d& x_primary) -> Vector3d {
    // project v to plane ⟂ X (primary). If degenerate, pick a stable orthogonal to X.
    Vector3d Xn = (x_primary.squaredNorm() > 0.0) ? x_primary.normalized() : Vector3d::UnitX();
    Vector3d p = v - (v.dot(Xn)) * Xn;
    if (p.squaredNorm() < tol * tol)
      p = sodf::geometry::computeOrthogonalAxis(Xn);
    return p;
  };

  Vector3d Xraw, Yraw, Zraw;

  switch (s.type)
  {
    // 2D surfaces + plane + circle:
    // Canonical: X = Normal, Y/Z = in-plane references
    case ST::Rectangle:
    case ST::Triangle:
    case ST::Polygon:
    case ST::Plane:
    case ST::Circle:
    {
      Xraw = getShapePrimaryAxis(s);                   // Normal → canonical X
      Yraw = ensure_in_plane(getShapeUAxis(s), Xraw);  // in-plane ref → canonical Y
      Zraw = ensure_in_plane(getShapeVAxis(s), Xraw);  // in-plane ref → canonical Z
      Eigen::Matrix3d R = makeOrthonormalRightHanded(Xraw, Yraw, Zraw);
      X = R.col(0);
      Y = R.col(1);
      Z = R.col(2);
      return;
    }

    // Box / TriangularPrism: canonical axes are (X,Y,Z) as stored
    case ST::Box:
    case ST::TriangularPrism:
    {
      Xraw = getShapePrimaryAxis(s);  // Box: X; Prism: Extrusion
      Yraw = getShapeUAxis(s);        // Box: Y; Prism: Base
      Zraw = getShapeVAxis(s);        // Box: Z; Prism: Altitude
      Eigen::Matrix3d R = makeOrthonormalRightHanded(Xraw, Yraw, Zraw);
      X = R.col(0);
      Y = R.col(1);
      Z = R.col(2);
      return;
    }

    // Surfaces of revolution: X = Symmetry, Y/Z = base-plane references
    case ST::Cylinder:
    case ST::Cone:
    case ST::SphericalSegment:
    {
      Xraw = getShapePrimaryAxis(s);                   // Symmetry → canonical X
      Yraw = ensure_in_plane(getShapeUAxis(s), Xraw);  // base-plane ref → Y
      Zraw = ensure_in_plane(getShapeVAxis(s), Xraw);  // base-plane ref → Z
      Eigen::Matrix3d R = makeOrthonormalRightHanded(Xraw, Yraw, Zraw);
      X = R.col(0);
      Y = R.col(1);
      Z = R.col(2);
      return;
    }

    // Mesh: use as-authored frame (X,Y,Z)
    case ST::Mesh:
    {
      Xraw = getShapePrimaryAxis(s);
      Yraw = getShapeUAxis(s);
      Zraw = getShapeVAxis(s);
      Eigen::Matrix3d R = makeOrthonormalRightHanded(Xraw, Yraw, Zraw);
      X = R.col(0);
      Y = R.col(1);
      Z = R.col(2);
      return;
    }

    // Line / Sphere: provide a benign default
    case ST::Line:
    case ST::Sphere:
    default:
    {
      X = Vector3d::UnitX();
      Y = Vector3d::UnitY();
      Z = Vector3d::UnitZ();
      return;
    }
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> pickCanonicalUV(const sodf::geometry::Shape& s)
{
  using ST = sodf::geometry::ShapeType;

  switch (s.type)
  {
    // 2D: axes = [X=Normal, Y=RefU, Z=RefV] → U=Y, V=Z
    case ST::Rectangle:
    case ST::Triangle:
    case ST::Polygon:
    case ST::Circle:
    case ST::Plane:
      return { getShapeUAxis(s), getShapeVAxis(s) };

    case ST::Line:
    {
      const Eigen::Vector3d U = (s.axes.empty() ? Eigen::Vector3d::UnitX() : s.axes.at(0));  // direction
      const Eigen::Vector3d V = sodf::geometry::computeOrthogonalAxis(U);
      return { U, V };
    }

    default:
      throw std::runtime_error("pickCanonicalUV: unsupported ShapeType for 2D mapping");
  }
}

static inline Eigen::Vector3d unit_or(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback)
{
  double n = v.norm();
  if (n > 0.0 && std::isfinite(n))
    return v / n;
  return fallback;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> deduceUVFromNormal(const Eigen::Vector3d& normal,
                                                               const Eigen::Vector3d& u_seed)
{
  using Eigen::Vector3d;

  // Z := plane normal (unit with fallback)
  Vector3d Z = unit_or(normal, Vector3d::UnitZ());

  // U := project seed onto plane ⟂ Z; if unusable, pick a stable orthogonal
  Vector3d U = u_seed;
  bool ok_seed = (U.norm() > 0.0) && std::isfinite(U.norm());
  if (ok_seed)
  {
    U -= Z * (U.dot(Z));
    if (!(U.norm() > 0.0) || !std::isfinite(U.norm()))
      ok_seed = false;
  }
  if (!ok_seed)
    U = computeOrthogonalAxis(Z);  // unit & ⟂ Z
  else
    U.normalize();

  // V completes right-handed triad; re-tighten U
  Vector3d V = (Z.cross(U)).normalized();
  U = V.cross(Z);  // now (U,V,Z) is orthonormal & right-handed
  return { U, V };
}

std::vector<Eigen::Vector3d> local2DVerticesToWorldFromNormal(const std::vector<Eigen::Vector3d>& uv_vertices,
                                                              const Eigen::Vector3d& normal,
                                                              const Eigen::Vector3d& u_seed,
                                                              const Eigen::Isometry3d& pose,
                                                              const Eigen::Vector2d& uv_scale, bool orthonormalize)
{
  using Eigen::Vector3d;

  // 1) Deduce in-plane U,V from the normal (+ optional U seed)
  auto [Uraw, Vraw] = deduceUVFromNormal(normal, u_seed);

  Vector3d U = Uraw, V = Vraw;
  if (!orthonormalize)
  {
    // If caller really wants raw directions, only ensure they are finite & non-zero
    if (!(U.norm() > 0.0) || !std::isfinite(U.norm()))
      U = Vector3d::UnitX();
    if (!(V.norm() > 0.0) || !std::isfinite(V.norm()))
      V = computeOrthogonalAxis(U);
  }

  // 2) World-space scaled basis
  const Vector3d Uw = pose.linear() * (uv_scale.x() * U);  // U = “u” direction
  const Vector3d Vw = pose.linear() * (uv_scale.y() * V);  // V = “v” direction
  const Vector3d Ow = pose.translation();

  // 3) Map (0,u,v) → O + u*Uw + v*Vw   (your convention: vertex.y==u, vertex.z==v)
  std::vector<Vector3d> out;
  out.reserve(uv_vertices.size());
  for (const Vector3d& uv0 : uv_vertices)
  {
    const double u = uv0.y();
    const double v = uv0.z();
    out.emplace_back(Ow + u * Uw + v * Vw);
  }
  return out;
}

std::vector<Eigen::Vector3d> shape2DVerticesToWorldFromNormal(const sodf::geometry::Shape& s,
                                                              const Eigen::Isometry3d& pose,
                                                              const Eigen::Vector3d& normal,
                                                              const Eigen::Vector3d& u_seed, bool apply_policy,
                                                              bool orthonormalize)
{
  // Start from authored (0,u,v) vertices and apply 2D origin policy if requested
  std::vector<Eigen::Vector3d> uv = s.vertices;
  if (apply_policy)
    applyOriginPolicy2DVertices(s, uv);

  // Scale in plane uses Y,Z components of per-instance scale (since (0,u,v))
  const Eigen::Vector2d scale2(s.scale.y(), s.scale.z());
  return local2DVerticesToWorldFromNormal(uv, normal, u_seed, pose, scale2, orthonormalize);
}

// ---- Validation utilities ---------------------------------------------------

bool isUnitVector(const Eigen::Vector3d& v, double tol)
{
  return std::abs(v.norm() - 1.0) <= tol;
}

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol)
{
  return isUnitVector(a, tol) && isUnitVector(b, tol) && std::abs(a.dot(b)) <= tol;
}

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double tol)
{
  return isUnitVector(a, tol) && isUnitVector(b, tol) && isUnitVector(c, tol) && std::abs(a.dot(b)) <= tol &&
         std::abs(a.dot(c)) <= tol && std::abs(b.dot(c)) <= tol;
}

bool isRightHanded(const Eigen::Vector3d& X, const Eigen::Vector3d& Y, const Eigen::Vector3d& Z, double tol)
{
  // det of [X Y Z] equals scalar triple product X · (Y × Z)
  const double detR = X.dot(Y.cross(Z));
  const double handed = (X.cross(Y)).dot(Z);
  return (detR > 1.0 - tol) && (handed > 1.0 - tol);
}

bool isRightHandedOrthonormal(const Eigen::Vector3d& X, const Eigen::Vector3d& Y, const Eigen::Vector3d& Z, double tol)
{
  return areVectorsOrthonormal(X, Y, Z, tol) && isRightHanded(X, Y, Z, tol);
}

}  // namespace geometry
}  // namespace sodf
