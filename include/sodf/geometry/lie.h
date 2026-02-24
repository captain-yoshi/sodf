#ifndef SODF_GEOMETRY_LIE_H_
#define SODF_GEOMETRY_LIE_H_

#include <algorithm>  // std::clamp
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sodf {
namespace geometry {

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Unsigned angle between arbitrary 3D vectors (stable near 0 and pi)
inline double angle_between(const Vector3d& a, const Vector3d& b)
{
  const double na = a.norm(), nb = b.norm();
  if (na < 1e-15 || nb < 1e-15)
    return 0.0;
  const double dot = a.dot(b) / (na * nb);
  const double c = std::clamp(dot, -1.0, 1.0);
  const double s = (a.cross(b)).norm() / (na * nb);  // == sin(theta)
  return std::atan2(s, c);
}

// Faster when inputs are already unit-length
inline double angle_between_unit(const Vector3d& ua, const Vector3d& ub)
{
  const double c = std::clamp(ua.dot(ub), -1.0, 1.0);
  const double s = ua.cross(ub).norm();
  return std::atan2(s, c);
}

// Signed angle from a to b around reference axis n (n should be unit)
inline double signed_angle_between(const Vector3d& a, const Vector3d& b, const Vector3d& n_unit)
{
  const double na = a.norm(), nb = b.norm();
  if (na < 1e-15 || nb < 1e-15)
    return 0.0;
  const Vector3d ua = a / na, ub = b / nb;
  const double c = std::clamp(ua.dot(ub), -1.0, 1.0);
  const Vector3d cross = ua.cross(ub);
  const double s = cross.norm();
  double ang = std::atan2(s, c);
  // sign from orientation relative to n
  if (cross.dot(n_unit) < 0.0)
    ang = -ang;
  return ang;
}

// Generic so(3) "hat" (skew) with any Eigen 3-vector type / scalar
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew3(const Eigen::MatrixBase<Derived>& a)
{
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1, "skew3 expects a 3x1 vector");
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix<Scalar, 3, 3> S;
  S << Scalar(0), -a.z(), a.y(), a.z(), Scalar(0), -a.x(), -a.y(), a.x(), Scalar(0);
  return S;
}

// Double-dispatch convenience for Vector3d (keeps your original name usable)
inline Eigen::Matrix3d skew(const Eigen::Vector3d& a)
{
  return skew3(a);
}

// Vee operator (so(3) -> R^3): inverse of hat
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> vee3(const Eigen::MatrixBase<Derived>& S)
{
  static_assert(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3, "vee3 expects a 3x3 matrix");
  using Scalar = typename Derived::Scalar;
  // Take the average of the two off-diagonals to be robust to small numeric asymmetries
  const Scalar wx = Scalar(0.5) * (S(2, 1) - S(1, 2));
  const Scalar wy = Scalar(0.5) * (S(0, 2) - S(2, 0));
  const Scalar wz = Scalar(0.5) * (S(1, 0) - S(0, 1));
  return Eigen::Matrix<Scalar, 3, 1>(wx, wy, wz);
}

// SE(3) wedge / vee helpers (useful for Jacobians & adjoints)
// xi = [w; v] in R^6  ->  se(3) 4x4 matrix
template <class Scalar>
inline Eigen::Matrix<Scalar, 4, 4> wedge6(const Eigen::Matrix<Scalar, 6, 1>& xi)
{
  const Eigen::Matrix<Scalar, 3, 1> w = xi.template head<3>();
  const Eigen::Matrix<Scalar, 3, 1> v = xi.template tail<3>();
  Eigen::Matrix<Scalar, 4, 4> X = Eigen::Matrix<Scalar, 4, 4>::Zero();
  X.template block<3, 3>(0, 0) = skew3(w);
  X.template block<3, 1>(0, 3) = v;
  return X;
}

// se(3) 4x4 -> xi (R^6)
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 1> vee6(const Eigen::MatrixBase<Derived>& X)
{
  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4, "vee6 expects a 4x4 matrix");
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix<Scalar, 6, 1> xi;
  xi.template head<3>() = vee3(X.template block<3, 3>(0, 0));
  xi.template tail<3>() = X.template block<3, 1>(0, 3);
  return xi;
}

// Adjoint of SE(3): maps twists between frames (Ad_T * xi)
// T = [ R p; 0 1 ]
template <class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 6, 6>
adjointSE3(const Eigen::Transform<typename Derived::Scalar, 3, Eigen::Isometry>& T)
{
  using Scalar = typename Derived::Scalar;
  const Eigen::Matrix<Scalar, 3, 3> R = T.linear();
  const Eigen::Matrix<Scalar, 3, 1> p = T.translation();
  Eigen::Matrix<Scalar, 6, 6> Ad;
  Ad.setZero();
  Ad.template block<3, 3>(0, 0) = R;
  Ad.template block<3, 3>(0, 3) = skew3(p) * R;  // p^ * R
  Ad.template block<3, 3>(3, 3) = R;
  return Ad;
}

// Orthogonal projector onto the plane perpendicular to unit vector n
inline Matrix3d proj_perp(const Vector3d& n_unit)
{
  return Matrix3d::Identity() - n_unit * n_unit.transpose();
}

// Rodrigues exponential: SO(3) <- so(3)
inline Matrix3d expSO3(const Vector3d& w)
{
  const double th = w.norm();
  if (th < 1e-12)
    return Matrix3d::Identity();

  const Vector3d a = w / th;  // unit axis
  const Matrix3d A = skew(a);
  return Matrix3d::Identity() + std::sin(th) * A + (1.0 - std::cos(th)) * (A * A);
}

// SE(3) exponential using the left Jacobian V for translation
// xi = [w; v] with w in so(3), v in R^3
inline Isometry3d expSE3(const Eigen::Matrix<double, 6, 1>& xi)
{
  const Vector3d w = xi.head<3>();
  const Vector3d v = xi.tail<3>();

  Isometry3d T = Isometry3d::Identity();

  const double th = w.norm();
  const Matrix3d R = expSO3(w);
  Matrix3d V = Matrix3d::Identity();

  if (th > 1e-12)
  {
    const Matrix3d W = skew(w);
    const double s = std::sin(th), c = std::cos(th);
    V = Matrix3d::Identity() + (1.0 - c) / (th * th) * W + (th - s) / (th * th * th) * (W * W);
  }

  T.linear() = R;
  T.translation() = V * v;
  return T;
}

// SO(3) logarithm: so(3) vector from rotation matrix
inline Vector3d logSO3(const Matrix3d& R)
{
  const double cos_theta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);

  if (theta < 1e-12)
  {
    // Small angle approximation
    return vee3(R - R.transpose()) * 0.5;
  }

  const Matrix3d W = (R - R.transpose()) * (0.5 * theta / std::sin(theta));
  return vee3(W);
}

// SE(3) logarithm (left-trivialized)
// Returns xi = [w; v] such that expSE3(xi) = T
inline Eigen::Matrix<double, 6, 1> logSE3(const Isometry3d& T)
{
  Eigen::Matrix<double, 6, 1> xi;
  xi.setZero();

  const Matrix3d R = T.linear();
  const Vector3d t = T.translation();

  // --- rotation ---
  const Vector3d w = logSO3(R);
  xi.head<3>() = w;

  const double th = w.norm();

  Matrix3d V_inv = Matrix3d::Identity();

  // ---------------------------
  // Small-angle safe handling
  // ---------------------------
  if (th < 1e-8)
  {
    // First-order Taylor expansion
    // V_inv ≈ I - 1/2 W
    const Matrix3d W = skew(w);
    V_inv = Matrix3d::Identity() - 0.5 * W;
  }
  else
  {
    const Matrix3d W = skew(w);
    const double s = std::sin(th);
    const double c = std::cos(th);

    const double A = s / th;
    const double B = (1.0 - c) / (th * th);

    // Safe form (avoids catastrophic cancellation)
    const double coeff = (1.0 - A / (2.0 * B)) / (th * th);

    V_inv = Matrix3d::Identity() - 0.5 * W + coeff * (W * W);
  }

  xi.tail<3>() = V_inv * t;

  return xi;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_LIE_H_
