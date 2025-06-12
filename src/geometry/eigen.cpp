#include <sodf/geometry/eigen.h>

namespace sodf {
namespace geometry {

// Constructs a rotation matrix from (possibly non-perfect) axes.
// The axes should be linearly independent and right-handed.
// Columns: X, Y, Z
Eigen::Matrix3d computeOrientationFromAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                                           const Eigen::Vector3d& z_axis)
{
  // Stack axes as columns
  Eigen::Matrix3d R;
  R.col(0) = x_axis.normalized();
  R.col(1) = y_axis.normalized();
  R.col(2) = z_axis.normalized();

  // Optionally orthonormalize in case of slight numeric error
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  R = svd.matrixU() * svd.matrixV().transpose();

  // Validate determinant to ensure it's a rotation (not a reflection)
  if (R.determinant() < 0)
  {
    throw std::runtime_error(
        "computeOrientationFromAxes: Provided axes result in a left-handed or invalid rotation matrix.");
  }

  return R;
}

// --- Helper: build isometry from position/axis ---
Eigen::Isometry3d buildIsometry(const Eigen::Vector3d& pos, const Eigen::Vector3d& axis)
{
  // Align +Z to axis (for most shapes)
  Eigen::Vector3d z = axis.normalized();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  if (std::abs(z.dot(x)) > 0.99)
    x = Eigen::Vector3d::UnitY();  // Avoid colinear
  Eigen::Vector3d y = z.cross(x).normalized();
  x = y.cross(z).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = rot;
  iso.translation() = pos;
  return iso;
}

Eigen::Vector3d computeOrthogonalAxis(const Eigen::Vector3d& axis)
{
  Eigen::Vector3d fallback = (std::abs(axis.z()) < 0.99) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
  return axis.cross(fallback).normalized();
}

bool isUnitVector(const Eigen::Vector3d& v, double tol)
{
  return std::abs(v.norm() - 1.0) < tol;
}

bool areVectorsOrthogonal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol)
{
  // Optional: Validate input vectors are unit length (within a tolerance)
  if (std::abs(a.norm() - 1.0) > tol || std::abs(b.norm() - 1.0) > tol)
    return false;

  // Orthogonality: dot product should be zero (within tolerance)
  return std::abs(a.dot(b)) < tol;
}

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol)
{
  // Check unit length
  if (std::abs(a.norm() - 1.0) > tol || std::abs(b.norm() - 1.0) > tol)
    return false;

  // Check orthogonality
  if (std::abs(a.dot(b)) > tol)
    return false;

  return true;
}

bool areVectorsOrthonormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double tol)
{
  // Check all are unit length
  if (std::abs(a.norm() - 1.0) > tol || std::abs(b.norm() - 1.0) > tol || std::abs(c.norm() - 1.0) > tol)
    return false;

  // Check pairwise orthogonality
  if (std::abs(a.dot(b)) > tol)
    return false;
  if (std::abs(a.dot(c)) > tol)
    return false;
  if (std::abs(b.dot(c)) > tol)
    return false;

  return true;
}

}  // namespace geometry
}  // namespace sodf
