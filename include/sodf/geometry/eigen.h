#ifndef SODF_GEOMETRY_EIGEN_H_
#define SODF_GEOMETRY_EIGEN_H_

#include <Eigen/Geometry>

namespace sodf {
namespace geometry {

inline std::ostream& operator<<(std::ostream& os, const Eigen::Vector3d& v)
{
  os << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return os;
}

// Constructs a rotation matrix from (possibly non-perfect) axes.
// The axes should be linearly independent and right-handed.
// Columns: X, Y, Z
Eigen::Matrix3d computeOrientationFromAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis,
                                           const Eigen::Vector3d& z_axis);

// --- Helper: build isometry from position/axis ---
Eigen::Isometry3d buildIsometry(const Eigen::Vector3d& pos, const Eigen::Vector3d& axis);

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_EIGEN_H_
