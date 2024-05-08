#include <sodf/conversion.h>

namespace sodf {

Eigen::Isometry3d RPY(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());

  return Eigen::Isometry3d(Eigen::Quaterniond(y * p * r));
}
}  // namespace sodf
