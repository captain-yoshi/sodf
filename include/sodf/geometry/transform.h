#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <Eigen/Geometry>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

namespace sodf {
namespace geometry {

class Transform
{
public:
  Transform(const Eigen::Isometry3d& tf);
  Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation);
  Transform(double px, double py, double pz, double qx, double qy, double qz, double qw);
  Transform(const geometry_msgs::Transform& tf);
  Transform(const geometry_msgs::Vector3& position, const geometry_msgs::Quaternion& rotation);
  Transform(const geometry_msgs::Pose& tf);
  Transform(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& rotation);

  const Eigen::Isometry3d& tf() const;

private:
  const Eigen::Isometry3d tf_;
};

}  // namespace geometry
}  // namespace sodf

#endif  // TRANSFORM_H_
