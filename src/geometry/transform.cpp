#include <sodf/geometry/transform.h>

#include <tf2_eigen/tf2_eigen.h>

namespace sodf {
namespace geometry {

Transform::Transform(const Eigen::Isometry3d& tf) : tf_(tf)
{
}

Transform::Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation)
  : tf_(Eigen::Isometry3d(rotation).pretranslate(position))
{
}

Transform::Transform(double px, double py, double pz, double qx, double qy, double qz, double qw)
  : tf_(Eigen::Isometry3d(Eigen::Translation3d(px, py, pz) * Eigen::Quaterniond(qw, qx, qy, qz)))
{
}

Transform::Transform(const geometry_msgs::Transform& tf) : tf_(tf2::transformToEigen(tf))
{
}

Transform::Transform(const geometry_msgs::Vector3& position, const geometry_msgs::Quaternion& rotation)
  : tf_(Eigen::Isometry3d(Eigen::Translation3d(position.x, position.y, position.z) *
                          Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)))

{
}

Transform::Transform(const geometry_msgs::Pose& tf)
  : tf_(Eigen::Isometry3d(Eigen::Translation3d(tf.position.x, tf.position.y, tf.position.z) *
                          Eigen::Quaterniond(tf.orientation.w, tf.orientation.x, tf.orientation.y, tf.orientation.z)))
{
}

Transform::Transform(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& rotation)
  : tf_(Eigen::Isometry3d(Eigen::Translation3d(position.x, position.y, position.z) *
                          Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)))
{
}

const Eigen::Isometry3d& Transform::tf() const
{
  return tf_;
}

}  // namespace geometry
}  // namespace sodf
