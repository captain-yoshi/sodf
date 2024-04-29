#include <sodf/geometry/transform.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>

#include <eigen_conversions/eigen_kdl.h>

namespace sodf {
namespace geometry {

namespace {
KDL::Frame transformEigenToKDL(const Eigen::Isometry3d& e)
{
  KDL::Frame k;
  tf::transformEigenToKDL(e, k);

  return k;
}
}  // namespace

Transform::Transform(const KDL::Frame& frame, const std::string& frame_id) : frame_(frame), frame_id_(frame_id)

{
}
Transform::Transform(const KDL::Vector& position, const KDL::Rotation& rotation, const std::string& frame_id)
  : frame_(KDL::Frame(rotation, position)), frame_id_(frame_id)
{
}

Transform::Transform(const Eigen::Isometry3d& frame, const std::string& frame_id)
  : frame_(transformEigenToKDL(frame)), frame_id_(frame_id)
{
}

Transform::Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation, const std::string& frame_id)
  : frame_(transformEigenToKDL(Eigen::Isometry3d(rotation).pretranslate(position))), frame_id_(frame_id)
{
}

Transform::Transform(double px, double py, double pz, double qx, double qy, double qz, double qw,
                     const std::string& frame_id)
  : frame_(KDL::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw), KDL::Vector(px, py, pz))), frame_id_(frame_id)
{
}

Transform::Transform(const geometry_msgs::TransformStamped& frame, const std::string& frame_id)
  : frame_(tf2::transformToKDL(frame)), frame_id_(frame.header.frame_id)
{
}

Transform::Transform(const geometry_msgs::Vector3& position, const geometry_msgs::Quaternion& rotation,
                     const std::string& frame_id)
  : frame_(KDL::Frame(KDL::Rotation::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                      KDL::Vector(position.x, position.y, position.z)))
  , frame_id_(frame_id)

{
}

Transform::Transform(const geometry_msgs::Pose& frame, const std::string& frame_id)
  : frame_(KDL::Frame(KDL::Rotation::Quaternion(frame.orientation.x, frame.orientation.y, frame.orientation.z,
                                                frame.orientation.w),
                      KDL::Vector(frame.position.x, frame.position.y, frame.position.z)))
  , frame_id_(frame_id)
{
}

Transform::Transform(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& rotation,
                     const std::string& frame_id)
  : frame_(KDL::Frame(KDL::Rotation::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                      KDL::Vector(position.x, position.y, position.z)))
  , frame_id_(frame_id)
{
}

const KDL::Frame& Transform::frame() const
{
  return frame_;
}

const std::string& Transform::frameId() const
{
  return frame_id_;
}

}  // namespace geometry
}  // namespace sodf
