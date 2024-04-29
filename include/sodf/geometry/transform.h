#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

namespace sodf {
namespace geometry {

class Transform
{
public:
  Transform(const KDL::Frame& frame, const std::string& frame_id);
  Transform(const KDL::Vector& position, const KDL::Rotation& rotation, const std::string& frame_id);
  Transform(const Eigen::Isometry3d& frame, const std::string& frame_id);
  Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation, const std::string& frame_id);
  Transform(double px, double py, double pz, double qx, double qy, double qz, double qw, const std::string& frame_id);
  Transform(const geometry_msgs::TransformStamped& frame, const std::string& frame_id);
  Transform(const geometry_msgs::Vector3& position, const geometry_msgs::Quaternion& rotation,
            const std::string& frame_id);
  Transform(const geometry_msgs::Pose& frame, const std::string& frame_id);
  Transform(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& rotation,
            const std::string& frame_id);

  const KDL::Frame& frame() const;
  const std::string& frameId() const;

private:
  const KDL::Frame frame_;
  const std::string frame_id_;
};

}  // namespace geometry
}  // namespace sodf

#endif  // TRANSFORM_H_
