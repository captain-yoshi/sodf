#ifndef SODF_COMPONENTS_JOINT_H_
#define SODF_COMPONENTS_JOINT_H_

#include <kdl/joint.hpp>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct Joint
{
  KDL::Joint joint;
  std::string parent;
  double joint_position = 0.0;
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
};

struct JointCollection
{
  std::vector<std::pair<std::string, Joint>> joint_map;
};

}  // namespace components
}  // namespace sodf

#endif
