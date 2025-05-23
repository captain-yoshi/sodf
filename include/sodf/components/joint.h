#ifndef SODF_COMPONENTS_JOINT_H_
#define SODF_COMPONENTS_JOINT_H_

#include <Eigen/Geometry>
#include <sodf/ecs.h>

namespace sodf {
namespace components {

enum class JointType
{
  FIXED,
  REVOLUTE,
  PRISMATIC
};

enum class JointActuation
{
  FIXED,    // when JointType = FIXED
  ACTIVE,   // actively commanded
  PASSIVE,  // free to move (e.g. floating)
  SPRING,   // has a restoring force but no actuator
  VIRTUAL,  // e.g. height of a liquid
};

struct Joint
{
  JointType type = JointType::FIXED;
  JointActuation actuation = JointActuation::FIXED;

  double position = 0.0;  // angle (rad) or displacement (m)
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
};

struct JointComponent
{
  FlatMap<std::string, Joint> joint_map;  //

  // Joint* get(const std::string& name)
  // {
  //   for (auto& [n, joint] : joints)
  //   {
  //     if (n == name)
  //       return &joint;
  //   }
  //   return nullptr;
  // }
};

}  // namespace components
}  // namespace sodf

#endif
