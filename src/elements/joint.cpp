#include <sodf/elements/joint.h>

namespace sodf {
namespace elements {

PassiveJoint::PassiveJoint(const geometry::Transform& tf, const KDL::Joint& joint,
                           const std::map<std::string, double>& position_map)
  : tf_(tf), joint_(joint), position_map_(position_map), Element()
{
}

const geometry::Transform& PassiveJoint::tf() const
{
  return tf_;
}
const KDL::Joint& PassiveJoint::joint() const
{
  return joint_;
}

const std::map<std::string, double>& PassiveJoint::jointPositionMap() const
{
  return position_map_;
}

double PassiveJoint::getJointPosition(const std::string& state) const
{
  auto node = position_map_.find(state);
  if (node == position_map_.end())
    throw std::runtime_error("joint position state not in map: " + state);

  return node->second;
}

}  // namespace elements
}  // namespace sodf
