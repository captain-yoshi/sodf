#ifndef JOINT_H_
#define JOINT_H_

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class PassiveJoint : public Element
{
public:
  PassiveJoint(const geometry::Transform& tf, const KDL::Joint& joint,
               const std::map<std::string, double>& position_map);

  virtual ~PassiveJoint(){};

  const geometry::Transform& tf() const;
  const KDL::Joint& joint() const;

  double getJointPosition(const std::string& state) const;
  const std::map<std::string, double>& jointPositionMap() const;

  virtual const geometry::Transform* getTransform() const override
  {
    return &tf();
  };

  virtual const KDL::Joint* getJoint() const override
  {
    return &joint_;
  };

protected:
  const geometry::Transform tf_;
  const KDL::Joint joint_;
  std::map<std::string, double> position_map_;
};

}  // namespace elements
}  // namespace sodf

#endif  // JOINT_H_
