#ifndef ELEMENT_H_
#define ELEMENT_H_

#include <memory>

#include <kdl/tree.hpp>

#include <sodf/geometry/transform.h>

namespace sodf {

class Element;

using ElementPtr = std::shared_ptr<Element>;
using ElementUniquePtr = std::unique_ptr<Element>;

class Element
{
public:
  using pointer = std::unique_ptr<Element>;
  virtual ~Element() = default;

  virtual const geometry::Transform* getTransform() const
  {
    return nullptr;
  };
  virtual const KDL::Joint* getJoint() const
  {
    return nullptr;
  };

protected:
  Element(){};
};

}  // namespace sodf

#endif  // ELEMENT_H_
