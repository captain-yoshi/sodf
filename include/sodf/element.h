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

  virtual bool addFramesToTree(KDL::Tree& tree)
  {
    return true;
  };

  bool addFrameToTree(KDL::Tree& tree, const geometry::Transform& tf)
  {
    return tree.addSegment(KDL::Segment(tf.frameId(), KDL::Joint(KDL::Joint::None), tf.frame()), tf.parentFrameId());
  };

protected:
  Element(){};
};

}  // namespace sodf

#endif  // ELEMENT_H_
