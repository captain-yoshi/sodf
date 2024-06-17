#pragma once

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Transform : public Element
{
public:
  using pointer = std::unique_ptr<Transform>;

  Transform(const geometry::Transform& tf) : tf_(tf), Element(){};

  const geometry::Transform& tf() const
  {
    return tf_;
  }

  virtual const geometry::Transform* getTransform() const override
  {
    return &tf_;
  };

  virtual bool isMesh() const override
  {
    return false;
  };

private:
  const geometry::Transform tf_;
};

}  // namespace elements
}  // namespace sodf
