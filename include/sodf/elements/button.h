#ifndef BUTTON_H_
#define BUTTON_H_

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Button : public Element
{
public:
  Button(const geometry::Transform& center_tf, double press_force);

  virtual ~Button(){};

  const geometry::Transform& centerTF() const;
  double pressForce() const;

  virtual const geometry::Transform* getTransform() const override
  {
    return &centerTF();
  };

protected:
  const geometry::Transform center_tf_;  // +X axis pointing towards the center of a button

  const double press_force_;  // Towards +X axis in Newtons
};

}  // namespace elements
}  // namespace sodf

#endif  // BUTTON_H_
