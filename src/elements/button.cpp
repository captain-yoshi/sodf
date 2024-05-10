
#include <sodf/elements/button.h>

namespace sodf {
namespace elements {

Button::Button(const geometry::Transform& center_tf, double press_force)
  : center_tf_(center_tf), press_force_(press_force), Element()
{
}

const geometry::Transform& Button::centerTF() const
{
  return center_tf_;
}
double Button::pressForce() const
{
  return press_force_;
}

}  // namespace elements
}  // namespace sodf
