#include <sodf/elements/touchscreen.h>

namespace sodf {
namespace elements {

Touchscreen::Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z)
  : center_tf_(center_tf), size_y_(size_y), size_z_(size_z)
{
}

const geometry::Transform& Touchscreen::centerTF() const
{
  return center_tf_;
}

}  // namespace elements
}  // namespace sodf
