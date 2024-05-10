#include <sodf/elements/grasp.h>

namespace sodf {
namespace elements {

Grasp::Grasp(const geometry::Transform& center_tf, double gap_size, double height, double rotation_constraint,
             GraspType type)
  : center_tf_(center_tf)
  , gap_size_(gap_size)
  , height_(height)
  , rotation_constraint_(rotation_constraint)
  , type_(type)
  , Element()
{
}

const geometry::Transform& Grasp::centerTF() const
{
  return center_tf_;
}
double Grasp::gapSize() const
{
  return gap_size_;
}
double Grasp::height() const
{
  return height_;
}
double Grasp::rotationConstraint() const
{
  return rotation_constraint_;
}

GraspType Grasp::type() const
{
  return type_;
}

}  // namespace elements
}  // namespace sodf
