#ifndef TOUCHSCREEN_H_
#define TOUCHSCREEN_H_

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Touchscreen : public Element
{
public:
  Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z);

  virtual ~Touchscreen(){};

  const geometry::Transform& centerTF() const;
  double pressForce() const;

  virtual const geometry::Transform* getTransform() const override
  {
    return &centerTF();
  };

protected:
  const geometry::Transform center_tf_;  // +X points towards the center of the screen
                                         // +Y parallel to bottom/top screen and points towards the left screen
                                         // +Z parallel to left/right screen and points towards the top screen
  const double size_y_;
  const double size_z_;

  std::string current_state_;
};

}  // namespace elements
}  // namespace sodf

#endif  // TOUCHSCREEN_H_
