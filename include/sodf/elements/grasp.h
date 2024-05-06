#ifndef GRASP_H_
#define GRASP_H_

#include <memory>
#include <map>

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

enum class GraspType : uint8_t
{
  PARALLEL_INTERNAL,  // Approach must be greater then the gap size
  PARALLEL_EXTERNAL,  // Approach must be lower then the gap size
  ENCOMPASSING,       // Surround
};

class Grasp : public Element
{
public:
  Grasp(const geometry::Transform& center_tf, double gap_size, double height, double rotation_constraint,
        GraspType type);

  virtual ~Grasp(){};

  const geometry::Transform& centerTF() const;
  double gapSize() const;
  double height() const;
  double rotationConstraint() const;
  GraspType type() const;

  virtual bool addFramesToTree(KDL::Tree& tree) override;

protected:
  const geometry::Transform center_tf_;  // X axis pointing towards the bottom of the container, wrt. object origin

  double gap_size_;             // Gap Size along Y axis (GAP_SIZE/2 on +Y and -Y axis )
  double height_;               //
  double rotation_constraint_;  //  Along X axis, e.g. 2 will rotate pose to 0 and 180 degrees.
  GraspType type_;              //
};

}  // namespace elements
}  // namespace sodf

#endif  // GRASP_H_
