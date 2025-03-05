#ifndef SODF_COMPONENT_GRASP_H_
#define SODF_COMPONENT_GRASP_H_

#include <memory>
#include <map>

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace components {

enum class GraspType : uint8_t
{
  PARALLEL_INTERNAL,  // Approach must be greater then the gap size
  PARALLEL_EXTERNAL,  // Approach must be lower then the gap size
  ENCOMPASSING,       // Surround
};

struct Grasp
{
  double gap_size;             // Gap Size along Y axis (GAP_SIZE/2 on +Y and -Y axis )
  double rotation_constraint;  //  Along X axis, e.g. 2 will rotate pose to 0 and 180 degrees.
  GraspType type;

  geometry::Frame frame;  // +X axis points towards gravity at the centroid of the
};

struct GraspCollection
{
  std::vector<std::pair<std::string, Grasp>> grasp_map;
};

}  // namespace components
}  // namespace sodf

#endif
