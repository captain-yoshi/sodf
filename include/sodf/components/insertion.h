#ifndef SODF_COMPONENTS_INSERTION_H_
#define SODF_COMPONENTS_INSERTION_H_

#include <sodf/components/data_type.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct Insertion
{
  // Axis along which we approach and insert (local to target_frame_id).
  Eigen::Vector3d axis_insertion;
  Eigen::Vector3d axis_reference;
  uint32_t rotational_symmetry = 1;  // Number of unique orientations wrt. the axis
                                     // 0 = infinite, 1 = unique, 2 = 0 and 180 degrees, etc...

  // Distance to stage along -axis_insertion before engaging [m]
  double approach_offset = 0.0;

  // Max allowed travel along +axis_insertion from mouth/opening
  double max_depth = 0.0;
};

struct InsertionComponent
{
  ElementMap<std::string, Insertion> elements;
};

}  // namespace components
}  // namespace sodf

#endif
