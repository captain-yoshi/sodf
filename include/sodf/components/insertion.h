#ifndef SODF_COMPONENTS_INSERTION_H_
#define SODF_COMPONENTS_INSERTION_H_

#include <sodf/components/data_type.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

// What side is this feature describing?
enum class InsertionRole : uint8_t
{
  Receptacle,  // cavity / socker /receptacle (female)
  Insert       // insertable / exterior (male)
};

struct Insertion
{
  InsertionRole role;

  // Axis along which we approach and insert (local to target_frame_id).
  Eigen::Vector3d axis_insertion;
  Eigen::Vector3d axis_reference;
  uint32_t rotational_symmetry = 1;  // Number of unique orientations wrt. the insertion axis
                                     // 0 = infinite, 1 = unique, 2 = 0 and 180 degrees, etc...

  std::string stacked_shape_id;
  std::string stacked_shape_frame_id;

  // Receptacle: max allowed travel along +axis_insertion from mouth/opening
  //             sockets don't usally have a max depth
  // Insert: usally no max_depth (could be used to set the distance to a shoulder)
  double max_depth = 0.0;
};

struct InsertionComponent
{
  ElementMap<std::string, Insertion> elements;
};

}  // namespace components
}  // namespace sodf

#endif
