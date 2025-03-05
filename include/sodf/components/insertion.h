#ifndef SODF_COMPONENTS_INSERTION_H_
#define SODF_COMPONENTS_INSERTION_H_

#include <vector>
#include <utility>

#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Insertion
{
  double distance;  // safe distance before insertion

  geometry::Frame frame;  // +X axis points at the end of the insertion
};

struct InsertionCollection
{
  std::vector<std::pair<std::string, Insertion>> insertion_map;
};

}  // namespace components
}  // namespace sodf

#endif
