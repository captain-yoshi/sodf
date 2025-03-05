#ifndef SODF_COMPONENTS_BUTTON_H_
#define SODF_COMPONENTS_BUTTON_H_

#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Button
{
  double min_force;  // Towards +X axis [N]

  geometry::Frame frame;  // +X axis points towards the button and located at the button centroid
};

struct ButtonCollection
{
  std::vector<std::pair<std::string, Button>> button_map;
};

}  // namespace components
}  // namespace sodf

#endif
