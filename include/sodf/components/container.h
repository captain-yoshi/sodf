#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <vector>

#include <Eigen/Geometry>

#include <sodf/geometry/volume.h>
#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Container
{
  double volume = 0.0;  // SI m^3

  // collection of 3d volumes wrt the frame along the -X axis
  std::vector<geometry::BaseVolumePtr> shape;
};

struct ContainerCollection
{
  std::vector<std::pair<std::string, Container> > container_map;
};

}  // namespace components
}  // namespace sodf

#endif
