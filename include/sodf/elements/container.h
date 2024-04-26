#ifndef CONTAINER_H_
#define CONTAINER_H_

#include <memory>
#include <map>

#include <sodf/geometry/volume.h>
#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Container;

using ContainerMap = std::map<std::string, Container>;

class Container : public Element
{
public:
  Container(const std::vector<geometry::BaseVolumePtr>& shape);
  Container(const std::vector<geometry::BaseVolumePtr>& shape, const Eigen::Isometry3d& bottom_tf);

  virtual ~Container(){};

  double getCurrentVolume() const;
  double getMaxVolume() const;
  double getRemainingVolume() const;
  double addVolume(double volume);
  double removeVolume(double volume);

  double getCurrentHeight() const;  // wrt. current volume
  double getMaxHeight() const;
  double
  getHeightFromAddingVolume(double volume) const;  // height if adding or removing the volume to the current volume

  const Eigen::Isometry3d& getBottomTF() const;

protected:
  double volume_ = 0;
  const double max_volume_;

  const Eigen::Isometry3d bottom_tf_;  // X axis pointing towards the bottom of the container, wrt. object origin

  const std::vector<geometry::BaseVolumePtr> shape_;
};

}  // namespace elements
}  // namespace sodf

#endif  // CONTAINER_H_
