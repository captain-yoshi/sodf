#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <Eigen/Geometry>

namespace sodf {
namespace geometry {

struct Transform
{
  std::string parent;
  std::string child;
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
};

}  // namespace geometry
}  // namespace sodf

#endif  // TRANSFORM_H_
