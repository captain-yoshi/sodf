#ifndef SODF_COMPONENTS_ORIGIN_H_
#define SODF_COMPONENTS_ORIGIN_H_

#include <variant>
#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct OriginComponent
{
  using OriginVariant = std::variant<geometry::Transform,       // direct transform (absolute pose)
                                     geometry::AlignFrames,     // single-frame alignment
                                     geometry::AlignPairFrames  // two-frame alignment
                                     >;

  OriginVariant origin;
};

}  // namespace components
}  // namespace sodf

#endif
