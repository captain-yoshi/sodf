#ifndef SODF_COMPONENTS_ORIGIN_H_
#define SODF_COMPONENTS_ORIGIN_H_

#include <variant>
#include <sodf/geometry/transform.h>
#include <sodf/components/origin_constraint.h>

namespace sodf {
namespace components {

/// One operation in the origin “program”
using OriginConstraint = std::variant<geometry::Transform,  // absolute pose
                                      Coincident,           //
                                      Concentric,           //
                                      Parallel,             //
                                      Angle,                //
                                      Distance,             //
                                      SeatConeOnCylinder    //
                                      >;

struct OriginComponent
{
  // Optional default scoping for local refs (so you can write "insert/M24")
  std::string host_object;   // e.g. "table"
  std::string guest_object;  // e.g. this object id (often set by loader)

  // A sequence of steps applied in order; last one wins.
  std::vector<OriginConstraint> constraints;
};

}  // namespace components
}  // namespace sodf

#endif
