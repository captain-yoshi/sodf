#ifndef SODF_COMPONENTS_ORIGIN_CONSTRAINT_H_
#define SODF_COMPONENTS_ORIGIN_CONSTRAINT_H_

#include <variant>
#include <sodf/geometry/transform.h>

namespace sodf {
namespace components {

struct Coincident
{
  std::string host, guest;
};
struct Concentric
{
  std::string host, guest;
};
struct Parallel
{
  std::string host, guest;
};
struct Angle
{
  std::string host, guest;
  double radians;
};
struct Distance
{
  std::string host, guest;
  double value;
};
struct SeatConeOnCylinder
{
  std::string host_cyl, guest_cone;
  double tol{ 1e-9 };
  int max_it{ 60 };
};

}  // namespace components
}  // namespace sodf

#endif
