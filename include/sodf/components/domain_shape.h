#ifndef SODF_COMPONENTS_DOMAIN_SHAPE_H_
#define SODF_COMPONENTS_DOMAIN_SHAPE_H_

#include <sodf/components/data_type.h>
#include <sodf/components/def_or_ref.h>
#include <sodf/physics/domain_shape.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct DomainShapeComponent
{
  using definition_type = physics::DomainShape;
  ElementMap<std::string, DefOrRef<definition_type>> elements;
};

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_DOMAIN_SHAPE_H_
