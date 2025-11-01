#ifndef SODF_PHYSICS_DOMAIN_SHAPE_CONVERSION_H_
#define SODF_PHYSICS_DOMAIN_SHAPE_CONVERSION_H_

#include <sodf/geometry/shape.h>
#include <sodf/physics/domain_shape.h>

namespace sodf {
namespace physics {

DomainShapeBasePtr shapeToDomainShape(const geometry::Shape& s, sodf::physics::DomainType dtype);

DomainShapeBasePtr shapeMeshToDomainShape(const geometry::Shape& meshShape, const geometry::TriangleMesh& mesh,
                                          sodf::physics::DomainType dtype);

DomainShapeBasePtr shapeMeshToDomainShape(const geometry::Shape& meshShape, sodf::physics::DomainType dtype);

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_DOMAIN_SHAPE_CONVERSION_H_
