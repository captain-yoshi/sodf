#ifndef SODF_SYSTEMS_LIQUID_LEVEL_JOINT_H_
#define SODF_SYSTEMS_LIQUID_LEVEL_JOINT_H_

#include <sodf/ecs/database.h>

#include <sodf/components/container.h>
#include <sodf/components/joint.h>
#include <sodf/components/transform.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/shape.h>

namespace sodf {
namespace systems {

/**
 * @brief Update all fluid liquid-level joints using the container's DomainShape.
 */
void update_liquid_level_joints_from_domain(ecs::Database& db, Eigen::Vector3d& gravity_world);

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_LIQUID_LEVEL_JOINT_H_
