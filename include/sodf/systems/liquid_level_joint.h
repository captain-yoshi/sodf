#pragma once

#include <Eigen/Core>
#include <sodf/database/database.h>

namespace sodf {
namespace systems {

// Existing base-db version
void update_liquid_level_joints_from_domain(database::Database& db, Eigen::Vector3d& gravity_world);

// NEW diff-safe version
void update_liquid_level_joints_from_domain(database::DatabaseDiff& diff, Eigen::Vector3d& gravity_world);

}  // namespace systems
}  // namespace sodf
