#ifndef SODF_SYSTEMS_APPLY_ORIGIN_CONSTRAINTS_H_
#define SODF_SYSTEMS_APPLY_ORIGIN_CONSTRAINTS_H_

#include <sodf/ecs/database.h>

namespace sodf {
namespace systems {

/**
 * Apply OriginComponent constraints to position each entity's root frame.
 * - Supports geometry::Transform / AlignFrames / AlignPairFrames
 * - Supports constraint leaves: Coincident, Concentric, Parallel, Angle, Distance, SeatConeOnCylinder
 * - Uses selector_context_db to resolve strings like "table/insert/M24#axis"
 */
void apply_origin_constraints(ecs::Database& db, const ecs::ObjectEntityMap& object_map);

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_APPLY_ORIGIN_CONSTRAINTS_H_
