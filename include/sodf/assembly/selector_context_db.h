#ifndef SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_
#define SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_

#include <sodf/ecs/database.h>
#include <sodf/assembly/constraint_selector.h>

namespace sodf {
namespace assembly {

/**
 * Build a SelectorContext wired to your ECS Database.
 *
 * @param db        Your ecs::Database
 * @param objects   Map from object id string -> entity id (ecs::ObjectEntityMap)
 */
SelectorContext makeSelectorContext(ecs::Database& db, const ecs::ObjectEntityMap& objects);

SelectorContext makeSelectorContextInHostSpace(sodf::ecs::Database& db, const sodf::ecs::ObjectEntityMap& objects,
                                               const std::string& host_object_id);

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_
