#ifndef SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_
#define SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_

#include <sodf/database/database.h>
#include <sodf/assembly/constraint_selector.h>

namespace sodf {
namespace assembly {

/**
 * Build a SelectorContext wired to your DATABASE Database.
 *
 * @param db        Your database::Database
 * @param objects   Map from object id string -> entity id (database::ObjectEntityMap)
 */
SelectorContext makeSelectorContext(database::Database& db, const database::ObjectEntityMap& objects);

SelectorContext makeSelectorContextInHostSpace(sodf::database::Database& db,
                                               const sodf::database::ObjectEntityMap& objects,
                                               const std::string& host_object_id);

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_SELECTOR_CONTEXT_DB_H_
