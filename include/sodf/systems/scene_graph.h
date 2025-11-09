#ifndef SODF_SYSTEMS_SCENE_GRAPH_H_
#define SODF_SYSTEMS_SCENE_GRAPH_H_

#include <optional>
#include <string>
#include <Eigen/Geometry>

#include <sodf/database/database.h>
#include <sodf/components/joint.h>
#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/tag.h>
#include <sodf/components/transform.h>
#include <sodf/geometry/alignment.h>

namespace sodf {
namespace systems {

// Build object-id entity map
database::ObjectEntityMap make_object_entity_map(database::Database& db);

// Local transform from root named frame (within one entity)
Eigen::Isometry3d get_local_to_root(database::Database& db, database::EntityID eid, const std::string& frame_name);

// Find the unique root/global entity (tagged or parentless)
std::optional<database::EntityID> find_root_frame_entity(database::Database& db);

// Recursively update one frame’s global transform
void update_global_transform(database::Database& db, database::EntityID id, components::TransformComponent& tf,
                             std::size_t frame_idx, const database::ObjectEntityMap& obj_map,
                             std::optional<database::EntityID> global_root);

// Update only one entity's global transforms (its whole local subtree).
// Parents are updated on-demand (not forced) if the root points to an external parent.
void update_entity_global_transforms(database::Database& db, database::EntityID eid,
                                     const database::ObjectEntityMap& obj_map);

// Update all entities global transforms
void update_all_global_transforms(database::Database& db);

// Get global transform by (entity, frame name)
Eigen::Isometry3d get_global_transform(database::Database& db, database::EntityID eid, const std::string& frame_name);

// Get global transform by absolute path “/objectId/frame”
Eigen::Isometry3d get_global_transform(database::Database& db, const std::string& abs_path);

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_SCENE_GRAPH_H_
