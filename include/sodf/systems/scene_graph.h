#ifndef SODF_SYSTEMS_SCENE_GRAPH_H_
#define SODF_SYSTEMS_SCENE_GRAPH_H_

#include <Eigen/Geometry>
#include <sodf/ecs.h>

namespace sodf {
namespace systems {
/**
 * @brief Builds a mapping from object IDs (string) to their owning EntityID in the ECS database.
 *
 * This function visits all ObjectComponent instances in the ECS database
 * and constructs an unordered_map that allows fast lookup of the entity ID
 * corresponding to a given object ID.
 *
 * @param db The ECS database to visit.
 * @return std::unordered_map<std::string, EntityID> Map of object id string to EntityID.
 *
 * Usage:
 *   auto object_entity_map = make_object_id_entity_map(db);
 *   EntityID eid = object_entity_map.at("my_object_id");
 */
ObjectEntityMap make_object_entity_map(ginseng::database& db);

/**
 * @brief Computes the local transform of a given frame with respect to the root frame
 *        of the same entity, following the parent chain.
 *
 * This accumulates all local transforms from the root frame up to the specified frame,
 * traversing via each frame's `parent` field. Joint-driven frames are taken as-is,
 * assuming the JointComponent (if present) has already updated them.
 *
 * @param db         The ECS database.
 * @param eid        The entity ID.
 * @param frame_name The name of the local frame.
 * @return           The Eigen::Isometry3d transform from root to the given frame.
 */
Eigen::Isometry3d get_local_to_root(ginseng::database& db, EntityID eid, const std::string& frame_name);

/**
 * @brief Resolve the parent entity ID for the root/origin transform in each TransformComponent.
 *
 * For each TransformComponent, sets tf.parent_ent_id to the EntityID of its parent object,
 * based on the origin's parent string. Throws if the parent is not found.
 */
void resolve_transform_parent_entities(ginseng::database& db, const ObjectEntityMap& obj_ent_map);

/**
 * @brief Updates the local transform of the "origin" frame in entities with AlignFramesComponent,
 *        so a source frame in this entity aligns with a target frame in another entity.
 *
 * This function is used for geometric alignment ("snapping") between two objects.
 * It computes the transform required for the origin frame so that, after propagation,
 * the source frame (in this entity) coincides with the target frame (in the target entity).
 *
 * @param db           The ECS database containing all entities and components.
 * @param obj_ent_map  A map from object id string to entity id, used to resolve target entities.
 *
 */
void align_origin_transforms(ginseng::database& db, const ObjectEntityMap& obj_ent_map);

// Returns the EntityID of the unique root/global entity.
// Throws if there is ambiguity (multiple tags or multiple roots).
std::optional<ginseng::database::ent_id> find_root_frame_entity(ginseng::database& db);

void update_all_global_transforms(ginseng::database& db);

/**
 * @brief Returns the global transform of a named frame in a given entity.
 *
 * @param db         The ECS database.
 * @param eid        The entity ID.
 * @param frame_name The name of the frame (e.g. "top", "button/power").
 * @return           The Eigen::Isometry3d global pose of the frame.
 */
inline Eigen::Isometry3d get_global_transform(ginseng::database& db, EntityID eid, const std::string& frame_name);

/**
 * @brief Returns the global transform for a frame given a single absolute path string.
 *        E.g. "/objectid/transformname" (leading slash required).
 *
 * @param db         The ECS database.
 * @param abs_path   Absolute path string: "/objectid/framename"
 * @return           The Eigen::Isometry3d global pose of the frame.
 */
inline Eigen::Isometry3d get_global_transform(ginseng::database& db, const std::string& abs_path);

}  // namespace systems
}  // namespace sodf

#endif
