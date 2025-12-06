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

namespace sodf {
namespace systems {

// -----------------------------------------------------------------------------
// Read-only helpers (generic: Database or DatabaseDiff)
// -----------------------------------------------------------------------------

// Build object-id → entity map
template <class DBLike>
database::ObjectEntityMap make_object_entity_map(DBLike& db)
{
  database::ObjectEntityMap map;
  db.each([&](database::EntityID id, const components::ObjectComponent& obj) { map.emplace(obj.id, id); });
  return map;
}

// Local transform from root → named frame (within one entity)
template <class DBLike>
Eigen::Isometry3d get_local_to_root(DBLike& db, database::EntityID eid, const std::string& frame_name)
{
  const auto* tfc = db.template get_const<components::TransformComponent>(eid);
  if (!tfc)
    throw std::runtime_error("Entity missing TransformComponent.");

  const auto& tmap = tfc->elements;

  auto find_idx = [&](const std::string& name) -> int {
    for (size_t i = 0; i < tmap.size(); ++i)
      if (tmap[i].first == name)
        return static_cast<int>(i);
    return -1;
  };

  const int idx = find_idx(frame_name);
  if (idx == -1)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  std::vector<int> chain;
  for (int cur = idx; cur != 0;)
  {
    chain.push_back(cur);
    const std::string& parent = tmap[cur].second.parent;
    const int pidx = find_idx(parent);
    if (pidx == -1)
      throw std::runtime_error("Parent frame '" + parent + "' not found in entity.");
    cur = pidx;
  }

  for (auto it = chain.rbegin(); it != chain.rend(); ++it)
    T = T * tmap[*it].second.local;

  return T;
}

// Find the unique root/global entity (tagged)
template <class DBLike>
std::optional<database::EntityID> find_root_frame_entity(DBLike& db)
{
  std::vector<database::EntityID> tagged;
  db.each([&](database::EntityID id, const components::RootFrameTag&) { tagged.push_back(id); });
  if (tagged.empty())
    return std::nullopt;
  if (tagged.size() > 1)
    throw std::runtime_error("Multiple RootFrameTag");
  return tagged.front();
}

// -----------------------------------------------------------------------------
// Global-cache update + queries (Database-only for now)
// -----------------------------------------------------------------------------

/// Get the global transform of the root frame (first element) for a given object id.
/// Uses an existing ObjectEntityMap.
Eigen::Isometry3d get_root_global_transform(database::Database& db, const database::ObjectEntityMap& obj_map,
                                            const std::string& object_id);

// Update only one entity's global transforms (its whole local subtree).
// Parents are updated on-demand (not forced) if the root points to an external parent.
void update_entity_global_transforms(database::Database& db, database::EntityID eid,
                                     const database::ObjectEntityMap& obj_map);

// Update all entities’ global transforms
void update_all_global_transforms(database::Database& db);

// Get global transform by (entity, frame name)
Eigen::Isometry3d get_global_transform(database::Database& db, database::EntityID eid, const std::string& frame_name);

// Get global transform by absolute path “/objectId/frame”
Eigen::Isometry3d get_global_transform(database::Database& db, const std::string& abs_path);

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_SCENE_GRAPH_H_
