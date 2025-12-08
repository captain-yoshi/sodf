#ifndef SODF_SYSTEMS_SCENE_GRAPH_H_
#define SODF_SYSTEMS_SCENE_GRAPH_H_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>

#include <Eigen/Geometry>

#include <sodf/database/database.h>
#include <sodf/components/joint.h>
#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/tag.h>
#include <sodf/components/transform.h>

namespace sodf {
namespace systems {

/// ---------------------------------------------------------------------------
/// Generic helpers (DB-like)
//  We keep these light, header-only, and only rely on the minimal API:
///   - each(lambda(EntityID, const ObjectComponent&))
///   - get_const<TransformComponent>(eid)
///   - get_element<TransformComponent>(eid, key)
///
/// This lets us support Database and DatabaseDiff without duplicating logic.
/// ---------------------------------------------------------------------------

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

  // climb to root(0), collect indices
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

  // compose root→child
  for (auto it = chain.rbegin(); it != chain.rend(); ++it)
    T = T * tmap[*it].second.local;

  return T;
}

// Find the unique root/global entity (tagged)
// NOTE: we keep this Database-only for now because it typically depends on tags
//       that are authored in the base DB and not patched in diffs.
std::optional<database::EntityID> find_root_frame_entity(database::Database& db);

/// ---------------------------------------------------------------------------
/// Base (Database) scene graph API
/// ---------------------------------------------------------------------------

Eigen::Isometry3d get_root_global_transform(database::Database& db, const database::ObjectEntityMap& obj_map,
                                            const std::string& object_id);

void update_entity_global_transforms(database::Database& db, database::EntityID eid,
                                     const database::ObjectEntityMap& obj_map);

void update_all_global_transforms(database::Database& db);

Eigen::Isometry3d get_global_transform(database::Database& db, database::EntityID eid, const std::string& frame_name);

Eigen::Isometry3d get_global_transform(database::Database& db, const std::string& abs_path);

/// ---------------------------------------------------------------------------
/// Diff-local transform cache (Path C)
/// ---------------------------------------------------------------------------

class SceneGraphCache
{
public:
  using EntityID = database::EntityID;

  explicit SceneGraphCache(database::DatabaseDiff& diff) : diff_(&diff)
  {
  }

  database::DatabaseDiff& diff() const
  {
    return *diff_;
  }

  void clear()
  {
    revision_ = 0;
    tcomps_.clear();
  }

  /// Public hook for the diff scene-graph implementation.
  components::TransformComponent& get_or_materialize_transform(EntityID eid)
  {
    return get_or_materialize(eid);
  }

private:
  friend Eigen::Isometry3d get_root_global_transform(SceneGraphCache& cache, const database::ObjectEntityMap& obj_map,
                                                     const std::string& object_id);
  friend void update_entity_global_transforms(SceneGraphCache& cache, EntityID eid,
                                              const database::ObjectEntityMap& obj_map);
  friend void update_all_global_transforms(SceneGraphCache& cache);
  friend Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, EntityID eid, const std::string& frame_name);
  friend Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, const std::string& abs_path);

  void ensure_fresh()
  {
    const std::size_t cur = diff_->revision();
    if (revision_ != cur)
    {
      revision_ = cur;
      tcomps_.clear();
    }
  }

  components::TransformComponent& get_or_materialize(EntityID eid);

  database::DatabaseDiff* diff_{ nullptr };
  std::size_t revision_{ 0 };
  std::unordered_map<EntityID, components::TransformComponent> tcomps_;
};

/// ---------------------------------------------------------------------------
/// Diff-aware Scene Graph API using SceneGraphCache
/// ---------------------------------------------------------------------------

Eigen::Isometry3d get_root_global_transform(SceneGraphCache& cache, const database::ObjectEntityMap& obj_map,
                                            const std::string& object_id);

void update_entity_global_transforms(SceneGraphCache& cache, database::EntityID eid,
                                     const database::ObjectEntityMap& obj_map);

void update_all_global_transforms(SceneGraphCache& cache);

Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, database::EntityID eid, const std::string& frame_name);

Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, const std::string& abs_path);

/// ---------------------------------------------------------------------------
/// Convenience overloads for DatabaseDiff users (thin wrappers)
/// ---------------------------------------------------------------------------

inline Eigen::Isometry3d get_root_global_transform(database::DatabaseDiff& diff,
                                                   const database::ObjectEntityMap& obj_map,
                                                   const std::string& object_id)
{
  SceneGraphCache cache(diff);
  return get_root_global_transform(cache, obj_map, object_id);
}

inline void update_entity_global_transforms(database::DatabaseDiff& diff, database::EntityID eid,
                                            const database::ObjectEntityMap& obj_map)
{
  SceneGraphCache cache(diff);
  update_entity_global_transforms(cache, eid, obj_map);
}

inline void update_all_global_transforms(database::DatabaseDiff& diff)
{
  SceneGraphCache cache(diff);
  update_all_global_transforms(cache);
}

inline Eigen::Isometry3d get_global_transform(database::DatabaseDiff& diff, database::EntityID eid,
                                              const std::string& frame_name)
{
  SceneGraphCache cache(diff);
  return get_global_transform(cache, eid, frame_name);
}

inline Eigen::Isometry3d get_global_transform(database::DatabaseDiff& diff, const std::string& abs_path)
{
  SceneGraphCache cache(diff);
  return get_global_transform(cache, abs_path);
}

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_SCENE_GRAPH_H_
