#include <sodf/systems/scene_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sodf {
namespace systems {

namespace {

using EntityID = database::EntityID;

static int find_idx(const components::TransformComponent& tf, const std::string& name)
{
  for (std::size_t i = 0; i < tf.elements.size(); ++i)
    if (tf.elements[i].first == name)
      return static_cast<int>(i);
  return -1;
}

// IMPORTANT:
// We intentionally call DatabaseDiff::get_element with a C-string key.
// This avoids the "auto return type deduction recursion" you hit earlier
// when Key was std::string in some template instantiations.
inline const geometry::TransformNode* get_diff_element_node(database::DatabaseDiff& diff, EntityID eid,
                                                            const std::string& frame_name)
{
  return diff.get_element<components::TransformComponent>(eid, frame_name.c_str());
}

// Recursively update one frame’s global transform (returns true if recomputed)
//
// Strategy:
// - The cache snapshot provides stable ordering and storage for globals/dirty.
// - For relationship + local pose, we *prefer* the element override from the diff
//   (if any), otherwise fall back to the cached snapshot node.
//
// This directly addresses RootGlobalReflectsDiffOverride without requiring
// materialize_component to merge element patches.
static bool update_global_transform(SceneGraphCache& cache, EntityID id, components::TransformComponent& tf,
                                    std::size_t frame_idx, const database::ObjectEntityMap& obj_map, bool force)
{
  if (frame_idx >= tf.elements.size())
    return false;

  auto& diff = cache.diff();

  auto& kv = tf.elements[frame_idx];
  const std::string& frame_name = kv.first;
  auto& frame_cached = kv.second;

  // Prefer explicit diff element override if present
  const auto* frame_view = get_diff_element_node(diff, id, frame_name);
  const geometry::TransformNode& frame_src = frame_view ? *frame_view : frame_cached;

  bool parent_recomputed = false;
  Eigen::Isometry3d parent_global = Eigen::Isometry3d::Identity();

  if (frame_idx == 0)
  {
    // Root can have an external parent object
    if (!frame_src.parent.empty())
    {
      auto it = obj_map.find(frame_src.parent);
      if (it == obj_map.end())
        throw std::runtime_error("Parent object id '" + frame_src.parent + "' not found.");

      const EntityID parent_eid = it->second;

      auto& ptf = cache.get_or_materialize_transform(parent_eid);
      if (!ptf.elements.empty())
      {
        parent_recomputed = update_global_transform(cache, parent_eid, ptf, 0, obj_map, /*force=*/false);
        parent_global = ptf.elements[0].second.global;
      }
    }
  }
  else
  {
    const std::string& parent_name = frame_src.parent;

    auto pit =
        std::find_if(tf.elements.begin(), tf.elements.end(), [&](const auto& p) { return p.first == parent_name; });

    if (pit != tf.elements.end())
    {
      std::size_t pidx = static_cast<std::size_t>(std::distance(tf.elements.begin(), pit));
      parent_recomputed = update_global_transform(cache, id, tf, pidx, obj_map, /*force=*/force);
      parent_global = pit->second.global;
    }
    else
    {
      parent_global = Eigen::Isometry3d::Identity();
    }
  }

  const bool view_dirty = frame_view ? frame_view->dirty : false;
  const bool need_recompute = force || frame_cached.dirty || view_dirty || parent_recomputed;

  if (!need_recompute)
    return false;

  // Keep cached node coherent with the source we used
  frame_cached.parent = frame_src.parent;
  frame_cached.local = frame_src.local;
  frame_cached.rest_local = frame_src.rest_local;
  frame_cached.is_static = frame_src.is_static;

  frame_cached.global = parent_global * frame_src.local;
  frame_cached.dirty = false;

  return true;
}

}  // namespace

// ---- SceneGraphCache methods ----

components::TransformComponent& SceneGraphCache::get_or_materialize(EntityID eid)
{
  ensure_fresh();

  auto it = tcomps_.find(eid);
  if (it != tcomps_.end())
    return it->second;

  components::TransformComponent tmp;
  if (!diff_->materialize_component<components::TransformComponent>(eid, tmp))
    throw std::runtime_error("[scene_graph_diff] entity missing TransformComponent in base");

  auto [ins_it, _] = tcomps_.emplace(eid, std::move(tmp));
  return ins_it->second;
}

// ---- Cached API ----

Eigen::Isometry3d get_root_global_transform(SceneGraphCache& cache, const database::ObjectEntityMap& obj_map,
                                            const std::string& object_id)
{
  auto it = obj_map.find(object_id);
  if (it == obj_map.end())
    throw std::runtime_error("[scene_graph_diff] get_root_global_transform: object id '" + object_id + "' not found");

  const EntityID eid = it->second;
  update_entity_global_transforms(cache, eid, obj_map);

  auto& tf = cache.get_or_materialize_transform(eid);
  if (tf.elements.empty())
    throw std::runtime_error("[scene_graph_diff] get_root_global_transform: empty TransformComponent");

  return tf.elements.front().second.global;
}

void update_entity_global_transforms(SceneGraphCache& cache, EntityID eid, const database::ObjectEntityMap& obj_map)
{
  auto& tf = cache.get_or_materialize_transform(eid);
  if (tf.elements.empty())
    throw std::runtime_error("[scene_graph_diff] update_entity_global_transforms: entity has empty TransformComponent");

  const bool force_subtree = tf.elements[0].second.dirty;

  for (std::size_t i = 0; i < tf.elements.size(); ++i)
    update_global_transform(cache, eid, tf, i, obj_map, /*force=*/force_subtree);
}

void update_all_global_transforms(SceneGraphCache& cache)
{
  auto obj_map = make_object_entity_map(cache.diff());

  cache.diff().each(
      [&](EntityID id, const components::TransformComponent&) { update_entity_global_transforms(cache, id, obj_map); });
}

Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, EntityID eid, const std::string& frame_name)
{
  auto obj_map = make_object_entity_map(cache.diff());
  update_entity_global_transforms(cache, eid, obj_map);

  auto& tf = cache.get_or_materialize_transform(eid);
  const int idx = find_idx(tf, frame_name);
  if (idx < 0)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");

  return tf.elements[static_cast<std::size_t>(idx)].second.global;
}

Eigen::Isometry3d get_global_transform(SceneGraphCache& cache, const std::string& abs_path)
{
  if (abs_path.empty() || abs_path[0] != '/')
    throw std::runtime_error("get_global_transform(diff): path must start with '/'");

  const std::size_t slash_pos = abs_path.find('/', 1);
  if (slash_pos == std::string::npos)
    throw std::runtime_error("get_global_transform(diff): use '/objectId/frameName'");

  const std::string object_id = abs_path.substr(1, slash_pos - 1);
  const std::string frame_name = abs_path.substr(slash_pos + 1);

  auto map = make_object_entity_map(cache.diff());
  auto it = map.find(object_id);
  if (it == map.end())
    throw std::runtime_error("get_global_transform(diff): object id '" + object_id + "' not found");

  return get_global_transform(cache, it->second, frame_name);
}

}  // namespace systems
}  // namespace sodf
