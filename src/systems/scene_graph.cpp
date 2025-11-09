#include <sodf/systems/scene_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sodf {
namespace systems {

// Build object-id → entity map
database::ObjectEntityMap make_object_entity_map(database::Database& db)
{
  database::ObjectEntityMap map;
  db.each([&](database::EntityID id, const components::ObjectComponent& obj) { map.emplace(obj.id, id); });
  return map;
}

// Local transform from root → named frame (within one entity)
Eigen::Isometry3d get_local_to_root(database::Database& db, database::EntityID eid, const std::string& frame_name)
{
  const auto* tfc = db.get_const<components::TransformComponent>(eid);
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
std::optional<database::EntityID> find_root_frame_entity(database::Database& db)
{
  std::vector<database::EntityID> tagged;
  db.each([&](database::EntityID id, const components::RootFrameTag&) { tagged.push_back(id); });
  if (tagged.empty())
    return std::nullopt;  // no explicit global root
  if (tagged.size() > 1)
    throw std::runtime_error("Multiple RootFrameTag");
  return tagged.front();
}

// Recursively update one frame’s global transform (returns true if recomputed)
static bool update_global_transform(database::Database& db, database::EntityID id, components::TransformComponent& tf,
                                    std::size_t frame_idx, const database::ObjectEntityMap& obj_map, bool force)
{
  auto& kv = tf.elements[frame_idx];
  const auto& frame_name = kv.first;
  auto& frame = kv.second;

  // --- 1) Ensure parent is up to date; capture its global if any
  bool parent_recomputed = false;
  Eigen::Isometry3d parent_global = Eigen::Isometry3d::Identity();

  if (frame_idx == 0)
  {
    // Root can have an external parent object
    if (!frame.parent.empty())
    {
      auto it = obj_map.find(frame.parent);
      if (it == obj_map.end())
        throw std::runtime_error("Parent object id '" + frame.parent + "' not found.");
      auto parent_eid = it->second;

      if (auto* ptf = db.get<components::TransformComponent>(parent_eid))
      {
        if (!ptf->elements.empty())
        {
          // Do NOT force the parent entity; just update if it’s dirty.
          parent_recomputed = update_global_transform(db, parent_eid, *ptf, 0, obj_map, /*force=*/false);
          parent_global = ptf->elements[0].second.global;
        }
      }
    }
  }
  else
  {
    // Inner frame depends on another frame in the same TransformComponent
    const std::string& parent_name = frame.parent;
    auto pit =
        std::find_if(tf.elements.begin(), tf.elements.end(), [&](const auto& p) { return p.first == parent_name; });

    if (pit != tf.elements.end())
    {
      std::size_t pidx = static_cast<std::size_t>(std::distance(tf.elements.begin(), pit));
      parent_recomputed = update_global_transform(db, id, tf, pidx, obj_map, /*force=*/force);
      parent_global = pit->second.global;
    }
    else
    {
      parent_global = Eigen::Isometry3d::Identity();
    }
  }

  // --- 2) Decide whether THIS node needs recompute
  const bool need_recompute = force || frame.dirty || parent_recomputed;
  if (!need_recompute)
    return false;

  // --- 3) If joint-driven, (re)build local before composing global
  if (frame_idx != 0 && !frame.is_static)
  {
    if (auto* joints = db.get<components::JointComponent>(id))
    {
      auto* joint = database::get_element(joints->elements, frame_name);
      if (!joint)
        throw std::runtime_error("Cannot find joint element: " + frame_name);

      switch (joint->type)
      {
        case components::JointType::REVOLUTE:
        {
          Eigen::Vector3d axis = joint->axes.col(0);
          frame.local = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(joint->position[0], axis.normalized());
          break;
        }
        case components::JointType::PRISMATIC:
        {
          Eigen::Vector3d axis = joint->axes.col(0);
          frame.local = Eigen::Translation3d(joint->position[0] * axis.normalized());
          break;
        }
        case components::JointType::SPHERICAL:
        {
          double yaw = joint->position[0], pitch = joint->position[1], roll = joint->position[2];
          Eigen::AngleAxisd Rx(roll, joint->axes.col(0).normalized());
          Eigen::AngleAxisd Ry(pitch, joint->axes.col(1).normalized());
          Eigen::AngleAxisd Rz(yaw, joint->axes.col(2).normalized());
          frame.local = Eigen::Translation3d(0, 0, 0) * (Rz * Ry * Rx);
          break;
        }
        case components::JointType::PLANAR:
        {
          double tx = joint->position[0], ty = joint->position[1], rz = joint->position[2];
          Eigen::Vector3d x = joint->axes.col(0).normalized();
          Eigen::Vector3d y = joint->axes.col(1).normalized();
          Eigen::Vector3d n = joint->axes.col(2).normalized();
          frame.local = Eigen::Translation3d(tx * x + ty * y) * Eigen::AngleAxisd(rz, n);
          break;
        }
        case components::JointType::FLOATING:
        {
          Eigen::Vector3d t = joint->position.segment<3>(0);
          double yaw = joint->position[3], pitch = joint->position[4], roll = joint->position[5];
          Eigen::AngleAxisd Rx(roll, joint->axes.col(3).normalized());
          Eigen::AngleAxisd Ry(pitch, joint->axes.col(4).normalized());
          Eigen::AngleAxisd Rz(yaw, joint->axes.col(5).normalized());
          frame.local = Eigen::Translation3d(t) * (Rz * Ry * Rx);
          break;
        }
        case components::JointType::FIXED:
        default:
          break;
      }
    }
  }

  // --- 4) Compose global
  if (frame_idx == 0)
  {
    // world or attached to external parent
    frame.global = parent_global * frame.local;
  }
  else
  {
    frame.global = parent_global * frame.local;
  }

  frame.dirty = false;
  return true;
}

void update_entity_global_transforms(database::Database& db, database::EntityID eid,
                                     const database::ObjectEntityMap& obj_map)
{
  auto* tf = db.get<components::TransformComponent>(eid);
  if (!tf)
    throw std::runtime_error("[scene_graph] update_entity_global_transforms: entity " + std::to_string(eid.index) +
                             " is missing TransformComponent.");
  if (tf->elements.empty())
    throw std::runtime_error("[scene_graph] update_entity_global_transforms: entity " + std::to_string(eid.index) +
                             " has an empty TransformComponent (no frames).");

  // If the root was just modified, force the whole local subtree for this entity.
  const bool force_subtree = tf->elements[0].second.dirty;

  for (std::size_t i = 0; i < tf->elements.size(); ++i)
  {
    // This call already:
    //  - updates the parent entity’s root if needed (and only if needed),
    //  - respects `force_subtree` for this entity,
    //  - walks children frames inside this TransformComponent.
    update_global_transform(db, eid, *tf, i, obj_map, /*force=*/force_subtree);
  }
}

// Update all entities’ global transforms (forces subtree when root is dirty)
void update_all_global_transforms(database::Database& db)
{
  auto obj_map = make_object_entity_map(db);

  db.each([&](database::EntityID id, components::TransformComponent& tf) {
    // If root is dirty, force the whole subtree of this entity
    const bool force_subtree = (!tf.elements.empty() && tf.elements[0].second.dirty);

    for (std::size_t i = 0; i < tf.elements.size(); ++i)
      update_global_transform(db, id, tf, i, obj_map, force_subtree);
  });
}

// Get global transform by (entity, frame name)
Eigen::Isometry3d get_global_transform(database::Database& db, database::EntityID eid, const std::string& frame_name)
{
  auto* tf = db.get_element<components::TransformComponent>(eid, frame_name);
  if (!tf)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");
  return tf->global;
}

// Get global transform by absolute path “/objectId/frame”
Eigen::Isometry3d get_global_transform(database::Database& db, const std::string& abs_path)
{
  if (abs_path.empty() || abs_path[0] != '/')
    throw std::runtime_error("get_global_transform: path must start with '/'");

  const std::size_t slash_pos = abs_path.find('/', 1);
  if (slash_pos == std::string::npos)
    throw std::runtime_error("get_global_transform: use '/objectId/frameName'");

  const std::string object_id = abs_path.substr(1, slash_pos - 1);
  const std::string frame_name = abs_path.substr(slash_pos + 1);

  auto map = make_object_entity_map(db);
  auto it = map.find(object_id);
  if (it == map.end())
    throw std::runtime_error("get_global_transform: object id '" + object_id + "' not found");

  return get_global_transform(db, it->second, frame_name);
}

}  // namespace systems
}  // namespace sodf
