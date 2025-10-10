#include <sodf/systems/scene_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sodf {
namespace systems {

// Build object-id → entity map
ecs::ObjectEntityMap make_object_entity_map(ecs::Database& db)
{
  ecs::ObjectEntityMap map;
  db.each([&](ecs::EntityID id, const components::ObjectComponent& obj) { map.emplace(obj.id, id); });
  return map;
}

// Local transform from root → named frame (within one entity)
Eigen::Isometry3d get_local_to_root(ecs::Database& db, ecs::EntityID eid, const std::string& frame_name)
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

// Apply OriginComponent directives to align entity roots
void align_origin_transforms(ecs::Database& db, const ecs::ObjectEntityMap& map)
{
  db.each([&](ecs::EntityID id, components::TransformComponent& tf, components::OriginComponent& origin) {
    if (tf.elements.empty())
      throw std::runtime_error("Cannot align Origin: TransformComponent has no frames.");

    std::visit(
        [&](const auto& variant) {
          using T = std::decay_t<decltype(variant)>;

          if constexpr (std::is_same_v<T, sodf::geometry::Transform>)
          {
            auto& o = tf.elements[0].second;
            o.local = variant.tf;
            o.parent = variant.parent;
            o.dirty = true;
          }
          else if constexpr (std::is_same_v<T, sodf::geometry::AlignFrames>)
          {
            auto it = map.find(variant.target_id);
            if (it == map.end())
              throw std::runtime_error("AlignFrames: target '" + variant.target_id + "' not found.");
            ecs::EntityID target = it->second;

            const auto& src = get_local_to_root(db, id, variant.source_tf);
            const auto& tgt = get_local_to_root(db, target, variant.target_tf);

            auto& o = tf.elements[0].second;
            o.local = tgt.inverse() * src;
            o.parent = variant.target_id;
            o.dirty = true;
          }
          else if constexpr (std::is_same_v<T, sodf::geometry::AlignPairFrames>)
          {
            auto it = map.find(variant.target_id);
            if (it == map.end())
              throw std::runtime_error("AlignPairFrames: target '" + variant.target_id + "' not found.");
            ecs::EntityID target = it->second;

            const auto& src1 = get_local_to_root(db, id, variant.source_tf1);
            const auto& src2 = get_local_to_root(db, id, variant.source_tf2);
            const auto& tgt1 = get_local_to_root(db, target, variant.target_tf1);
            const auto& tgt2 = get_local_to_root(db, target, variant.target_tf2);

            auto tf_align = geometry::alignCenterFrames(tgt1, tgt2, src1, src2, variant.tolerance);

            auto& o = tf.elements[0].second;
            o.local = tf_align;
            o.parent = variant.target_id;
            o.dirty = true;
          }
          else
          {
            throw std::runtime_error("Origin variant not handled.");
          }
        },
        origin.origin);
  });
}

// Find the unique root/global entity (tagged or parentless)
std::optional<ecs::EntityID> find_root_frame_entity(ecs::Database& db)
{
  std::vector<ecs::EntityID> tagged, roots;
  std::size_t ctr = 0;

  db.each([&](ecs::EntityID id, const components::RootFrameTag&) {
    if (ctr++)
      throw std::runtime_error("Multiple RootFrameTag");
    tagged.push_back(id);
  });

  db.each([&](ecs::EntityID id, const components::TransformComponent& tf) {
    if (!tf.elements.empty() && tf.elements[0].second.parent.empty())
      roots.push_back(id);
  });

  if (!tagged.empty())
    return tagged.front();
  if (roots.size() > 1)
    throw std::runtime_error("Multiple root entities (origin.parent empty)");
  if (!roots.empty())
    return roots.front();
  return std::nullopt;
}

// Recursively update one frame’s global transform
void update_global_transform(ecs::Database& db, ecs::EntityID id, components::TransformComponent& tf,
                             std::size_t frame_idx, const ecs::ObjectEntityMap& obj_map,
                             std::optional<ecs::EntityID> global_root)
{
  auto& kv = tf.elements[frame_idx];
  const std::string& frame_name = kv.first;
  auto& frame = kv.second;

  if (!frame.dirty)
    return;

  // Joints drive local pose
  if (frame_idx != 0 && !frame.is_static)
  {
    if (auto* joints = db.get<components::JointComponent>(id))
    {
      auto* joint = ecs::get_element(joints->elements, frame_name);
      if (!joint)
        std::runtime_error("Cannot find joint element: " + frame_name);

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

  if (frame_idx == 0)
  {
    // root/origin: may be attached to another object by name, or world
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
          update_global_transform(db, parent_eid, *ptf, 0, obj_map, global_root);
          frame.global = ptf->elements[0].second.global * frame.local;
        }
        else
        {
          frame.global = frame.local;
        }
      }
      else
      {
        frame.global = frame.local;
      }
    }
    else if (global_root && id != *global_root)
    {
      if (auto* rtf = db.get<components::TransformComponent>(*global_root))
      {
        if (!rtf->elements.empty())
        {
          update_global_transform(db, *global_root, *rtf, 0, obj_map, global_root);
          frame.global = rtf->elements[0].second.global * frame.local;
        }
        else
        {
          frame.global = frame.local;
        }
      }
      else
      {
        frame.global = frame.local;
      }
    }
    else
    {
      frame.global = frame.local;  // world root
    }
    frame.dirty = false;
  }
  else
  {
    // inner frame depends on parent frame
    const std::string& parent_name = frame.parent;
    auto pit =
        std::find_if(tf.elements.begin(), tf.elements.end(), [&](const auto& p) { return p.first == parent_name; });
    if (pit != tf.elements.end())
    {
      std::size_t pidx = static_cast<std::size_t>(std::distance(tf.elements.begin(), pit));
      update_global_transform(db, id, tf, pidx, obj_map, global_root);
      frame.global = pit->second.global * frame.local;
    }
    else
    {
      frame.global = frame.local;
    }
    frame.dirty = false;
  }
}

// Update all entities’ global transforms
void update_all_global_transforms(ecs::Database& db)
{
  auto obj_map = make_object_entity_map(db);
  auto global_root = find_root_frame_entity(db);

  if (global_root)
  {
    if (auto* root_tf = db.get<components::TransformComponent>(*global_root))
    {
      for (std::size_t i = 0; i < root_tf->elements.size(); ++i)
        update_global_transform(db, *global_root, *root_tf, i, obj_map, global_root);
    }
  }

  db.each([&](ecs::EntityID id, components::TransformComponent& tf) {
    if (global_root && id == *global_root)
      return;
    for (std::size_t i = 0; i < tf.elements.size(); ++i)
      update_global_transform(db, id, tf, i, obj_map, global_root);
  });
}

// Get global transform by (entity, frame name)
Eigen::Isometry3d get_global_transform(ecs::Database& db, ecs::EntityID eid, const std::string& frame_name)
{
  auto* tf = db.get_element<components::TransformComponent>(eid, frame_name);
  if (!tf)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");
  return tf->global;
}

// Get global transform by absolute path “/objectId/frame”
Eigen::Isometry3d get_global_transform(ecs::Database& db, const std::string& abs_path)
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
