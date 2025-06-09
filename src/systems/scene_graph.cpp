#include <sodf/systems/scene_graph.h>

#include <sodf/components/joint.h>
#include <sodf/components/transform.h>
#include <sodf/components/object.h>
#include <sodf/geometry/utilities.h>

namespace sodf {
namespace systems {

ObjectEntityMap make_object_entity_map(ginseng::database& db)
{
  ObjectEntityMap obj_ent_map;
  db.visit([&](EntityID id, const components::ObjectComponent& object) { obj_ent_map.insert({ object.id, id }); });
  return obj_ent_map;
}

Eigen::Isometry3d get_local_to_root(ginseng::database& db, EntityID eid, const std::string& frame_name)
{
  // Get the TransformComponent for this entity
  const auto* tfc = db.get_component<components::TransformComponent*>(eid);
  if (!tfc)
    throw std::runtime_error("Entity missing TransformComponent.");

  const auto& tmap = tfc->transform_map;

  // Helper lambda: find frame index by name
  auto find_frame_index = [&](const std::string& name) -> int {
    for (size_t i = 0; i < tmap.size(); ++i)
      if (tmap[i].first == name)
        return static_cast<int>(i);
    return -1;
  };

  // Start with the requested frame
  int idx = find_frame_index(frame_name);
  if (idx == -1)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");

  // Compose transforms from root to the desired frame
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // Walk the parent chain from root (index 0) up to the requested frame
  std::vector<int> chain;
  int current = idx;
  while (current != 0)  // 0 is always root
  {
    chain.push_back(current);
    const std::string& parent = tmap[current].second.parent;
    int parent_idx = find_frame_index(parent);
    if (parent_idx == -1)
      throw std::runtime_error("Parent frame '" + parent + "' not found in entity.");
    current = parent_idx;
  }
  // Walk from root (not included) to child in forward order
  for (auto it = chain.rbegin(); it != chain.rend(); ++it)
  {
    T = T * tmap[*it].second.local;
  }
  return T;
}

/**
 * @brief Resolve the parent entity ID for the root/origin transform in each TransformComponent.
 *
 * For each TransformComponent, sets tf.parent_ent_id to the EntityID of its parent object,
 * based on the origin's parent string. Throws if the parent is not found.
 */
void resolve_transform_parent_entities(ginseng::database& db, const ObjectEntityMap& obj_ent_map)
{
  db.visit([&](ginseng::database::ent_id id, components::TransformComponent& tf) {
    if (tf.transform_map.empty())
      return;

    // Assume first entry is always the "root"/origin
    auto& origin = tf.transform_map[0].second;

    if (origin.parent.empty())
    {
      // No parent: world/root
      tf.parent_ent_id = std::optional<EntityID>();
    }
    else
    {
      // Look up parent in the object id -> EntityID map
      auto it = obj_ent_map.find(origin.parent);
      if (it == obj_ent_map.end())
      {
        throw std::runtime_error("resolveParentIDTransform: Could not find parent entity for frame '" + origin.parent +
                                 "' (required by origin/root of object).");
      }
      tf.parent_ent_id = it->second;
    }
  });
}

void align_origin_transforms(ginseng::database& db, const ObjectEntityMap& obj_ent_map)
{
  db.visit([&](ginseng::database::ent_id id, components::TransformComponent& tf, components::OriginComponent& origin) {
    if (tf.transform_map.empty())
      throw std::runtime_error("Cannot set align Origin component if not transforms exists");

    std::visit(
        [&](const auto& variant) {
          using T = std::decay_t<decltype(variant)>;

          if constexpr (std::is_same_v<T, sodf::geometry::Transform>)
          {
            auto& origin = tf.transform_map[0].second;

            origin.local = variant.tf;
            origin.parent = variant.parent;
            origin.dirty = true;
          }
          else if constexpr (std::is_same_v<T, sodf::geometry::AlignFrames>)
          {
            auto it = obj_ent_map.find(variant.target_id);
            if (it == obj_ent_map.end())
              throw std::runtime_error("AlignFrames: Target object id '" + variant.target_id + "' not found.");

            EntityID target_eid = it->second;

            // Get local source frame in this entity and target frame in target entity
            const auto& src_frame = get_local_to_root(db, id, variant.source_tf);          // stylus/tip
            const auto& tgt_frame = get_local_to_root(db, target_eid, variant.target_tf);  // adapter/stylus

            auto& origin = tf.transform_map[0].second;

            origin.local = tgt_frame.inverse() * src_frame;
            origin.parent = variant.target_id;
            origin.dirty = true;
          }
          else if constexpr (std::is_same_v<T, sodf::geometry::AlignPairFrames>)
          {
            auto it = obj_ent_map.find(variant.target_id);
            if (it == obj_ent_map.end())
              throw std::runtime_error("AlignFrames: Target object id '" + variant.target_id + "' not found.");

            EntityID target_eid = it->second;

            // Get source frame in this entity and target frame in target entity
            const auto& src1_frame = get_local_to_root(db, id, variant.source_tf1);  // stylus/tip
            const auto& src2_frame = get_local_to_root(db, id, variant.source_tf2);  // stylus/tip
            const auto& tgt1_frame = get_local_to_root(db, id, variant.target_tf1);  // stylus/tip
            const auto& tgt2_frame = get_local_to_root(db, id, variant.target_tf2);  // stylus/tip

            // Variant.source frames to target frames
            auto tf_align = geometry::alignCenterFrames(tgt1_frame,  //
                                                        tgt2_frame,  //
                                                        src1_frame,  //
                                                        src2_frame,  //
                                                        variant.tolerance);

            auto& origin = tf.transform_map[0].second;

            origin.local = tf_align;
            origin.parent = variant.target_id;
            origin.dirty = true;
          }
          else
          {
            throw std::runtime_error("Origin variant not defined in visit");
          }
        },
        origin.origin);
  });
}

// Returns the EntityID of the unique root/global entity.
// Throws if there is ambiguity (multiple tags or multiple roots).
std::optional<ginseng::database::ent_id> find_root_frame_entity(ginseng::database& db)
{
  std::vector<ginseng::database::ent_id> tagged;
  std::vector<ginseng::database::ent_id> roots;

  // Find all entities with GlobalFrameTag
  std::size_t ctr = 0;
  db.visit([&](ginseng::database::ent_id id, components::RootFrameTag const&) {
    if (ctr > 0)
      throw std::runtime_error("Multiple entities with GlobalFrameTag found!");

    tagged.push_back(id);
    ctr++;
  });

  // No tag: find all entities with TransformComponent but no parent
  db.visit([&](ginseng::database::ent_id id, components::TransformComponent const& tf) {
    if (!tf.parent_ent_id.has_value())
      roots.push_back(id);
  });

  if (roots.size() > 1)
    throw std::runtime_error("Multiple root entities (no parent_ent_id) found!");
  if (!roots.empty())
    return roots.front();

  return std::nullopt;  // No root found
}

void update_global_transform(ginseng::database& db, ginseng::database::ent_id id, components::TransformComponent& tf,
                             size_t frame_idx, std::optional<ginseng::database::ent_id> global_root)
{
  // FlatMap is vector<pair<string, TransformFrame>>
  auto& pair = tf.transform_map[frame_idx];
  const std::string& frame_name = pair.first;
  auto& frame = pair.second;

  // Only update if dirty
  if (!frame.dirty)
    return;

  // Update the local transform from joint if this is NOT the first frame
  // components::JointComponent* joint_comp = db.get_component<components::JointComponent*>(id);
  // if (frame_idx != 0 && !frame.is_static && joint_comp)
  // {
  //   auto joint_it = std::find_if(joint_comp->joint_map.begin(), joint_comp->joint_map.end(),
  //                                [&](const auto& joint_pair) { return joint_pair.first == frame_name; });
  //   if (joint_it != joint_comp->joint_map.end())
  //   {
  //     const auto& joint = joint_it->second;
  //     if (joint.type == components::JointType::REVOLUTE)
  //     {
  //       // Revolute: pure rotation about axis
  //       frame.local = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(joint.position, joint.axis);
  //     }
  //     else if (joint.type == components::JointType::PRISMATIC)
  //     {
  //       // Prismatic: pure translation along axis
  //       frame.local = Eigen::Translation3d(joint.position * joint.axis);
  //     }
  //     // else: FIXED, keep as is
  //   }
  // }

  components::JointComponent* joint_comp = db.get_component<components::JointComponent*>(id);
  if (frame_idx != 0 && !frame.is_static && joint_comp)
  {
    auto joint_it = std::find_if(joint_comp->joint_map.begin(), joint_comp->joint_map.end(),
                                 [&](const auto& joint_pair) { return joint_pair.first == frame_name; });
    if (joint_it != joint_comp->joint_map.end())
    {
      const auto& joint = joint_it->second;
      switch (joint.type)
      {
        case components::JointType::REVOLUTE:
        {
          // 1 DOF rotation
          Eigen::Vector3d axis = joint.axes.col(0);
          frame.local = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(joint.position[0], axis.normalized());
          break;
        }
        case components::JointType::PRISMATIC:
        {
          // 1 DOF translation
          Eigen::Vector3d axis = joint.axes.col(0);
          frame.local = Eigen::Translation3d(joint.position[0] * axis.normalized());
          break;
        }
        case components::JointType::SPHERICAL:
        {
          // 3 DOF rotation; interpret as ZYX Euler angles (yaw, pitch, roll) by convention
          double yaw = joint.position[0];    // Z
          double pitch = joint.position[1];  // Y
          double roll = joint.position[2];   // X
          Eigen::AngleAxisd rollAngle(roll, joint.axes.col(0).normalized());
          Eigen::AngleAxisd pitchAngle(pitch, joint.axes.col(1).normalized());
          Eigen::AngleAxisd yawAngle(yaw, joint.axes.col(2).normalized());
          // Apply in ZYX order: yaw * pitch * roll
          frame.local = Eigen::Translation3d(0, 0, 0) * (yawAngle * pitchAngle * rollAngle);
          break;
        }
        case components::JointType::PLANAR:
        {
          // 2 DOF translation in plane, 1 DOF rotation about plane normal (axes: 0=X, 1=Y, 2=Normal)
          double tx = joint.position[0];
          double ty = joint.position[1];
          double rz = joint.position[2];
          Eigen::Vector3d x_axis = joint.axes.col(0).normalized();
          Eigen::Vector3d y_axis = joint.axes.col(1).normalized();
          Eigen::Vector3d normal = joint.axes.col(2).normalized();
          // Local translation in plane
          Eigen::Vector3d translation = tx * x_axis + ty * y_axis;
          Eigen::AngleAxisd rot(rz, normal);
          frame.local = Eigen::Translation3d(translation) * rot;
          break;
        }
        case components::JointType::FLOATING:
        {
          // 3 DOF translation + 3 DOF rotation (ZYX Euler angles)
          Eigen::Vector3d translation = joint.position.segment<3>(0);
          double yaw = joint.position[3];
          double pitch = joint.position[4];
          double roll = joint.position[5];
          Eigen::AngleAxisd rollAngle(roll, joint.axes.col(3).normalized());
          Eigen::AngleAxisd pitchAngle(pitch, joint.axes.col(4).normalized());
          Eigen::AngleAxisd yawAngle(yaw, joint.axes.col(5).normalized());
          frame.local = Eigen::Translation3d(translation) * (yawAngle * pitchAngle * rollAngle);
          break;
        }
        case components::JointType::FIXED:
        default:
          // Do nothing (fixed, or unknown type)
          break;
      }
    }
  }

  if (frame_idx == 0)
  {
    // First frame: parent is another entity, or global root
    if (tf.parent_ent_id)
    {
      // Parent is another entity
      auto* parent_tf = db.get_component<components::TransformComponent*>(*tf.parent_ent_id);
      if (parent_tf && !parent_tf->transform_map.empty())
      {
        // Ensure parent's first frame is up-to-date
        update_global_transform(db, *tf.parent_ent_id, *parent_tf, 0, global_root);
        frame.global = parent_tf->transform_map[0].second.global * frame.local;
      }
      else
      {
        frame.global = frame.local;
      }
    }
    else if (global_root && global_root->get_index() != id.get_index())
    {
      // Parent is the global frame
      auto* root_tf = db.get_component<components::TransformComponent*>(global_root.value());
      if (root_tf && !root_tf->transform_map.empty())
      {
        update_global_transform(db, global_root.value(), *root_tf, 0, global_root);
        frame.global = root_tf->transform_map[0].second.global * frame.local;
      }
      else
      {
        frame.global = frame.local;
      }
    }
    else
    {
      // This entity is the global root, or no root found
      frame.global = frame.local;
    }
    frame.dirty = false;
  }
  else
  {
    // Internal frame: parent is another frame in the same entity
    const std::string& parent_name = frame.parent;
    auto parent_it = std::find_if(tf.transform_map.begin(), tf.transform_map.end(),
                                  [&](const auto& p) { return p.first == parent_name; });
    if (parent_it != tf.transform_map.end())
    {
      size_t parent_idx = std::distance(tf.transform_map.begin(), parent_it);
      // Recurse: update parent frame first
      update_global_transform(db, id, tf, parent_idx, global_root);
      frame.global = parent_it->second.global * frame.local;
    }
    else
    {
      frame.global = frame.local;
    }
    frame.dirty = false;
  }
}

void update_all_global_transforms(ginseng::database& db)
{
  auto global_root = find_root_frame_entity(db);

  // 1. Update global frame entity first (if it exists)
  if (global_root)
  {
    auto* root_tf = db.get_component<components::TransformComponent*>(*global_root);
    if (root_tf)
    {
      for (size_t i = 0; i < root_tf->transform_map.size(); ++i)
        update_global_transform(db, *global_root, *root_tf, i, global_root);
    }
  }

  // 2. Update all other entities
  db.visit([&](ginseng::database::ent_id id, components::TransformComponent& tf) {
    if (global_root && id == *global_root)
      return;  // Already done

    for (size_t i = 0; i < tf.transform_map.size(); ++i)
      update_global_transform(db, id, tf, i, global_root);
  });
}

/**
 * @brief Returns the global transform of a named frame in a given entity.
 *
 * @param db         The ECS database.
 * @param eid        The entity ID.
 * @param frame_name The name of the frame (e.g. "top", "button/power").
 * @return           The Eigen::Isometry3d global pose of the frame.
 */
inline Eigen::Isometry3d get_global_transform(ginseng::database& db, EntityID eid, const std::string& frame_name)
{
  auto* tfc = db.get_component<components::TransformComponent*>(eid);
  if (!tfc)
    throw std::runtime_error("Entity missing TransformComponent.");

  const auto& tmap = tfc->transform_map;
  for (const auto& pair : tmap)
    if (pair.first == frame_name)
      return pair.second.global;

  throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");
}

/**
 * @brief Returns the global transform for a frame given a single absolute path string.
 *        E.g. "/objectid/transformname" (leading slash required).
 *
 * @param db         The ECS database.
 * @param abs_path   Absolute path string: "/objectid/framename"
 * @return           The Eigen::Isometry3d global pose of the frame.
 */
inline Eigen::Isometry3d get_global_transform(ginseng::database& db, const std::string& abs_path)
{
  // Remove leading slash and split at the next slash
  if (abs_path.empty() || abs_path[0] != '/')
    throw std::runtime_error("get_global_transform: path must start with '/'");

  size_t slash_pos = abs_path.find('/', 1);
  if (slash_pos == std::string::npos)
    throw std::runtime_error("get_global_transform: path must be of the form '/objectid/framename'");

  std::string object_id = abs_path.substr(1, slash_pos - 1);
  std::string frame_name = abs_path.substr(slash_pos + 1);

  // Lookup entity
  ObjectEntityMap obj_ent_map = make_object_entity_map(db);
  auto it = obj_ent_map.find(object_id);
  if (it == obj_ent_map.end())
    throw std::runtime_error("get_global_transform: object id '" + object_id + "' not found");

  return get_global_transform(db, it->second, frame_name);
}

}  // namespace systems
}  // namespace sodf
