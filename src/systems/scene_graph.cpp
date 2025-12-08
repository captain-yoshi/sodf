#include <sodf/systems/scene_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sodf {
namespace systems {

// -----------------------------------------------------------------------------
// Database-only global-cache implementation
// -----------------------------------------------------------------------------

Eigen::Isometry3d get_root_global_transform(database::Database& db, const database::ObjectEntityMap& obj_map,
                                            const std::string& object_id)
{
  auto it = obj_map.find(object_id);
  if (it == obj_map.end())
    throw std::runtime_error("[scene_graph] get_root_global_transform: object id '" + object_id + "' not found");

  const database::EntityID eid = it->second;

  update_entity_global_transforms(db, eid, obj_map);

  auto* tf = db.get<components::TransformComponent>(eid);
  if (!tf || tf->elements.empty())
    throw std::runtime_error("[scene_graph] get_root_global_transform: entity has no TransformComponent or no frames");

  return tf->elements.front().second.global;
}

// Recursively update one frame’s global transform (returns true if recomputed)
static bool update_global_transform(database::Database& db, database::EntityID id, components::TransformComponent& tf,
                                    std::size_t frame_idx, const database::ObjectEntityMap& obj_map, bool force)
{
  auto& kv = tf.elements[frame_idx];
  const std::string& frame_name = kv.first;
  auto& frame = kv.second;

  bool parent_recomputed = false;
  Eigen::Isometry3d parent_global = Eigen::Isometry3d::Identity();

  if (frame_idx == 0)
  {
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
          parent_recomputed = update_global_transform(db, parent_eid, *ptf, 0, obj_map, /*force=*/false);
          parent_global = ptf->elements[0].second.global;
        }
      }
    }
  }
  else
  {
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

  const bool need_recompute = force || frame.dirty || parent_recomputed;
  if (!need_recompute)
    return false;

  if (frame_idx != 0 && !frame.is_static)
  {
    if (auto* joints = db.get<components::JointComponent>(id))
    {
      if (auto* joint = database::get_element(joints->elements, frame_name))
      {
        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();

        switch (joint->type)
        {
          case components::JointType::REVOLUTE:
          {
            Eigen::Vector3d axis = joint->axes.col(0);
            if (axis.norm() > 0.0)
            {
              axis.normalize();
              delta = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(joint->position[0], axis);
            }
            break;
          }

          case components::JointType::PRISMATIC:
          {
            Eigen::Vector3d axis = joint->axes.col(0);
            if (axis.norm() > 0.0)
            {
              axis.normalize();
              delta = Eigen::Translation3d(joint->position[0] * axis);
            }
            break;
          }

          case components::JointType::SPHERICAL:
          {
            if (joint->position.size() >= 3)
            {
              const double yaw = joint->position[0];
              const double pitch = joint->position[1];
              const double roll = joint->position[2];

              Eigen::Vector3d ax0 = joint->axes.col(0).normalized();
              Eigen::Vector3d ax1 = joint->axes.col(1).normalized();
              Eigen::Vector3d ax2 = joint->axes.col(2).normalized();

              Eigen::AngleAxisd Rx(roll, ax0);
              Eigen::AngleAxisd Ry(pitch, ax1);
              Eigen::AngleAxisd Rz(yaw, ax2);

              delta = Eigen::Translation3d(0, 0, 0) * (Rz * Ry * Rx);
            }
            break;
          }

          case components::JointType::PLANAR:
          {
            if (joint->position.size() >= 3)
            {
              const double tx = joint->position[0];
              const double ty = joint->position[1];
              const double rz = joint->position[2];

              Eigen::Vector3d x = joint->axes.col(0).normalized();
              Eigen::Vector3d y = joint->axes.col(1).normalized();
              Eigen::Vector3d n = joint->axes.col(2).normalized();

              Eigen::Vector3d t = tx * x + ty * y;
              delta = Eigen::Translation3d(t) * Eigen::AngleAxisd(rz, n);
            }
            break;
          }

          case components::JointType::FLOATING:
          {
            if (joint->position.size() >= 6)
            {
              Eigen::Vector3d t = joint->position.segment<3>(0);
              const double yaw = joint->position[3];
              const double pitch = joint->position[4];
              const double roll = joint->position[5];

              Eigen::Vector3d ax3 = joint->axes.col(3).normalized();
              Eigen::Vector3d ax4 = joint->axes.col(4).normalized();
              Eigen::Vector3d ax5 = joint->axes.col(5).normalized();

              Eigen::AngleAxisd Rx(roll, ax3);
              Eigen::AngleAxisd Ry(pitch, ax4);
              Eigen::AngleAxisd Rz(yaw, ax5);

              delta = Eigen::Translation3d(t) * (Rz * Ry * Rx);
            }
            break;
          }

          case components::JointType::FIXED:
          default:
            break;
        }

        frame.local = frame.rest_local * delta;
      }
    }
  }

  frame.global = parent_global * frame.local;
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

  const bool force_subtree = tf->elements[0].second.dirty;

  for (std::size_t i = 0; i < tf->elements.size(); ++i)
    update_global_transform(db, eid, *tf, i, obj_map, /*force=*/force_subtree);
}

void update_all_global_transforms(database::Database& db)
{
  auto obj_map = make_object_entity_map(db);

  db.each([&](database::EntityID id, components::TransformComponent& tf) {
    const bool force_subtree = (!tf.elements.empty() && tf.elements[0].second.dirty);

    for (std::size_t i = 0; i < tf.elements.size(); ++i)
      update_global_transform(db, id, tf, i, obj_map, force_subtree);
  });
}

Eigen::Isometry3d get_global_transform(database::Database& db, database::EntityID eid, const std::string& frame_name)
{
  auto* tf = db.get_element<components::TransformComponent>(eid, frame_name);
  if (!tf)
    throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");
  return tf->global;
}

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
