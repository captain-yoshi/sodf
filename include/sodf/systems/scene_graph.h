#ifndef SODF_SYSTEM_SCENE_GRAPH_H_
#define SODF_SYSTEM_SCENE_GRAPH_H_

#include <iostream>
#include <unordered_map>
#include <string>
#include <sodf/ecs.h>

#include <sodf/components/joint.h>
#include <sodf/components/transform.h>
#include <sodf/components/object.h>

#include <sodf/geometry/utilities.h>

namespace sodf {
namespace systems {

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

// Helper: get TransformComponent and frame by name
// const components::TransformFrame& get_frame(ginseng::database& db, EntityID eid, const std::string& frame_name)
// {
//   const auto* tfc = db.get_component<components::TransformComponent*>(eid);
//   if (!tfc)
//     throw std::runtime_error("Entity missing TransformComponent.");
//   const auto& tmap = tfc->transform_map;

//   for (const auto& pair : tmap)
//   {
//     if (pair.first == frame_name)
//       return pair.second;
//   }
//   throw std::runtime_error("Frame '" + frame_name + "' not found in entity.");
// }

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
ObjectEntityMap make_object_entity_map(ginseng::database& db)
{
  ObjectEntityMap obj_ent_map;
  db.visit([&](EntityID id, const components::ObjectComponent& object) { obj_ent_map.insert({ object.id, id }); });
  return obj_ent_map;
}

/**
 * @brief Resolve the parent entity ID for the root/origin transform in each TransformComponent.
 *
 * For each TransformComponent, sets tf.parent_ent_id to the EntityID of its parent object,
 * based on the origin's parent string. Throws if the parent is not found.
 */
void resolveParentEntitiesForTransform(ginseng::database& db, const ObjectEntityMap& obj_ent_map)
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

  // db.visit(
  //     [&](ginseng::database::ent_id id, components::TransformComponent& tf, components::AlignFramesComponent& align) {
  //       if (tf.transform_map.empty())
  //         return;

  //       // Assume first entry is always the "root"/origin
  //       auto& origin = tf.transform_map[0].second;

  //       if (align.target_id.empty())
  //       {
  //         // No parent: world/root
  //         tf.parent_ent_id = std::optional<EntityID>();
  //       }
  //       else
  //       {
  //         // Look up parent in the object id -> EntityID map
  //         auto it = obj_ent_map.find(align.target_id);
  //         if (it == obj_ent_map.end())
  //         {
  //           throw std::runtime_error("resolveParentIDTransform: Could not find parent entity for frame '" +
  //                                    origin.parent + "' (required by origin/root of object).");
  //         }
  //         tf.parent_ent_id = it->second;
  //       }
  //     });
  // db.visit([&](ginseng::database::ent_id id, components::TransformComponent& tf,
  //              components::AlignGeometricPairComponent& align) {
  //   if (tf.transform_map.empty())
  //     return;

  //   // Assume first entry is always the "root"/origin
  //   auto& origin = tf.transform_map[0].second;

  //   if (align.target_id.empty())
  //   {
  //     // No parent: world/root
  //     tf.parent_ent_id = std::optional<EntityID>();
  //   }
  //   else
  //   {
  //     // Look up parent in the object id -> EntityID map
  //     auto it = obj_ent_map.find(align.target_id);
  //     if (it == obj_ent_map.end())
  //     {
  //       throw std::runtime_error("resolveParentIDTransform: Could not find parent entity for frame '" + origin.parent +
  //                                "' (required by origin/root of object).");
  //     }
  //     tf.parent_ent_id = it->second;
  //   }
  // });
}

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
void resolveOriginComponents(ginseng::database& db, const ObjectEntityMap& obj_ent_map)
{
  db.visit(
      [&](ginseng::database::ent_id id, components::TransformComponent& tf, components::AlignFramesComponent& align) {
        if (tf.transform_map.empty())
          return;

        auto it = obj_ent_map.find(align.target_id);
        if (it == obj_ent_map.end())
          throw std::runtime_error("AlignFrames: Target object id '" + align.target_id + "' not found.");

        EntityID target_eid = it->second;

        // Get local source frame in this entity and target frame in target entity
        const auto& src_frame = get_local_to_root(db, id, align.source);          // stylus/tip
        const auto& tgt_frame = get_local_to_root(db, target_eid, align.target);  // adapter/stylus

        auto& origin = tf.transform_map[0].second;

        origin.local = tgt_frame.inverse() * src_frame;
        origin.parent = align.target_id;
        origin.dirty = true;
      });

  db.visit([&](ginseng::database::ent_id id, components::TransformComponent& tf,
               components::AlignGeometricPairComponent& align) {
    if (tf.transform_map.empty())
      return;

    auto it = obj_ent_map.find(align.target_id);
    if (it == obj_ent_map.end())
      throw std::runtime_error("AlignFrames: Target object id '" + align.target_id + "' not found.");

    EntityID target_eid = it->second;

    // Get source frame in this entity and target frame in target entity
    const auto& src1_frame = get_local_to_root(db, id, align.source1);  // stylus/tip
    const auto& src2_frame = get_local_to_root(db, id, align.source2);  // stylus/tip
    const auto& tgt1_frame = get_local_to_root(db, id, align.target1);  // stylus/tip
    const auto& tgt2_frame = get_local_to_root(db, id, align.target2);  // stylus/tip

    // Align source frames to target frames
    auto tf_align = geometry::alignCenterFrames(tgt1_frame,  //
                                                tgt2_frame,  //
                                                src1_frame,  //
                                                src2_frame,  //
                                                align.tolerance);

    auto& origin = tf.transform_map[0].second;

    origin.local = tf_align;
    origin.parent = align.target_id;
    origin.dirty = true;
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
  components::JointComponent* joint_comp = db.get_component<components::JointComponent*>(id);
  if (frame_idx != 0 && !frame.is_static && joint_comp)
  {
    auto joint_it = std::find_if(joint_comp->joint_map.begin(), joint_comp->joint_map.end(),
                                 [&](const auto& joint_pair) { return joint_pair.first == frame_name; });
    if (joint_it != joint_comp->joint_map.end())
    {
      const auto& joint = joint_it->second;
      if (joint.type == components::JointType::REVOLUTE)
      {
        // Revolute: pure rotation about axis
        frame.local = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(joint.position, joint.axis);
      }
      else if (joint.type == components::JointType::PRISMATIC)
      {
        // Prismatic: pure translation along axis
        frame.local = Eigen::Translation3d(joint.position * joint.axis);
      }
      // else: FIXED, keep as is
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

// void addSegment(KDL::Tree& tree, const std::string& ref_segment_name, const std::string& segment_name,
//                 const KDL::Joint& joint, const KDL::Frame& tip_frame)
// {
//   if (!tree.addSegment(KDL::Segment(segment_name, joint, tip_frame), ref_segment_name))
//     throw std::runtime_error("Reference segment '" + ref_segment_name + "' does not exists in the element tree.");
//   else
//   {
//     std::cout << "Segment created: " << segment_name << std::endl;
//   }
// }

// void splitFrameId(const std::string& id, std::string& abs, std::string& rel, const std::string& delimiter)
// {
//   abs = id.substr(0, id.find(delimiter));
//   rel = (id.size() == abs.size()) ? "" : id.substr(abs.size() + delimiter.size(), -1);
// }

// class SceneGraph
// { .
// public:
//   SceneGraph(ginseng::database& db) : db(db)
//   {
//     tree = std::make_shared<KDL::Tree>("_root");

//     // HOTFIX to circumvent bug in kdl, when there are no joints
//     // Joint MUST be different then Joint::None
//     joints = std::make_shared<KDL::JntArray>(1);
//     addSegment(*tree, "_root", "root", KDL::Joint(KDL::Joint::RotX), KDL::Frame());

//     // 1- Contraints
//     db.visit([&](ginseng::database::ent_id eid, const components::ID& id, const components::Constraint& c,
//                  const components::Transforms& transforms) {
//       const auto& tf1 = c.transform;

//       KDL::Frame kdl_frame;
//       tf::transformEigenToKDL(tf1.frame, kdl_frame);

//       // create absolute id
//       std::string segment_name = id.id + delimiter + tf1.child;

//       std::string abs, rel;
//       splitFrameId(segment_name, abs, rel, delimiter);

//       if (rel == "root")
//       {
//         addSegment(*tree, tf1.parent, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);
//       }
//       else
//       {
//         // find rel in child
//         for (const auto& tf2 : transforms.transforms)
//         {
//           if (rel == tf2.child)
//           {
//             tf::transformEigenToKDL(tf1.frame * tf2.frame.inverse(), kdl_frame);
//             addSegment(*tree, tf1.parent, id.id + "/root", KDL::Joint(KDL::Joint::None), kdl_frame);
//             break;
//           }
//         }
//       }
//     });

//     // 2- Joints
//     db.visit(
//         [&](ginseng::database::ent_id eid, const components::ID& id, const components::JointCollection& collection) {
//           // Add every components frame, start with joint
//           for (const auto& joint : collection.joint_map)
//           {
//             // Add temp segment, joints are located at the root of the specified frame
//             std::string segment_name = id.id + delimiter + "_" + joint.first;

//             std::string ref_name = id.id + delimiter + joint.second.parent;
//             KDL::Frame kdl_frame;
//             tf::transformEigenToKDL(joint.second.frame, kdl_frame);

//             addSegment(*tree, ref_name, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);

//             // Add real joint
//             ref_name = segment_name;
//             segment_name = id.id + delimiter + joint.first;

//             joints->resize(joints->data.size() + 1);

//             addSegment(*tree, ref_name, segment_name, joint.second.joint, KDL::Frame());
//           }
//         });

//     // 3- Transforms
//     db.visit([&](ginseng::database::ent_id eid, const components::ID& id, const components::Transforms& transforms) {
//       for (const auto& tf : transforms.transforms)
//       {
//         KDL::Frame kdl_frame;
//         tf::transformEigenToKDL(tf.frame, kdl_frame);

//         std::string ref_name = id.id + delimiter + tf.parent;
//         std::string segment_name = id.id + delimiter + tf.child;

//         addSegment(*tree, ref_name, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);
//       }
//     });

//     fk_solver.reset(new KDL::TreeFkSolverPos_recursive(*tree));
//   }

//   Eigen::Isometry3d transformFrame(const std::string& from, const std::string& to)
//   {
//     // update joint positions
//     updateJointPositions();

//     KDL::Frame kdl_from;
//     KDL::Frame kdl_to;

//     auto rc = fk_solver->JntToCart(*joints, kdl_from, from);
//     if (rc < 0)
//       throw std::runtime_error("forward kinematic solver internal error with frame " + from);

//     rc = fk_solver->JntToCart(*joints, kdl_to, to);
//     if (rc < 0)
//       throw std::runtime_error("forward kinematic solver internal error with frame " + to);

//     Eigen::Isometry3d eig_from;
//     Eigen::Isometry3d eig_to;

//     tf::transformKDLToEigen(kdl_from, eig_from);
//     tf::transformKDLToEigen(kdl_to, eig_to);

//     return eig_to.inverse() * eig_from;
//   }

//   Eigen::Isometry3d displayInObjectRoot(const std::string& id, const std::string& rel_frame_name)
//   {
//     return transformFrame(id + delimiter + rel_frame_name, id + delimiter + "root");
//   }

//   void updateJointPositions()
//   {
//     // do not update virtual joint -> index = 0
//     std::size_t joint_index = 1;

//     db.visit([&](ginseng::database::ent_id eid, const components::JointCollection& collection) {
//       if (collection.joint_map.size() + joint_index > joints->data.size())
//         throw std::runtime_error("JointCollection components exceed joint size in scene graph");

//       for (const auto& joint : collection.joint_map)
//         joints->data[joint_index++] = joint.second.joint_position;
//     });
//   }

// private:
//   std::shared_ptr<KDL::Tree> tree;
//   std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver;
//   std::shared_ptr<KDL::JntArray> joints;

//   std::string delimiter = "/";

//   ginseng::database& db;
// };

}  // namespace systems
}  // namespace sodf

#endif
