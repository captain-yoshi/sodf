#ifndef SODF_SYSTEM_SCENE_GRAPH_H_
#define SODF_SYSTEM_SCENE_GRAPH_H_

#include <iostream>

#include <ginseng.hpp>

#include <sodf/components/constraint.h>
#include <sodf/components/id.h>
#include <sodf/components/joint.h>
#include <sodf/components/container.h>
#include <sodf/components/direct_kinematic.h>

#include <eigen_conversions/eigen_kdl.h>

namespace sodf {
namespace systems {

void addSegment(KDL::Tree& tree, const std::string& ref_segment_name, const std::string& segment_name,
                const KDL::Joint& joint, const KDL::Frame& tip_frame)
{
  if (!tree.addSegment(KDL::Segment(segment_name, joint, tip_frame), ref_segment_name))
    throw std::runtime_error("Reference segment '" + ref_segment_name + "' does not exists in the element tree.");
  else
  {
    std::cout << "Segment created: " << segment_name << std::endl;
  }
}

void splitFrameId(const std::string& id, std::string& abs, std::string& rel, const std::string& delimiter)
{
  abs = id.substr(0, id.find(delimiter));
  rel = (id.size() == abs.size()) ? "" : id.substr(abs.size() + delimiter.size(), -1);
}

class SceneGraph
{
public:
  SceneGraph(ginseng::database& db) : db(db)
  {
    tree = std::make_shared<KDL::Tree>("_root");

    // HOTFIX to circumvent bug in kdl, when there are no joints
    // Joint MUST be different then Joint::None
    joints = std::make_shared<KDL::JntArray>(1);
    addSegment(*tree, "_root", "root", KDL::Joint(KDL::Joint::RotX), KDL::Frame());

    // 1- Contraints
    db.visit([&](ginseng::database::ent_id eid, const components::ID& id, const components::Constraint& c,
                 const components::Transforms& transforms) {
      const auto& tf1 = c.transform;

      KDL::Frame kdl_frame;
      tf::transformEigenToKDL(tf1.frame, kdl_frame);

      // create absolute id
      std::string segment_name = id.id + delimiter + tf1.child;

      std::string abs, rel;
      splitFrameId(segment_name, abs, rel, delimiter);

      if (rel == "root")
      {
        addSegment(*tree, tf1.parent, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);
      }
      else
      {
        // find rel in child
        for (const auto& tf2 : transforms.transforms)
        {
          if (rel == tf2.child)
          {
            tf::transformEigenToKDL(tf1.frame * tf2.frame.inverse(), kdl_frame);
            addSegment(*tree, tf1.parent, id.id + "/root", KDL::Joint(KDL::Joint::None), kdl_frame);
            break;
          }
        }
      }
    });

    // 2- Joints
    db.visit(
        [&](ginseng::database::ent_id eid, const components::ID& id, const components::JointCollection& collection) {
          // Add every components frame, start with joint
          for (const auto& joint : collection.joint_map)
          {
            // Add temp segment, joints are located at the root of the specified frame
            std::string segment_name = id.id + delimiter + "_" + joint.first;

            std::string ref_name = id.id + delimiter + joint.second.parent;
            KDL::Frame kdl_frame;
            tf::transformEigenToKDL(joint.second.frame, kdl_frame);

            addSegment(*tree, ref_name, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);

            // Add real joint
            ref_name = segment_name;
            segment_name = id.id + delimiter + joint.first;

            joints->resize(joints->data.size() + 1);

            addSegment(*tree, ref_name, segment_name, joint.second.joint, KDL::Frame());
          }
        });

    // 3- Transforms
    db.visit([&](ginseng::database::ent_id eid, const components::ID& id, const components::Transforms& transforms) {
      for (const auto& tf : transforms.transforms)
      {
        KDL::Frame kdl_frame;
        tf::transformEigenToKDL(tf.frame, kdl_frame);

        std::string ref_name = id.id + delimiter + tf.parent;
        std::string segment_name = id.id + delimiter + tf.child;

        addSegment(*tree, ref_name, segment_name, KDL::Joint(KDL::Joint::None), kdl_frame);
      }
    });

    fk_solver.reset(new KDL::TreeFkSolverPos_recursive(*tree));
  }

  Eigen::Isometry3d transformFrame(const std::string& from, const std::string& to)
  {
    // update joint positions
    updateJointPositions();

    KDL::Frame kdl_from;
    KDL::Frame kdl_to;

    auto rc = fk_solver->JntToCart(*joints, kdl_from, from);
    if (rc < 0)
      throw std::runtime_error("forward kinematic solver internal error with frame " + from);

    rc = fk_solver->JntToCart(*joints, kdl_to, to);
    if (rc < 0)
      throw std::runtime_error("forward kinematic solver internal error with frame " + to);

    Eigen::Isometry3d eig_from;
    Eigen::Isometry3d eig_to;

    tf::transformKDLToEigen(kdl_from, eig_from);
    tf::transformKDLToEigen(kdl_to, eig_to);

    return eig_to.inverse() * eig_from;
  }

  Eigen::Isometry3d displayInObjectRoot(const std::string& id, const std::string& rel_frame_name)
  {
    return transformFrame(id + delimiter + rel_frame_name, id + delimiter + "root");
  }

  void updateJointPositions()
  {
    // do not update virtual joint -> index = 0
    std::size_t joint_index = 1;

    db.visit([&](ginseng::database::ent_id eid, const components::JointCollection& collection) {
      if (collection.joint_map.size() + joint_index > joints->data.size())
        throw std::runtime_error("JointCollection components exceed joint size in scene graph");

      for (const auto& joint : collection.joint_map)
        joints->data[joint_index++] = joint.second.joint_position;
    });
  }

private:
  std::shared_ptr<KDL::Tree> tree;
  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::JntArray> joints;

  std::string delimiter = "/";

  ginseng::database& db;
};

}  // namespace systems
}  // namespace sodf

#endif
