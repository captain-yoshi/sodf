#ifndef OBJECT_H_
#define OBJECT_H_

#include <map>
#include <string>
#include <iostream>

#include <kdl/tree.hpp>

#include <sodf/element.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <geometry_msgs/InertiaStamped.h>

namespace sodf {

class Object;

using ObjectPtr = std::shared_ptr<Object>;
using ObjectID = std::string;
using ObjectMap = std::map<std::string, ObjectPtr, std::less<void>>;

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter = "/");

void splitObjectElement(const std::string& id, std::string& object, std::string& element,
                        const std::string& delimiter = "/");

Eigen::Isometry3d getRootToElementFrameEigen(const ObjectMap& object_map, const ObjectID& object_id,
                                             const ElementID& element_id);
geometry_msgs::Pose getRootToElementFramePoseMsg(const ObjectMap& object_map, const ObjectID& object_id,
                                                 const ElementID& element_id);

class Object
{
public:
  Object(const ObjectID& id);
  Object(const ObjectID& id, const geometry::Transform& tf);

  /// Elements
  bool addElement(const ElementID& id, Element::pointer&& element);
  bool removeElement(const std::string& id);

  /// child/parent
  void addParent(ObjectPtr parent);  // Overwrite if exists
  void addChildren(ObjectPtr child);
  void removeParent();
  void removeChildren(ObjectPtr child);
  const std::set<ObjectPtr>& children() const;

  const ObjectID& id() const;
  ObjectPtr parent();

  void init();

  template <typename T>
  T* getElement(const std::string& id)
  {
    static_assert(std::is_base_of<Element, T>::value, "T should inherit from sodf::Element");

    auto node = elements_.find(id);
    if (node == elements_.end())
      return nullptr;

    return dynamic_cast<T*>(node->second.get());
  };

  template <typename T>
  const T* getElement(const std::string& id) const
  {
    static_assert(std::is_base_of<Element, T>::value, "T should inherit from sodf::Element");

    const auto node = elements_.find(id);
    if (node == elements_.end())
      return nullptr;

    return dynamic_cast<const T*>(node->second.get());
  };

  const std::map<std::string, Element::pointer>& getElements()
  {
    return elements_;
  }

  /// Mesh
  const std::vector<ElementID>& meshes() const;

  /** Compute the inertia of all the meshes wrt. the root frame
   *
   * The frame_id is set to the id of this object.
   */
  geometry_msgs::InertiaStamped inertia() const;

  /// Tf
  const geometry::Transform& tf() const;

  KDL::Frame displayInRoot(const std::string& from) const;
  Eigen::Isometry3d displayInRootEigen(const std::string& from) const;
  geometry_msgs::Pose displayInRootPoseMsg(const std::string& from) const;

  void setTransform(const geometry::Transform& tf)
  {
    tf_ = tf;
  }

  bool updateJointPosition(const std::string& element_id, double position);
  bool updateJointPosition(const std::string& element_id, const std::string& position);

private:
  void initializeTree();

  const ObjectID id_;

  ObjectPtr parent_;
  std::set<ObjectPtr> childrens_;

  std::map<std::string, std::size_t> joint_index_map_;

  geometry::Transform tf_;  // parent to this object transform

  std::shared_ptr<KDL::Tree> element_tree_;  // root name is "root"

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::JntArray> joints_;

  std::vector<ElementID> meshes_;

  std::map<std::string, Element::pointer> elements_;
};

static void addChildParentRelationship(sodf::ObjectPtr& child, sodf::ObjectPtr& parent)
{
  child->addParent(parent);
  parent->addChildren(child);
}

static void removeChildParentRelationship(sodf::ObjectPtr& child, sodf::ObjectPtr& parent)
{
  child->removeParent();
  parent->removeChildren(child);
}

}  // namespace sodf

#endif  // OBJECT_H_
