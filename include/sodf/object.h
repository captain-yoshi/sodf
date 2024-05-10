#ifndef OBJECT_H_
#define OBJECT_H_

#include <map>
#include <string>
#include <iostream>

#include <kdl/tree.hpp>

#include <sodf/element.h>

#include <kdl/treefksolverpos_recursive.hpp>

namespace sodf {

class Object;

using ObjectPtr = std::shared_ptr<Object>;
using ObjectID = std::string;
using ObjectMap = std::map<std::string, ObjectPtr, std::less<void>>;

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter = "/");

void splitObjectElement(const std::string& id, std::string& object, std::string& element,
                        const std::string& delimiter = "/");

class Object
{
public:
  Object(const ObjectID& id, const geometry::Transform& tf);

  /// Elements
  bool addElement(const ElementID& id, Element::pointer&& element);
  bool removeElement(const std::string& id);

  /// child/parent
  void addParent(ObjectPtr parent);  // Overwrite if exists
  void addChildren(ObjectPtr child);
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

  /// Mesh
  const std::vector<ElementID>& meshes() const;

  /// Tf
  const geometry::Transform& tf() const;

  KDL::Frame displayInRoot(const std::string& from) const;
  Eigen::Isometry3d displayInRootEigen(const std::string& from) const;
  geometry_msgs::Pose displayInRootPoseMsg(const std::string& from) const;

  bool updateJointPosition(const std::string& element_id, double position);
  bool updateJointPosition(const std::string& element_id, const std::string& position);

private:
  const ObjectID id_;

  ObjectPtr parent_;
  std::set<ObjectPtr> childrens_;

  std::map<std::string, std::size_t> joint_index_map_;

  const geometry::Transform tf_;  // parent to this object transform

  std::shared_ptr<KDL::Tree> element_tree_;  // root name is "root"

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::JntArray> joints_;

  std::vector<ElementID> meshes_;

  std::map<std::string, Element::pointer> elements_;
};

}  // namespace sodf

#endif  // OBJECT_H_
