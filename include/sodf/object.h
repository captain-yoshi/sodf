#ifndef OBJECT_H_
#define OBJECT_H_

#include <map>
#include <string>
#include <iostream>

#include <kdl/tree.hpp>

#include <sodf/element.h>
#include <sodf/geometry/mesh.h>

#include <kdl/treefksolverpos_recursive.hpp>

namespace sodf {

class Object;

using ObjectPtr = std::shared_ptr<Object>;
using ObjectID = std::string;
using ObjectMap = std::map<std::string, ObjectPtr, std::less<void>>;

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter = "/");

class Object
{
public:
  Object(const ObjectID& id, const geometry::Transform& tf, const geometry::Mesh& mesh = geometry::Mesh());

  /// Elements
  bool addElement(const std::string& id, Element::pointer&& element);
  bool removeElement(const std::string& id);

  /// child/parent
  void addParent(ObjectPtr parent);  // Overwrite if exists
  void addChildren(ObjectPtr child);
  const std::set<ObjectPtr>& getChildrens() const;

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

    return static_cast<T*>(node->second.get());
  };

  /// Element Tree
  const KDL::Tree& elementTree() const;

  /// Mesh
  const geometry::Mesh& mesh() const;

  /// Tf
  const geometry::Transform& tf() const;

  KDL::Frame displayInRoot(const std::string& from) const;
  Eigen::Isometry3d displayInRootEigen(const std::string& from) const;
  geometry_msgs::Pose displayInRootPoseMsg(const std::string& from) const;

private:
  const ObjectID id_;

  ObjectPtr parent_;
  std::set<ObjectPtr> childrens_;

  const geometry::Transform tf_;  // parent to this object transform

  KDL::Tree element_tree_;  // root name is "root"

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::JntArray> joints_;

  const geometry::Mesh mesh_;

  std::map<std::string, Element::pointer> elements_;
};

}  // namespace sodf

#endif  // OBJECT_H_
