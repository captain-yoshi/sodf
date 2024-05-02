#ifndef OBJECT_H_
#define OBJECT_H_

#include <map>
#include <string>
#include <iostream>

#include <kdl/tree.hpp>

#include <sodf/element.h>
#include <sodf/geometry/mesh.h>

namespace sodf {

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter = "/");

class Object
{
public:
  Object(const geometry::Transform& tf, const geometry::Mesh& mesh = geometry::Mesh());

  /// Elements
  bool addElement(const std::string& id, Element::pointer&& element);
  bool removeElement(const std::string& id);

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

private:
  const geometry::Transform tf_;  // parent to this object transform

  KDL::Tree element_tree_;  // root name is "root"

  const geometry::Mesh mesh_;

  std::map<std::string, Element::pointer> elements_;
};

}  // namespace sodf

#endif  // OBJECT_H_
