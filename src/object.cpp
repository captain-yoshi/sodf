#include <sodf/object.h>

namespace sodf {

Object::Object(const geometry::Transform& tf, const geometry::Mesh& mesh) : tf_(tf), mesh_(mesh)
{
}

bool Object::addElement(const std::string& id, Element::pointer&& element)
{
  if (!element)
    return false;

  auto node = elements_.find(id);
  if (node != elements_.end())
    return false;

  // populate tree
  if (!element->addFramesToTree(element_tree_))
    return false;

  // add to map
  elements_.emplace(id, std::move(element));

  return true;
}

bool Object::removeElement(const std::string& id)
{
  auto node = elements_.extract(id);
  if (node.empty())
    return false;

  auto cls = std::move(node.mapped());
  return true;
}

const KDL::Tree& Object::elementTree() const
{
  return element_tree_;
}

const geometry::Mesh& Object::mesh() const
{
  return mesh_;
}

const geometry::Transform& Object::tf() const
{
  return tf_;
}

}  // namespace sodf
