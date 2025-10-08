#pragma once
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <tinyxml2.h>

#include <sodf/xml/macros.h>

namespace sodf {
namespace xml {

// One scene "component" (e.g., <Origin id="root">, <Link id="link/base">, ...)
struct SceneComponent
{
  SceneComponentType type;          // e.g., "Link", "Button"
  std::string id;                   // e.g., "link/base"
  const tinyxml2::XMLElement* xml;  // points to current element root

  // If we ever need to mutate it, we deep-clone into this document and
  // redirect 'xml' to point into that doc to keep lifetime stable.
  std::unique_ptr<tinyxml2::XMLDocument> owned_doc;
};

// A logical <Object> realized as a set of components (post-overlays/patches)
struct SceneObject
{
  std::string id;
  std::vector<SceneComponent> components;
  std::set<std::string> remove_ids;  // currently unused in patch ops, but kept for parity
};

}  // namespace xml
}  // namespace sodf
