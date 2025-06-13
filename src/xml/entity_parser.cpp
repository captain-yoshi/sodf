
#include <sodf/xml/utils.h>
#include <sodf/xml/macros.h>
#include <sodf/xml/entity_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>

#include <sodf/components/object.h>

// using namespace tinyxml2;
// using namespace sodf::components;
// using namespace sodf::geometry;

namespace sodf {
namespace xml {

// Stores all object elements with fully-qualified id
// using ObjectIndex = std::unordered_map<std::string, const tinyxml2::XMLElement*>;
struct ObjectSource
{
  const tinyxml2::XMLElement* xml;
  std::string filename;
};

using ObjectIndex = std::unordered_map<std::string, ObjectSource>;
// using ObjectIndex = std::unordered_map<std::string, std::vector<ObjectSource>>;
// using ObjectIndex = std::unordered_map<std::string, std::vector<const tinyxml2::XMLElement*>>;

struct SceneComponent
{
  SceneComponentType type;          // e.g., "Link", "Button"
  std::string id;                   // e.g., "link/base"
  const tinyxml2::XMLElement* xml;  // Pointer to XML element
};

struct SceneObject
{
  std::string id;
  std::vector<SceneComponent> components;
  std::set<std::string> remove_ids;
};
using SceneMap = std::unordered_map<std::string, SceneObject>;

std::string canonical_ns(const std::string& parent_ns, const std::string& local_ns)
{
  if (parent_ns.empty())
    return local_ns;
  if (local_ns.empty())
    return parent_ns;
  return parent_ns + "::" + local_ns;
}

std::string canonicalize_ref(const std::string& current_ns, const std::string& ref)
{
  if (ref.empty())
    return current_ns;
  // If absolute/global (starts with "::"), strip it and treat as global
  if (ref.rfind("::", 0) == 0)
    return ref.substr(2);
  // If already fully qualified with current_ns
  if (!current_ns.empty() && ref.compare(0, current_ns.size() + 2, current_ns + "::") == 0)
    return ref;
  // Otherwise, prepend current namespace
  if (!current_ns.empty())
    return current_ns + "::" + ref;
  return ref;
}

std::string extract_namespace(const std::string& qualified_id)
{
  auto pos = qualified_id.rfind("::");
  if (pos == std::string::npos)
    return "";
  return qualified_id.substr(0, pos);
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& parent_ns,
                      const std::string& base_dir, const std::string& filename);

void processInclude(const tinyxml2::XMLElement* include_elem, ObjectIndex& object_index,
                    const std::string& including_file_dir, const std::string& parent_ns)
{
  std::string path = include_elem->Attribute("path");
  std::string ns = include_elem->Attribute("ns") ? include_elem->Attribute("ns") : "";
  // std::string full_ns = parent_ns.empty() ? ns : (parent_ns.empty() ? ns : parent_ns + "::" + ns);
  std::string full_ns = canonical_ns(parent_ns, ns);

  // Resolve included file path relative to current file's directory
  std::string inc_filename = including_file_dir.empty() ? path : (including_file_dir + "/" + path);

  auto* inc_doc = new tinyxml2::XMLDocument();
  if (inc_doc->LoadFile(inc_filename.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Failed to include: " + inc_filename);

  std::string inc_file_dir = getDirectory(inc_filename);

  buildObjectIndex(inc_doc, object_index, full_ns, inc_file_dir, inc_filename);
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& ns,
                      const std::string& base_dir, const std::string& filename)
{
  const auto* root = doc->FirstChildElement("root");
  if (!root)
    return;

  // Handle <include> recursively
  for (const auto* incl = root->FirstChildElement("include"); incl; incl = incl->NextSiblingElement("include"))
  {
    processInclude(incl, object_index, base_dir, ns);
  }

  // Only one <Object> per qualified id is allowed!
  for (const auto* obj_elem = root->FirstChildElement("Object"); obj_elem;
       obj_elem = obj_elem->NextSiblingElement("Object"))
  {
    std::string id = obj_elem->Attribute("id") ? obj_elem->Attribute("id") : "";
    if (id.empty())
      continue;

    std::string qualified_id = ns.empty() ? id : (ns + "::" + id);

    if (object_index.count(qualified_id))
    {
      const ObjectSource& previous = object_index[qualified_id];
      throw std::runtime_error("Duplicate <Object> for id '" + qualified_id + "' in file '" + filename + "' at line " +
                               std::to_string(obj_elem->GetLineNum()) + ". Previous definition was at line " +
                               std::to_string(previous.xml->GetLineNum()) + " in file '" + previous.filename + "'.");
    }
    object_index[qualified_id] = ObjectSource{ obj_elem, filename };
  }
}

SceneObject composeObject(const tinyxml2::XMLElement* elem, const ObjectIndex& object_index,
                          const std::string& current_ns)
{
  SceneObject obj;
  obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";

  bool has_clone = elem->Attribute("clone");

  // --- CLONE (deep-merge) ---
  if (has_clone)
  {
    std::string clone_id = elem->Attribute("clone");
    std::string canonical_clone = canonicalize_ref(current_ns, clone_id);
    auto found = object_index.find(canonical_clone);
    if (found == object_index.end())
      throw std::runtime_error("Clone source not found: " + canonical_clone + " (line " +
                               std::to_string(elem->GetLineNum()) + ")");

    std::string cloned_ns = extract_namespace(canonical_clone);
    obj = composeObject(found->second.xml, object_index, cloned_ns);  // Recursively clone
    obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";
  }

  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    std::string tag = child->Name();
    // If inside clone, only patch operations are allowed
    if (has_clone)
    {
      if (tag == "add" || tag == "update" || tag == "upsert" || tag == "remove")
      {
        // === Patch operations ===
        for (const auto* comp = child->FirstChildElement(); comp; comp = comp->NextSiblingElement())
        {
          std::string comp_type_str = comp->Name();
          auto opt_type = sceneComponentTypeFromString(comp_type_str);
          if (!opt_type)
            throw std::runtime_error("Unknown ComponentType string: " + comp_type_str);
          SceneComponentType comp_type = *opt_type;
          std::string comp_id = comp->Attribute("id") ? comp->Attribute("id") : "";

          auto match_pred = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
          auto it = std::find_if(obj.components.begin(), obj.components.end(), match_pred);

          if (tag == "add")
          {
            if (it != obj.components.end())
            {
              throw std::runtime_error("Cannot <add> already existing component <" + comp_type_str + " id=\"" +
                                       comp_id + "\"> to <Object id=\"" + obj.id + "\"> at line " +
                                       std::to_string(comp->GetLineNum()) + ".");
            }
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "update")
          {
            if (it == obj.components.end())
            {
              throw std::runtime_error("Cannot <update> non-existent component <" + comp_type_str + " id=\"" + comp_id +
                                       "\"> in <Object id=\"" + obj.id + "\"> at line " +
                                       std::to_string(comp->GetLineNum()) + ".");
            }
            // Remove and replace
            obj.components.erase(it);
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "upsert")
          {
            // Remove if exists, then add
            obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred),
                                 obj.components.end());
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "remove")
          {
            if (it == obj.components.end())
            {
              throw std::runtime_error("Cannot <remove> non-existent component <" + comp_type_str + " id=\"" + comp_id +
                                       "\"> from <Object id=\"" + obj.id + "\"> at line " +
                                       std::to_string(comp->GetLineNum()) + ".");
            }
            obj.components.erase(it);
          }
        }
      }
      else
      {
        throw std::runtime_error(
            "Invalid child element <" + tag + "> inside <Object id=\"" + obj.id + "\" clone=\"...\"> at line " +
            std::to_string(child->GetLineNum()) +
            ". Only <add>, <update>, <upsert>, or <remove> tags are allowed as children of a cloned object.");
      }
    }
    else
    {
      // Only direct component children allowed when not cloning
      if (tag == "add" || tag == "update" || tag == "upsert" || tag == "remove")
      {
        throw std::runtime_error("Invalid child element <" + tag + "> inside non-clone <Object id=\"" + obj.id +
                                 "\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only direct component tags (e.g., <Origin>, <Link>) are allowed when not cloning.");
      }
      // Direct children act as upsert (default overlay)
      std::string comp_type_str = tag;

      auto opt_type = sceneComponentTypeFromString(comp_type_str);
      if (!opt_type)
        throw std::runtime_error("Unknown ComponentType string: " + comp_type_str);
      SceneComponentType comp_type = *opt_type;

      std::string comp_id = child->Attribute("id") ? child->Attribute("id") : "";

      auto match_pred = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
      obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred),
                           obj.components.end());
      obj.components.push_back({ comp_type, comp_id, child });
    }
  }
  return obj;
}

void parseSceneObjects(const tinyxml2::XMLDocument* doc, const ObjectIndex& object_index, SceneMap& scene)
{
  const auto* root = doc->FirstChildElement("root");
  if (!root)
    return;

  // Only create entities for objects defined in the MAIN SCENE file!
  for (const auto* obj_elem = root->FirstChildElement("Object"); obj_elem;
       obj_elem = obj_elem->NextSiblingElement("Object"))
  {
    std::string id = obj_elem->Attribute("id") ? obj_elem->Attribute("id") : "";
    if (id.empty())
      continue;

    // Find the full overlay chain for this id
    std::string qualified_id = id;  // If you support namespaces, do lookup here.
    auto it = object_index.find(qualified_id);
    if (it == object_index.end())
      throw std::runtime_error("Object '" + id + "' not found in index (line " +
                               std::to_string(obj_elem->GetLineNum()) + ")");

    // *** Extract current namespace for this object ***
    std::string current_ns = extract_namespace(it->first);

    // Compose the full object state
    SceneObject obj = composeObject(it->second.xml, object_index, current_ns);
    scene[obj.id] = std::move(obj);
  }
}

void parseComponentSubElements(const tinyxml2::XMLElement* parent, ginseng::database& db, EntityID eid)
{
  // Call every registered subcomponent parser with the parent element itself.
  for (const auto& subParseFunc : subParseFuncs)
  {
    subParseFunc(parent, db, eid);
  }
}

void parseComponents(const tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, ginseng::database& db, EntityID eid,
                     size_t parseFuncIdx)
{
  if (parseFuncIdx == ForLoopIndex)
  {
    // Expand and parse each child for each ForLoop iteration
    expandForLoop(elem, [&](const std::unordered_map<std::string, std::string>& ctx) {
      for (const tinyxml2::XMLElement* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
      {
        tinyxml2::XMLElement* clone = cloneAndSubstitute(child, doc, ctx);

        // Determine the type index of the expanded child
        std::string child_tag = clone->Name();
        auto opt_type = sceneComponentTypeFromString(child_tag);
        if (!opt_type)
          throw std::runtime_error("Unknown component type in ForLoop: <" + child_tag + ">");
        size_t child_idx = static_cast<size_t>(*opt_type);

        // Recursively handle, so nested ForLoops also work!
        parseComponents(clone, doc, db, eid, child_idx);
      }
    });
  }
  else
  {
    // Standard component: parse and handle subcomponents
    parseFuncs[parseFuncIdx](elem, db, eid);
    parseComponentSubElements(elem, db, eid);
  }
}

// Constructor to initialize filename
EntityParser::EntityParser()
{
}

EntityParser::~EntityParser()
{
}

bool EntityParser::loadEntitiesFromFile(const std::string& filename, ginseng::database& db)
{
  doc = std::make_unique<tinyxml2::XMLDocument>();
  if (doc->LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
  {
    std::cerr << "Error loading XML file: " << filename << std::endl;
    return false;
  }
  std::string base_dir = std::filesystem::path(filename).parent_path().string();
  return loadEntities(doc.get(), base_dir, db);
}

bool EntityParser::loadEntitiesFromText(const std::string& text, ginseng::database& db, const std::string& base_dir)
{
  doc = std::make_unique<tinyxml2::XMLDocument>();
  if (doc->Parse(text.c_str()) != tinyxml2::XML_SUCCESS)
  {
    std::cerr << "Error parsing XML text" << std::endl;
    return false;
  }
  // base_dir can be "" (no includes) or user-specified for relative includes
  return loadEntities(doc.get(), base_dir, db);
}

bool EntityParser::loadEntities(tinyxml2::XMLDocument* doc, const std::string& base_dir, ginseng::database& db)
{
  ObjectIndex object_index;
  buildObjectIndex(doc, object_index, "", base_dir, "[root]");

  SceneMap scene_map;
  parseSceneObjects(doc, object_index, scene_map);

  for (const auto& scene : scene_map)
  {
    auto eid = db.create_entity();
    components::ObjectComponent obj_comp{ .id = scene.second.id };
    db.add_component(eid, std::move(obj_comp));

    for (const auto& comp : scene.second.components)
    {
      size_t index = static_cast<size_t>(comp.type);
      if (index >= parseFuncs.size())
        throw std::runtime_error("No parse function for component id: " + comp.id);

      parseComponents(comp.xml, doc, db, eid, index);
    }
  }
  return true;
}

}  // namespace xml
}  // namespace sodf
