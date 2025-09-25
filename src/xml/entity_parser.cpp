#include <unordered_set>

#include <sodf/xml/utils.h>
#include <sodf/xml/macros.h>
#include <sodf/xml/entity_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>

#include <sodf/components/object.h>

namespace sodf {
namespace xml {

constexpr char NAMESPACE_SEP[] = ":";
constexpr std::size_t NAMESPACE_SEP_LEN = sizeof(NAMESPACE_SEP) - 1;

struct OverlaySource
{
  std::string id;      // "hmi-fw-2.4.3"
  std::string slot;    // "hmi"
  std::string target;  // "bio-rad:t100-thermal-cycler" (optional; empty = wildcard)
  const tinyxml2::XMLElement* xml;
  std::string filename;
};

// For <Object> overlay requests
struct OverlayRequest
{
  std::string slot;  // always required on <Object>
  std::string id;    // required when the <Object> has clone="..."
  bool disable = false;
  int line = 0;
};

// Stores all object elements with fully-qualified id
struct ObjectSource
{
  const tinyxml2::XMLElement* xml;
  std::string filename;
  std::string base_id;
  std::unordered_map<std::string, OverlaySource> overlay_map;  // key = id + slot
  std::unique_ptr<tinyxml2::XMLDocument> materialized_doc;
};

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

using ObjectIndex = std::unordered_map<std::string, ObjectSource>;
using SceneMap = std::unordered_map<std::string, SceneObject>;

SceneObject composeSceneObject(const tinyxml2::XMLElement* elem, ObjectIndex& object_index,
                               const std::string& current_ns);

std::string overlay_key(const std::string& id, const std::string& slot)
{
  static constexpr char SEP = '\x1f';
  return id + SEP + slot;
}

void attach_overlay_to_object(ObjectSource& obj, const OverlaySource& ov, const std::string& obj_qid)
{
  // Enforce: one overlay per slot per object (since <Object><Overlay slot="..."/></Object>)
  for (const auto& kv : obj.overlay_map)
  {
    const auto& prev = kv.second;
    if (prev.slot == ov.slot && prev.id != ov.id)
    {
      throw std::runtime_error("Overlay slot collision on object '" + obj_qid + "' for slot='" + ov.slot +
                               "'. Already have id='" + prev.id + "' from '" + prev.filename + "', new id='" + ov.id +
                               "' from '" + ov.filename + "'. Each object may have at most one overlay per slot.");
    }
  }

  // Also guard against exact duplicate (same id+slot)
  auto k = overlay_key(ov.id, ov.slot);
  auto [it, inserted] = obj.overlay_map.emplace(k, ov);
  if (!inserted)
  {
    const auto& prev = it->second;
    throw std::runtime_error("Duplicate Overlay for object '" + obj_qid + "' (id='" + ov.id + "', slot='" + ov.slot +
                             "'). First in '" + prev.filename + "', again in '" + ov.filename + "'.");
  }
}

void apply_overlay(SceneObject& obj, const tinyxml2::XMLElement* overlay_xml)
{
  for (const auto* child = overlay_xml->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    std::string tag = child->Name();

    // Direct component (acts like upsert)
    if (auto t = sceneComponentTypeFromString(tag))
    {
      SceneComponentType comp_type = *t;
      std::string comp_id = child->Attribute("id") ? child->Attribute("id") : "";
      auto match = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
      obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match), obj.components.end());
      obj.components.push_back({ comp_type, comp_id, child });
      continue;
    }

    // Patch block
    if (tag == "Add" || tag == "Update" || tag == "Upsert" || tag == "Remove")
    {
      for (const auto* comp = child->FirstChildElement(); comp; comp = comp->NextSiblingElement())
      {
        std::string comp_type_str = comp->Name();
        auto t2 = sceneComponentTypeFromString(comp_type_str);
        if (!t2)
          throw std::runtime_error("Unknown ComponentType in <Overlay>: " + comp_type_str);
        SceneComponentType comp_type = *t2;
        std::string comp_id = comp->Attribute("id") ? comp->Attribute("id") : "";

        auto match = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
        auto it = std::find_if(obj.components.begin(), obj.components.end(), match);

        if (tag == "Add")
        {
          if (it != obj.components.end())
            throw std::runtime_error("Overlay <Add> duplicates <" + comp_type_str + " id=\"" + comp_id + "\">");
          obj.components.push_back({ comp_type, comp_id, comp });
        }
        else if (tag == "Update")
        {
          if (it == obj.components.end())
            throw std::runtime_error("Overlay <Update> missing <" + comp_type_str + " id=\"" + comp_id + "\">");
          obj.components.erase(it);
          obj.components.push_back({ comp_type, comp_id, comp });
        }
        else if (tag == "Upsert")
        {
          obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match),
                               obj.components.end());
          obj.components.push_back({ comp_type, comp_id, comp });
        }
        else
        {  // Remove
          if (it == obj.components.end())
            throw std::runtime_error("Overlay <Remove> missing <" + comp_type_str + " id=\"" + comp_id + "\">");
          obj.components.erase(it);
        }
      }
      continue;
    }

    throw std::runtime_error("Unexpected tag <" + tag + "> inside <Overlay> definition.");
  }
}

std::string canonical_ns(const std::string& parent_ns, const std::string& local_ns)
{
  if (parent_ns.empty())
    return local_ns;
  if (local_ns.empty())
    return parent_ns;
  return parent_ns + NAMESPACE_SEP + local_ns;
}

// Replace your extract_namespace(...) with a version that uses the base_id.
std::string extract_namespace_from_qid_and_base(const std::string& qualified_id, const std::string& base_id)
{
  if (base_id.empty())
    return {};  // no info; conservatively say "no ns"

  // Expect: qualified_id == <ns> + ":" + base_id   OR   == base_id (no ns).
  const std::size_t sep_len = NAMESPACE_SEP_LEN;  // 1 for ":"
  if (qualified_id.size() <= base_id.size())
  {
    // Either equal (no ns) or malformed; treat as no namespace.
    return {};
  }

  const std::size_t maybe_sep_pos = qualified_id.size() - base_id.size() - sep_len;
  if (qualified_id.compare(maybe_sep_pos, sep_len, NAMESPACE_SEP) != 0)
  {
    return {};
  }

  // Everything before that ":" is the namespace (can include nested, e.g. "aa:bb").
  return qualified_id.substr(0, maybe_sep_pos);
}

std::unordered_set<std::string> collect_namespaces(const ObjectIndex& index)
{
  std::unordered_set<std::string> out;
  out.reserve(index.size());
  for (const auto& kv : index)
  {
    const std::string& qid = kv.first;
    const ObjectSource& os = kv.second;
    std::string ns = extract_namespace_from_qid_and_base(qid, os.base_id);
    if (!ns.empty())
      out.insert(ns);

    // Optional (nice-to-have): also insert all namespace prefixes (for nested ns)
    std::size_t start = 0;
    while (true)
    {
      std::size_t pos = ns.find(NAMESPACE_SEP, start);
      if (pos == std::string::npos)
        break;
      out.insert(ns.substr(0, pos));
      start = pos + NAMESPACE_SEP_LEN;
    }
  }
  return out;
}

// Build a qualified id from a namespace and base id
std::string qualify_id(const std::string& ns, const std::string& id)
{
  return ns.empty() ? id : (ns + NAMESPACE_SEP + id);
}

bool is_qualified_ref(const std::string& ref, const std::unordered_set<std::string>& known_ns)
{
  // Fast path: find first token
  std::size_t pos = ref.find(NAMESPACE_SEP);
  if (pos == std::string::npos)
    return false;

  // If the first token is a known ns, it's qualified.
  if (known_ns.count(ref.substr(0, pos)))
    return true;

  // Optional: check longer prefixes "aa:bb", "aa:bb:cc", ...
  // Walk forward across colons and see if any growing prefix is in known_ns.
  std::size_t scan = pos;
  while (scan != std::string::npos)
  {
    scan = ref.find(NAMESPACE_SEP, scan + NAMESPACE_SEP_LEN);
    if (scan == std::string::npos)
      break;
    if (known_ns.count(ref.substr(0, scan)))
      return true;
  }
  return false;
}

std::string canonicalize_ref(const std::string& current_ns, const std::string& ref,
                             const std::unordered_set<std::string>& known_ns)
{
  if (ref.empty())
    return current_ns;

  // Absolute/global: starts with namespace
  if (ref.rfind(NAMESPACE_SEP, 0) == 0)
    return ref.substr(NAMESPACE_SEP_LEN);

  // Already qualified if it starts with a known namespace
  if (is_qualified_ref(ref, known_ns))
    return ref;

  // Relative: prepend current namespace if we have one
  if (!current_ns.empty())
    return current_ns + NAMESPACE_SEP + ref;

  // No current namespace: leave as-is
  return ref;
}

static bool has_model_attr(const tinyxml2::XMLElement* e)
{
  return e->Attribute("model") != nullptr;
}

static bool declares_required_overlay(const tinyxml2::XMLElement* obj_elem)
{
  for (auto* ov = obj_elem->FirstChildElement("Overlay"); ov; ov = ov->NextSiblingElement("Overlay"))
  {
    const bool required = ov->BoolAttribute("required", /*default*/ true);
    const char* id = ov->Attribute("id");
    if (required && (!id || !*id))
      return true;  // contract slot
  }
  return false;
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& parent_ns,
                      const std::string& base_dir, const std::string& filename, bool promote_objects);

void processInclude(const tinyxml2::XMLElement* include_elem, ObjectIndex& object_index,
                    const std::string& including_file_dir, const std::string& parent_ns)
{
  std::string path = include_elem->Attribute("path");
  std::string ns = include_elem->Attribute("ns") ? include_elem->Attribute("ns") : "";
  std::string full_ns = canonical_ns(parent_ns, ns);

  // Resolve included file path relative to current file's directory
  std::string inc_filename = including_file_dir.empty() ? path : (including_file_dir + "/" + path);

  auto* inc_doc = new tinyxml2::XMLDocument();
  if (inc_doc->LoadFile(inc_filename.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Failed to include: " + inc_filename);

  std::string inc_file_dir = getDirectory(inc_filename);

  buildObjectIndex(inc_doc, object_index, full_ns, inc_file_dir, inc_filename, true);
}

bool ends_with(const std::string& s, const std::string& suf)
{
  return s.size() >= suf.size() && std::equal(s.end() - suf.size(), s.end(), suf.begin());
}

bool declares_overlay_contract(const tinyxml2::XMLElement* obj_elem)
{
  // If the model declares any <Overlay> child, treat it as a contract model.
  // (We could be stricter: only treat as contract if an <Overlay> has no id and required=true.
  // But your stated rule says *any* <Overlay> means “don’t materialize”.)
  return obj_elem->FirstChildElement("Overlay") != nullptr;
}

void register_materialized_model(ObjectIndex& index, const std::string& qualified_id, const std::string& base_id,
                                 const SceneObject& composed_obj)
{
  // Make a fresh doc containing exactly the final components
  auto doc = std::make_unique<tinyxml2::XMLDocument>();
  auto* obj_elem = doc->NewElement("Object");
  obj_elem->SetAttribute("id", base_id.c_str());
  doc->InsertFirstChild(obj_elem);

  // Copy each final component XML under the new <Object>
  for (const auto& sc : composed_obj.components)
  {
    // DeepCopy from the original component element

    // DeepCopy is a member of XMLNode
    tinyxml2::XMLNode* copied = sc.xml->DeepClone(doc.get());
    obj_elem->InsertEndChild(copied);
  }

  // Insert/overwrite the index entry (preserve overlay_map if already present)
  auto [it, inserted] = index.emplace(qualified_id, ObjectSource{});
  if (!inserted)
  {
    // preserve existing overlays
    // (we only overwrite xml/filename/base_id/materialized_doc)
  }

  it->second.xml = obj_elem;               // points into materialized_doc
  it->second.filename = "[materialized]";  // purely informational
  it->second.base_id = base_id;
  it->second.materialized_doc = std::move(doc);  // keep alive
}

bool is_patch_tag(std::string_view tag)
{
  return tag == "Add" || tag == "Update" || tag == "Upsert" || tag == "Remove";
}

void apply_patch_block(SceneObject& obj,
                       const tinyxml2::XMLElement* block,  // <Add>/<Update>/<Upsert>/<Remove>
                       std::string_view op_name)           // "Add"/"Update"/"Upsert"/"Remove"
{
  for (const auto* comp = block->FirstChildElement(); comp; comp = comp->NextSiblingElement())
  {
    const std::string comp_type_str = comp->Name();
    auto opt_type = sceneComponentTypeFromString(comp_type_str);
    if (!opt_type)
      throw std::runtime_error("Unknown ComponentType string: " + comp_type_str);

    SceneComponentType comp_type = *opt_type;
    std::string comp_id = comp->Attribute("id") ? comp->Attribute("id") : "";

    auto match_pred = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
    auto it = std::find_if(obj.components.begin(), obj.components.end(), match_pred);

    if (op_name == "Add")
    {
      if (it != obj.components.end())
        throw std::runtime_error("Cannot <Add> already existing component <" + comp_type_str + " id=\"" + comp_id +
                                 "\"> at line " + std::to_string(comp->GetLineNum()) + ".");
      obj.components.push_back({ comp_type, comp_id, comp });
    }
    else if (op_name == "Update")
    {
      if (it == obj.components.end())
        throw std::runtime_error("Cannot <Update> non-existent component <" + comp_type_str + " id=\"" + comp_id +
                                 "\"> at line " + std::to_string(comp->GetLineNum()) + ".");
      obj.components.erase(it);
      obj.components.push_back({ comp_type, comp_id, comp });
    }
    else if (op_name == "Upsert")
    {
      obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred),
                           obj.components.end());
      obj.components.push_back({ comp_type, comp_id, comp });
    }
    else /* Remove */
    {
      if (it == obj.components.end())
        throw std::runtime_error("Cannot <Remove> non-existent component <" + comp_type_str + " id=\"" + comp_id +
                                 "\"> at line " + std::to_string(comp->GetLineNum()) + ".");
      obj.components.erase(it);
    }
  }
}

// “Upsert” a direct component tag like <Link>, <Joint>, <FSM>, ...
void upsert_direct_component(SceneObject& obj, const tinyxml2::XMLElement* child)
{
  const std::string tag = child->Name();
  auto opt_type = sceneComponentTypeFromString(tag);
  if (!opt_type)
    throw std::runtime_error("Unknown ComponentType string: " + tag);
  SceneComponentType comp_type = *opt_type;

  std::string comp_id = child->Attribute("id") ? child->Attribute("id") : "";
  auto match_pred = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };

  obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred), obj.components.end());
  obj.components.push_back({ comp_type, comp_id, child });
}

// void register_ephemeral_model(ObjectIndex& index, const std::string& qualified_id, const std::string& base_id,
//                               const tinyxml2::XMLElement* obj_elem, const std::string& filename_hint = "[scene]")
// {
//   auto [it, inserted] = index.emplace(qualified_id, ObjectSource{});
//   // keep any overlays already attached for this qid (unlikely here, but safe)
//   it->second.xml = obj_elem;
//   it->second.filename = filename_hint;
//   it->second.base_id = base_id;  // store the bare id (no ns)
// }

SceneObject composeModelObject(const tinyxml2::XMLElement* elem, const ObjectIndex& object_index,
                               const std::string& current_ns)
{
  SceneObject obj;
  obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";

  // Collect overlay tags (model-level). We DO NOT enforce them here.
  // (We also do NOT apply model defaults here; can be added later if desired.)
  bool has_model = elem->Attribute("model");

  // If cloning a model: recursively compose the clone source as a MODEL (no enforcement)
  if (has_model)
  {
    // Resolve clone id (handle namespaces)
    auto known_ns = collect_namespaces(object_index);
    std::string model_id = elem->Attribute("model");
    std::string canonical_clone = canonicalize_ref(current_ns, model_id, known_ns);

    auto found = object_index.find(canonical_clone);
    if (found == object_index.end())
      throw std::runtime_error("Clone source not found: " + canonical_clone + " (line " +
                               std::to_string(elem->GetLineNum()) + ")");

    std::string cloned_ns = extract_namespace_from_qid_and_base(canonical_clone, found->second.base_id);

    // Recurse as MODEL
    obj = composeModelObject(found->second.xml, object_index, cloned_ns);
    obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";
  }

  // Walk children (centralized handling)
  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "Overlay")
      continue;  // model requirements handled at scene level

    if (has_model)
    {
      // Clone (model): only patch blocks are allowed
      if (!is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside <Object id=\"" + obj.id +
                                 "\" model=\"...\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only <Add>, <Update>, <Upsert>, <Remove> or <Overlay> are allowed.");

      apply_patch_block(obj, child, tag);
    }
    else
    {
      // Non-clone (model): only direct components (no patch blocks)
      if (is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside non-clone <Object id=\"" + obj.id +
                                 "\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only direct component tags (e.g., <Origin>, <Link>) are allowed.");

      upsert_direct_component(obj, child);
    }
  }

  return obj;
}

void processImport(const tinyxml2::XMLElement* import_elem, ObjectIndex& object_index,
                   const std::string& including_file_dir, const std::string& parent_ns, SceneMap& scene_map)
{
  // 1) Resolve path + ns
  const char* path_c = import_elem->Attribute("path");
  if (!path_c || !*path_c)
  {
    throw std::runtime_error("<Import> missing 'path' at line " + std::to_string(import_elem->GetLineNum()));
  }
  std::string path = path_c;
  std::string ns = import_elem->Attribute("ns") ? import_elem->Attribute("ns") : "";
  std::string full_ns = canonical_ns(parent_ns, ns);

  // 2) Load target doc
  std::string inc_filename = including_file_dir.empty() ? path : (including_file_dir + "/" + path);
  auto inc_doc = std::make_unique<tinyxml2::XMLDocument>();
  if (inc_doc->LoadFile(inc_filename.c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error("Failed to import: " + inc_filename);
  }
  std::string inc_file_dir = getDirectory(inc_filename);

  // 3) Register models/overlays from imported doc (NO entities)
  buildObjectIndex(inc_doc.get(), object_index, full_ns, inc_file_dir, inc_filename, true);

  // 4) Promote imported doc’s top-level <Object>s into snapshot models (no entities)
  if (const auto* inc_root = inc_doc->FirstChildElement("Root"))
  {
    for (const auto* obj_elem = inc_root->FirstChildElement("Object"); obj_elem;
         obj_elem = obj_elem->NextSiblingElement("Object"))
    {
      if (has_model_attr(obj_elem))
      {
        // Instance: compose with enforcement (applies overlays/patches), then snapshot
        SceneObject inst = composeSceneObject(obj_elem, object_index, full_ns);
        register_materialized_model(object_index, qualify_id(full_ns, inst.id), inst.id, inst);
      }
      else if (!declares_required_overlay(obj_elem))
      {
        // Plain model (no required overlay slots): safe to materialize without enforcement
        SceneObject m = composeModelObject(obj_elem, object_index, full_ns);
        register_materialized_model(object_index, qualify_id(full_ns, m.id), m.id, m);
      }
      // else: contract model → skip (must be satisfied by a scene instance)
    }
  }

  // 5) Import chaining: process <Import> tags found INSIDE the imported document
  //    (only when reached via Import; Include won't do this)
  if (const auto* root = inc_doc->FirstChildElement("Root"))
  {
    for (const auto* nested = root->FirstChildElement("Import"); nested; nested = nested->NextSiblingElement("Import"))
    {
      processImport(nested, object_index, inc_file_dir, full_ns, scene_map);
    }
  }

  // 6) Instantiate the <Object> children that are listed INSIDE THIS <Import> block
  //    (they belong to the CURRENT scene)
  const std::string current_ns;  // scene has no ns by default
  for (const auto* obj_elem = import_elem->FirstChildElement("Object"); obj_elem;
       obj_elem = obj_elem->NextSiblingElement("Object"))
  {
    SceneObject obj = composeSceneObject(obj_elem, object_index, current_ns);
    if (obj.id.empty())
    {
      throw std::runtime_error("Imported <Object> is missing 'id' at line " + std::to_string(obj_elem->GetLineNum()));
    }
    scene_map[obj.id] = std::move(obj);
  }
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& ns,
                      const std::string& base_dir, const std::string& filename, bool promote_objects)
{
  const auto* root = doc->FirstChildElement("Root");
  if (!root)
    return;

  // Handle <include> recursively
  for (const auto* incl = root->FirstChildElement("Include"); incl; incl = incl->NextSiblingElement("Include"))
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

    std::string qualified_id = ns.empty() ? id : (ns + NAMESPACE_SEP + id);

    // Create entry if missing; keep existing (placeholder) if present
    auto [it, inserted] = object_index.emplace(qualified_id, ObjectSource{});
    if (!inserted && it->second.xml != nullptr)
    {
      const ObjectSource& previous = it->second;
      throw std::runtime_error("Duplicate <Object> for id '" + qualified_id + "' in file '" + filename + "' at line " +
                               std::to_string(obj_elem->GetLineNum()) + ". Previous definition was at line " +
                               std::to_string(previous.xml->GetLineNum()) + " in file '" + previous.filename + "'.");
    }

    // Fill the placeholder (or newly inserted) WITHOUT clearing overlay_map
    it->second.xml = obj_elem;
    it->second.filename = filename;

    // If placeholder had empty base_id, set it now; if it had a value, sanity-check (optional)
    if (it->second.base_id.empty())
      it->second.base_id = id;
    else if (it->second.base_id != id)
    {
      // Optional: warn or throw if you expect consistency
      throw std::runtime_error("Mismatched base_id for '" + qualified_id + "' ...");
    }
  }

  // NEW: overlays...
  for (const auto* ov_elem = root->FirstChildElement("Overlay"); ov_elem;
       ov_elem = ov_elem->NextSiblingElement("Overlay"))
  {
    OverlaySource ov;
    ov.id = ov_elem->Attribute("id") ? ov_elem->Attribute("id") : "";
    ov.slot = ov_elem->Attribute("slot") ? ov_elem->Attribute("slot") : "";
    ov.target = ov_elem->Attribute("target") ? ov_elem->Attribute("target") : "";  // base id (no scene ns)
    ov.xml = ov_elem;
    ov.filename = filename;

    if (ov.slot.empty())
      throw std::runtime_error("<Overlay> missing 'slot' at line " + std::to_string(ov_elem->GetLineNum()));

    // Attach to any object already known whose base_id matches ov.target,
    // or to all if ov.target is empty (wildcard).
    // Also allow suffix match on qualified id to be tolerant of namespaces.
    for (auto& [qid, os] : object_index)
    {
      if (ov.target.empty() || os.base_id == ov.target || ends_with(qid, std::string(NAMESPACE_SEP) + ov.target) ||
          qid == ov.target)
      {
        attach_overlay_to_object(os, ov, qid);
        // std::cout << "[buildObjectIndex] Attached overlay id='" << ov.id << "' slot='" << ov.slot << "' target='"
        //           << ov.target << "' to object qid='" << qid << "' from file '" << ov.filename << "'\n";
      }
    }
  }

  // Compose *each* included <Object> into a materialized model (no entities created).
  if (promote_objects)
  {
    for (const auto* obj_elem = root->FirstChildElement("Object"); obj_elem;
         obj_elem = obj_elem->NextSiblingElement("Object"))
    {
      if (has_model_attr(obj_elem))
      {
        // INSTANCE: fully compose as a SCENE object (applies overlays & patches),
        // but DO NOT create entities here—just snapshot-register for cloning.
        SceneObject inst = composeSceneObject(obj_elem, object_index, ns);
        const std::string qid = qualify_id(ns, inst.id);
        register_materialized_model(object_index, qid, inst.id, inst);
        continue;
      }

      // MODEL (no model="..."):
      if (declares_required_overlay(obj_elem))
      {
        // Contract model → do NOT materialize at index time
        continue;
      }

      // Plain model: safe to materialize without enforcement
      SceneObject m = composeModelObject(obj_elem, object_index, ns);
      const std::string qid = qualify_id(ns, m.id);
      register_materialized_model(object_index, qid, m.id, m);
    }
  }
}

SceneObject composeSceneObject(const tinyxml2::XMLElement* elem, ObjectIndex& object_index,
                               const std::string& current_ns)
{
  // Build model_qid and compose the MODEL first (no overlay enforcement there)
  SceneObject obj;
  std::string model_qid;

  bool has_model = elem->Attribute("model");
  if (has_model)
  {
    auto known_ns = collect_namespaces(object_index);
    std::string clone_id = elem->Attribute("model");
    std::string canonical_clone = canonicalize_ref(current_ns, clone_id, known_ns);

    auto found = object_index.find(canonical_clone);
    if (found == object_index.end())
      throw std::runtime_error("Clone source not found: " + canonical_clone + " (line " +
                               std::to_string(elem->GetLineNum()) + ")");

    std::string modeled_ns = extract_namespace_from_qid_and_base(canonical_clone, found->second.base_id);

    obj = composeModelObject(found->second.xml, object_index, modeled_ns);
    obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";
    model_qid = canonical_clone;  // overlays belong to the clone source model
  }
  else
  {
    obj = composeModelObject(elem, object_index, current_ns);
    model_qid = qualify_id(current_ns, obj.id);
  }

  // Collect instance-level overlay requests
  std::vector<OverlayRequest> overlay_reqs;
  for (const auto* ov = elem->FirstChildElement("Overlay"); ov; ov = ov->NextSiblingElement("Overlay"))
  {
    OverlayRequest r;
    r.slot = ov->Attribute("slot") ? ov->Attribute("slot") : "";
    r.id = ov->Attribute("id") ? ov->Attribute("id") : "";
    r.disable = ov->BoolAttribute("disable", false);
    r.line = ov->GetLineNum();
    if (r.slot.empty())
      throw std::runtime_error("<Overlay> missing 'slot' at line " + std::to_string(r.line));
    overlay_reqs.push_back(std::move(r));
  }

  // Lookup model overlays and model requirements
  auto mit = object_index.find(model_qid);
  if (mit == object_index.end())
    throw std::runtime_error("Model '" + model_qid + "' not found in object index while resolving overlays.");
  const ObjectSource& model_os = mit->second;

  auto find_by_id_slot = [&](const std::string& id, const std::string& slot) -> const OverlaySource& {
    auto k = overlay_key(id, slot);
    auto it = model_os.overlay_map.find(k);
    if (it == model_os.overlay_map.end())
      throw std::runtime_error("Overlay id='" + id + "' slot='" + slot + "' not available for model '" + model_qid +
                               "'.");
    return it->second;
  };

  auto collect_required_slots = [&](const ObjectSource& mos) {
    std::unordered_set<std::string> req;
    const tinyxml2::XMLElement* model_elem = mos.xml;
    for (const auto* ov = model_elem->FirstChildElement("Overlay"); ov; ov = ov->NextSiblingElement("Overlay"))
    {
      const char* slot_c = ov->Attribute("slot");
      if (!slot_c || !*slot_c)
        throw std::runtime_error(std::string("<Overlay> missing 'slot' at line ") + std::to_string(ov->GetLineNum()) +
                                 " in model '" + model_qid + "'.");
      const char* id_c = ov->Attribute("id");
      const bool required = ov->BoolAttribute("required", /*default*/ true);
      if (required && (!id_c || !*id_c))
        req.insert(slot_c);
    }
    return req;
  };

  std::unordered_set<std::string> required_slots = collect_required_slots(model_os);
  std::unordered_set<std::string> satisfied_slots;

  // Apply/disable instance overlay requests; enforce clone vs non-clone rules
  if (has_model)
  {
    for (const auto& req : overlay_reqs)
    {
      if (req.disable)
      {
        satisfied_slots.insert(req.slot);
        continue;
      }
      if (req.id.empty())
        throw std::runtime_error("Overlay at clone site must specify 'id' or set disable=\"true\" "
                                 "(line " +
                                 std::to_string(req.line) + ").");
      const OverlaySource& ov = find_by_id_slot(req.id, req.slot);
      apply_overlay(obj, ov.xml);
      satisfied_slots.insert(req.slot);
    }
  }
  else
  {
    for (const auto& req : overlay_reqs)
    {
      if (req.disable)
      {
        satisfied_slots.insert(req.slot);
        continue;
      }
      if (req.id.empty())
        continue;  // marker only (no immediate application)
      const OverlaySource& ov = find_by_id_slot(req.id, req.slot);
      apply_overlay(obj, ov.xml);
      satisfied_slots.insert(req.slot);
    }
  }

  // Enforce: every model-required slot must be satisfied (applied or disabled)
  for (const auto& slot : required_slots)
  {
    if (!satisfied_slots.count(slot))
    {
      throw std::runtime_error("Object id='" + obj.id + "' (model '" + model_qid + "') requires overlay for slot='" +
                               slot + "' but none was supplied; use <Overlay slot=\"" + slot +
                               "\" id=\"...\"/> or disable=\"true\".");
    }
  }

  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "Overlay")
      continue;  // already processed

    if (has_model)
    {
      // In a cloned scene instance, only patch blocks are allowed
      if (!is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside scene <Object id=\"" + obj.id +
                                 "\" model=\"...\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only <Add>, <Update>, <Upsert>, or <Remove> are allowed.");

      apply_patch_block(obj, child, tag);
    }
    else
    {
      // In a non-clone scene instance, only direct components (no patch blocks)
      if (is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside non-clone scene <Object id=\"" + obj.id +
                                 "\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only direct component tags or <Overlay> are allowed.");

      upsert_direct_component(obj, child);
    }
  }

  // Compute the model qid for the instance itself (scene has no ns by default)
  const std::string self_model_qid = qualify_id(current_ns, obj.id);

  // Make this object available as a model for later use
  register_materialized_model(object_index, self_model_qid, obj.id, obj);

  return obj;
}

void parseSceneObjects(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, SceneMap& scene,
                       const std::string& base_dir)
{
  const auto* root = doc->FirstChildElement("Root");
  if (!root)
    return;

  for (const auto* imp = root->FirstChildElement("Import"); imp; imp = imp->NextSiblingElement("Import"))
  {
    processImport(imp, object_index, /*including_file_dir=*/base_dir, /*parent_ns=*/"", scene);
  }

  // Only create entities for objects defined in the MAIN SCENE file!
  for (const auto* obj_elem = root->FirstChildElement("Object"); obj_elem;
       obj_elem = obj_elem->NextSiblingElement("Object"))
  {
    // Compose directly from the SCENE object element.
    // (This lets <Overlay>, <Remove>, clone rules, etc. on the scene instance be respected.)
    const std::string current_ns;  // scene file has no namespace by default
    SceneObject obj = composeSceneObject(obj_elem, object_index, current_ns);

    if (obj.id.empty())
      throw std::runtime_error("Scene <Object> is missing 'id'.");

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
    // std::cout << "[parseComponents] Parsing component typeIdx=" << parseFuncIdx << " id='"
    //           << (elem->Attribute("id") ? elem->Attribute("id") : "") << "' tag='" << elem->Name() << "'\n";
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
  buildObjectIndex(doc, object_index, "", base_dir, "[root]", false);

  SceneMap scene_map;
  parseSceneObjects(doc, object_index, scene_map, base_dir);

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
