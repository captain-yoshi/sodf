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
  int line = 0;
};

// Stores all object elements with fully-qualified id
struct ObjectSource
{
  const tinyxml2::XMLElement* xml;
  std::string filename;
  std::string base_id;
  std::unordered_map<std::string, OverlaySource> overlay_map;  // key = id + slot
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

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& parent_ns,
                      const std::string& base_dir, const std::string& filename);

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

  buildObjectIndex(inc_doc, object_index, full_ns, inc_file_dir, inc_filename);
}

bool ends_with(const std::string& s, const std::string& suf)
{
  return s.size() >= suf.size() && std::equal(s.end() - suf.size(), s.end(), suf.begin());
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& ns,
                      const std::string& base_dir, const std::string& filename)
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
}

SceneObject composeObject(const tinyxml2::XMLElement* elem, const ObjectIndex& object_index,
                          const std::string& current_ns)
{
  SceneObject obj;
  obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";

  // Collect <Overlay .../> requests on THIS <Object>
  std::vector<OverlayRequest> overlay_reqs;
  for (const auto* ov = elem->FirstChildElement("Overlay"); ov; ov = ov->NextSiblingElement("Overlay"))
  {
    OverlayRequest r;
    r.slot = ov->Attribute("slot") ? ov->Attribute("slot") : "";
    r.id = ov->Attribute("id") ? ov->Attribute("id") : "";  // optional here; may be required later
    r.line = ov->GetLineNum();
    if (r.slot.empty())
      throw std::runtime_error("<Overlay> missing 'slot' at line " + std::to_string(r.line));
    overlay_reqs.push_back(std::move(r));
  }

  // Precompute once where you have the index (e.g., at the start of loadEntities)
  auto known_ns = collect_namespaces(object_index);

  bool has_clone = elem->Attribute("clone");

  std::string model_qid = qualify_id(current_ns, obj.id);  // default when not cloning

  // --- CLONE (deep-merge) ---
  if (has_clone)
  {
    std::string clone_id = elem->Attribute("clone");
    std::string canonical_clone = canonicalize_ref(current_ns, clone_id, known_ns);

    auto found = object_index.find(canonical_clone);
    if (found == object_index.end())
      throw std::runtime_error("Clone source not found: " + canonical_clone + " (line " +
                               std::to_string(elem->GetLineNum()) + ")");

    std::string cloned_ns = extract_namespace_from_qid_and_base(canonical_clone, found->second.base_id);
    obj = composeObject(found->second.xml, object_index, cloned_ns);  // Recursively clone
    obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";

    model_qid = canonical_clone;  // overlays are pulled from the clone source model
  }

  // Apply overlays BEFORE local Add/Update/Upsert/Remove
  if (!overlay_reqs.empty())
  {
    auto mit = object_index.find(model_qid);
    if (mit == object_index.end())
      throw std::runtime_error("Model '" + model_qid + "' not found in object index while resolving overlays.");
    const ObjectSource& model_os = mit->second;

    // lookup by (id, slot)
    auto find_by_id_slot = [&](const std::string& id, const std::string& slot) -> const OverlaySource& {
      auto k = overlay_key(id, slot);
      auto it = model_os.overlay_map.find(k);
      if (it == model_os.overlay_map.end())
        throw std::runtime_error("Overlay id='" + id + "' slot='" + slot + "' not available for model '" + model_qid +
                                 "'.");
      return it->second;
    };

    if (has_clone)
    {
      // CLONE site: every request must specify id + slot
      for (const auto& req : overlay_reqs)
      {
        if (req.id.empty())
          throw std::runtime_error("Overlay at clone site must specify both 'id' and 'slot' "
                                   "(line " +
                                   std::to_string(req.line) + ").");
        const OverlaySource& ov = find_by_id_slot(req.id, req.slot);
        apply_overlay(obj, ov.xml);
        // std::cout << "[composeObject] Applying overlay id='" << ov.id << "' slot='" << ov.slot << "' to object='"
        //           << obj.id << "' (model_qid='" << model_qid << "')\n";
      }
    }
    else
    {
      // BASE/MODEL: slot-only is requirement-only; id+slot may apply directly if you want to
      for (const auto& req : overlay_reqs)
      {
        if (req.id.empty())
          continue;  // requirement only, do not apply
        const OverlaySource& ov = find_by_id_slot(req.id, req.slot);
        apply_overlay(obj, ov.xml);
        // std::cout << "[composeObject] Applying overlay id='" << ov.id << "' slot='" << ov.slot << "' to object='"
        //           << obj.id << "' (model_qid='" << model_qid << "')\n";
      }
    }
  }

  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    std::string tag = child->Name();

    if (tag == "Overlay")
      continue;  // already processed / or requirement-only

    // If inside clone, only patch operations are allowed
    if (has_clone)
    {
      if (tag == "Add" || tag == "Update" || tag == "Upsert" || tag == "Remove")
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

          if (tag == "Add")
          {
            if (it != obj.components.end())
            {
              throw std::runtime_error("Cannot <Add> already existing component <" + comp_type_str + " id=\"" +
                                       comp_id + "\"> to <Object id=\"" + obj.id + "\"> at line " +
                                       std::to_string(comp->GetLineNum()) + ".");
            }
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "Update")
          {
            if (it == obj.components.end())
            {
              throw std::runtime_error("Cannot <Update> non-existent component <" + comp_type_str + " id=\"" + comp_id +
                                       "\"> in <Object id=\"" + obj.id + "\"> at line " +
                                       std::to_string(comp->GetLineNum()) + ".");
            }
            // Remove and replace
            obj.components.erase(it);
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "Upsert")
          {
            // Remove if exists, then add
            obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred),
                                 obj.components.end());
            obj.components.push_back({ comp_type, comp_id, comp });
          }
          else if (tag == "Remove")
          {
            if (it == obj.components.end())
            {
              throw std::runtime_error("Cannot <Remove> non-existent component <" + comp_type_str + " id=\"" + comp_id +
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
      if (tag == "Add" || tag == "Update" || tag == "Upsert" || tag == "Remove")
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
  const auto* root = doc->FirstChildElement("Root");
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
    std::string current_ns = extract_namespace_from_qid_and_base(it->first, it->second.base_id);

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
