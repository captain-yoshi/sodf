#include <unordered_set>

#include <sodf/component_type.h>
#include <sodf/uri_resolver.h>
#include <sodf/xml/utils.h>
#include <sodf/xml/entity_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>
#include <sodf/xml/patch_operations.h>

#include <sodf/components/object.h>

#include <iostream>

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

using ObjectIndex = std::unordered_map<std::string, ObjectSource>;
using SceneMap = std::unordered_map<std::string, SceneObject>;

SceneObject composeSceneObject(const tinyxml2::XMLElement* elem, ObjectIndex& object_index,
                               const std::string& current_ns, const std::string& scene_base_dir);

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
  // --- grab <Include> attributes ---
  const char* uri_attr = include_elem->Attribute("uri");
  if (!uri_attr)
  {
    throw std::runtime_error("<Include> missing required 'uri' attribute");
  }
  std::string uri = uri_attr;

  std::string ns = include_elem->Attribute("ns") ? include_elem->Attribute("ns") : "";
  std::string full_ns = canonical_ns(parent_ns, ns);

  // --- resolve the URI to a local path ---
  sodf::Resolved resolved = sodf::resolve_resource_uri(uri,
                                                       /*current_xml_dir=*/including_file_dir);

  if (resolved.local_path.empty())
  {
    throw std::runtime_error("Failed to resolve <Include> uri=" + uri);
  }

  // --- load the included XML ---
  auto* inc_doc = new tinyxml2::XMLDocument();
  if (inc_doc->LoadFile(resolved.local_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error("Failed to parse included file: " + resolved.local_path);
  }

  // --- recurse to build object index from included file ---
  std::string inc_file_dir = getDirectory(resolved.local_path);
  buildObjectIndex(inc_doc, object_index, full_ns, inc_file_dir, resolved.local_path, true);
}

bool ends_with(const std::string& s, const std::string& suf)
{
  return s.size() >= suf.size() && std::equal(s.end() - suf.size(), s.end(), suf.begin());
}

void register_materialized_model(ObjectIndex& index, const std::string& qualified_id, const std::string& base_id,
                                 const SceneObject& composed_obj, const std::string& base_dir)
{
  // Make a fresh doc containing exactly the final components
  auto doc = std::make_unique<tinyxml2::XMLDocument>();
  auto* obj_elem = doc->NewElement("Object");
  obj_elem->SetAttribute("id", base_id.c_str());
  if (!base_dir.empty())
  {
    obj_elem->SetAttribute("xml:base", base_dir.c_str());  // directory (absolute/canonical recommended)
  }
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
      continue;  // handled later

    if (is_patch_tag(tag))
      continue;  // handled at scene-level (not model-level)

    // Otherwise, itâ€™s a component tag â€” register directly
    upsert_direct_component(obj, child);
  }

  return obj;
}

void processImport(const tinyxml2::XMLElement* import_elem, ObjectIndex& object_index,
                   const std::string& including_file_dir, const std::string& parent_ns, SceneMap& scene_map)
{
  // 1) Required attributes: uri + (optional) ns
  const char* uri_c = import_elem->Attribute("uri");
  if (!uri_c || !*uri_c)
  {
    throw std::runtime_error(std::string("<Import> missing 'uri' at line ") + std::to_string(import_elem->GetLineNum()));
  }
  const std::string uri = uri_c;

  const std::string ns = import_elem->Attribute("ns") ? import_elem->Attribute("ns") : "";
  const std::string full_ns = canonical_ns(parent_ns, ns);

  // 2) Resolve the URI to a local file (supports sodf://, http(s)://, file://, or relative)
  sodf::Resolved resolved = sodf::resolve_resource_uri(uri,
                                                       /*current_xml_dir=*/including_file_dir);

  if (resolved.local_path.empty())
  {
    throw std::runtime_error(std::string("Failed to resolve <Import> uri='") + uri + "' at line " +
                             std::to_string(import_elem->GetLineNum()));
  }

  // 3) Load target doc
  auto inc_doc = std::make_unique<tinyxml2::XMLDocument>();
  if (inc_doc->LoadFile(resolved.local_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error("Failed to import: " + resolved.local_path);
  }
  const std::string inc_file_dir = getDirectory(resolved.local_path);

  // 4) Register models/overlays from imported doc (NO entities)
  buildObjectIndex(inc_doc.get(), object_index, full_ns, inc_file_dir, resolved.local_path, /*no_entities=*/true);

  // 5) Promote top-level <Object>s from imported doc into snapshot models (no entities)
  if (const auto* inc_root = inc_doc->FirstChildElement("Root"))
  {
    for (const tinyxml2::XMLElement* obj_elem = inc_root->FirstChildElement("Object"); obj_elem;
         obj_elem = obj_elem->NextSiblingElement("Object"))
    {
      if (has_model_attr(obj_elem))
      {
        SceneObject inst = composeSceneObject(obj_elem, object_index, full_ns, including_file_dir);
        register_materialized_model(object_index, qualify_id(full_ns, inst.id), inst.id, inst, inc_file_dir);
      }
      else if (!declares_required_overlay(obj_elem))
      {
        SceneObject m = composeModelObject(obj_elem, object_index, full_ns);
        register_materialized_model(object_index, qualify_id(full_ns, m.id), m.id, m, inc_file_dir);
      }
      // else: contract model â†’ skip
    }
  }

  // 6) Import chaining: process nested <Import> inside the imported document (import-only behavior)
  if (const auto* root = inc_doc->FirstChildElement("Root"))
  {
    for (const auto* nested = root->FirstChildElement("Import"); nested; nested = nested->NextSiblingElement("Import"))
    {
      processImport(nested, object_index, inc_file_dir, full_ns, scene_map);
    }
  }

  // 7) Instantiate <Object> children listed INSIDE THIS <Import> block into the CURRENT scene (no ns by default)
  const std::string current_ns;  // empty = scene namespace
  for (const auto* obj_elem = import_elem->FirstChildElement("Object"); obj_elem;
       obj_elem = obj_elem->NextSiblingElement("Object"))
  {
    SceneObject obj = composeSceneObject(obj_elem, object_index, current_ns, including_file_dir);
    if (obj.id.empty())
    {
      throw std::runtime_error(std::string("Imported <Object> missing 'id' at line ") +
                               std::to_string(obj_elem->GetLineNum()));
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

  // Stamp xml:base on <Root>
  {
    auto* root_nc = const_cast<tinyxml2::XMLElement*>(root);
    if (!root_nc->Attribute("xml:base"))
      root_nc->SetAttribute("xml:base", base_dir.c_str());
  }

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
        // but DO NOT create entities hereâ€”just snapshot-register for cloning.
        // SceneObject inst = composeSceneObject(obj_elem, object_index, ns);
        // const std::string qid = qualify_id(ns, inst.id);
        // register_materialized_model(object_index, qid, inst.id, inst);
        continue;
      }

      // MODEL (no model="..."):
      if (declares_required_overlay(obj_elem))
      {
        // Contract model â†’ do NOT materialize at index time
        continue;
      }

      // Plain model: safe to materialize without enforcement
      SceneObject m = composeModelObject(obj_elem, object_index, ns);
      const std::string qid = qualify_id(ns, m.id);
      register_materialized_model(object_index, qid, m.id, m, base_dir);
    }
  }
}

SceneObject composeSceneObject(const tinyxml2::XMLElement* elem, ObjectIndex& object_index,
                               const std::string& current_ns, const std::string& scene_base_dir)
{
  // --- Step 1: Build the base model (no overlays yet) ---
  SceneObject obj;
  std::string model_qid;

  const bool has_model = elem->Attribute("model");
  if (has_model)
  {
    // Resolve clone source
    auto known_ns = collect_namespaces(object_index);
    const std::string clone_id = elem->Attribute("model");
    const std::string canonical_clone = canonicalize_ref(current_ns, clone_id, known_ns);

    auto found = object_index.find(canonical_clone);
    if (found == object_index.end())
      throw std::runtime_error("Clone source not found: " + canonical_clone + " (line " +
                               std::to_string(elem->GetLineNum()) + ")");

    const std::string modeled_ns = extract_namespace_from_qid_and_base(canonical_clone, found->second.base_id);

    obj = composeModelObject(found->second.xml, object_index, modeled_ns);
    obj.id = elem->Attribute("id") ? elem->Attribute("id") : "";
    model_qid = canonical_clone;

    // Debug helper
    // auto dump_components = [&](const SceneObject& o) {
    //   std::cerr << "==== Final composed components for object: " << o.id << " ====\n";
    //   for (auto& sc : o.components)
    //   {
    //     auto* root = component_root(const_cast<SceneComponent&>(sc));
    //     const char* tag = root ? root->Name() : "<null>";
    //     const char* id = (root && root->Attribute("id")) ? root->Attribute("id") : "<no id>";
    //     const char* type = sceneComponentTypeToString(sc.type);

    //     std::cerr << "  â€¢ tag=\"" << tag << "\" id=\"" << id << "\" type=" << type << "\n";
    //     if (std::strcmp(tag, type) != 0)
    //     {
    //       std::cerr << "    [WARN] tag/type mismatch: tag=" << tag << " vs type=" << type << "\n";
    //     }

    //     tinyxml2::XMLPrinter pr;
    //     if (root)
    //     {
    //       root->Accept(&pr);
    //       std::cerr << pr.CStr() << "\n\n";
    //     }
    //   }
    //   std::cerr << "=============================================\n";
    // };
    // dump_components(obj);
  }
  else
  {
    // Scene object without a model
    obj = composeModelObject(elem, object_index, current_ns);
    model_qid = qualify_id(current_ns, obj.id);
  }

  // --- Step 2: Collect overlay requests ---
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

  // --- Step 3: Lookup overlays and apply them ---
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

  // Collect required overlay slots from model
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
      const bool required = ov->BoolAttribute("required", true);
      if (required && (!id_c || !*id_c))
        req.insert(slot_c);
    }
    return req;
  };

  std::unordered_set<std::string> required_slots = collect_required_slots(model_os);
  std::unordered_set<std::string> satisfied_slots;

  // Apply overlays (before any Remove)
  for (const auto& req : overlay_reqs)
  {
    if (req.disable)
    {
      satisfied_slots.insert(req.slot);
      continue;
    }
    if (req.id.empty())
    {
      if (has_model)
        throw std::runtime_error("Overlay at clone site must specify 'id' or disable=\"true\" (line " +
                                 std::to_string(req.line) + ").");
      continue;  // marker only
    }

    const OverlaySource& ov = find_by_id_slot(req.id, req.slot);
    apply_overlay(obj, ov.xml);
    satisfied_slots.insert(req.slot);
  }

  // Enforce required overlay coverage
  for (const auto& slot : required_slots)
  {
    if (!satisfied_slots.count(slot))
    {
      throw std::runtime_error("Object id='" + obj.id + "' (model '" + model_qid + "') requires overlay for slot='" +
                               slot + "' but none was supplied; use <Overlay slot=\"" + slot +
                               "\" id=\"...\"/> or disable=\"true\".");
    }
  }

  // --- Step 4: Apply patch blocks in two phases ---
  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "Overlay")
      continue;

    if (has_model)
    {
      if (!is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside scene <Object id=\"" + obj.id +
                                 "\" model=\"...\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only <Add>, <Update>, <Upsert>, or <Remove> are allowed.");
      apply_patch_block(obj, child, tag);
    }
    else
    {
      if (is_patch_tag(tag))
        throw std::runtime_error("Invalid child element <" + tag + "> inside non-clone scene <Object id=\"" + obj.id +
                                 "\"> at line " + std::to_string(child->GetLineNum()) +
                                 ". Only direct component tags or <Overlay> are allowed.");
      upsert_direct_component(obj, child);
    }
  }

  // --- Step 5: Finalize ---
  for (auto& sc : obj.components)
    ensure_owned(sc);

  const std::string self_model_qid = qualify_id(current_ns, obj.id);
  register_materialized_model(object_index, self_model_qid, obj.id, obj, scene_base_dir);

  // Purge empty components
  obj.components.erase(
      std::remove_if(obj.components.begin(), obj.components.end(),
                     [](const SceneComponent& sc) { return !sc.xml || !sc.xml->GetDocument() || !sc.xml->Name(); }),
      obj.components.end());

  return obj;
}

static void handleSceneElement(const tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, ObjectIndex& object_index,
                               SceneMap& scene, const std::string& base_dir)
{
  const std::string tag = elem->Name();

  // -------------------------------------------------------
  // Include (indexing phase already handled this)
  // -------------------------------------------------------
  if (tag == "Include")
  {
    return;
  }

  // -------------------------------------------------------
  // Import
  // -------------------------------------------------------
  if (tag == "Import")
  {
    processImport(elem, object_index,
                  /*including_file_dir=*/base_dir,
                  /*parent_ns=*/"", scene);
    return;
  }

  // -------------------------------------------------------
  // Object
  // -------------------------------------------------------
  if (tag == "Object")
  {
    const std::string current_ns;

    SceneObject obj = composeSceneObject(elem, object_index, current_ns, base_dir);

    if (obj.id.empty())
      throw std::runtime_error("Scene <Object> is missing 'id'.");

    if (scene.count(obj.id))
      throw std::runtime_error("Duplicate scene object id: " + obj.id);

    scene[obj.id] = std::move(obj);
    return;
  }

  // -------------------------------------------------------
  // ForLoop (scene-level expansion)
  // -------------------------------------------------------
  if (tag == "ForLoop")
  {
    expandForLoop(elem, [&](const std::unordered_map<std::string, std::string>& ctx) {
      for (const tinyxml2::XMLElement* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
      {
        tinyxml2::XMLElement* clone = cloneAndSubstitute(child, doc, ctx);

        handleSceneElement(clone, doc, object_index, scene, base_dir);
      }
    });

    return;
  }

  // -------------------------------------------------------
  // Unknown tag
  // -------------------------------------------------------
  throw std::runtime_error("Invalid top-level scene element <" + tag + "> at line " +
                           std::to_string(elem->GetLineNum()) + ". Only <Import>, <Object>, or <ForLoop> are allowed.");
}

void parseSceneObjects(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, SceneMap& scene,
                       const std::string& base_dir)
{
  const auto* root = doc->FirstChildElement("Root");
  if (!root)
    return;

  for (const auto* child = root->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    handleSceneElement(child, const_cast<tinyxml2::XMLDocument*>(doc), object_index, scene, base_dir);
  }
}

void parseComponentSubElements(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* parent,
                               database::Database& db, database::EntityID eid)
{
  // Call every registered subcomponent parser with the parent element itself.
  for (const auto& subParseFunc : subParseFuncs)
  {
    subParseFunc(doc, parent, db, eid);
  }
}

void parseComponents(const tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, database::Database& db,
                     database::EntityID eid, size_t parseFuncIdx)
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
    parseFuncs[parseFuncIdx](doc, elem, db, eid);
    parseComponentSubElements(doc, elem, db, eid);
  }
}

// Constructor to initialize filename
EntityParser::EntityParser()
{
}

EntityParser::~EntityParser()
{
}

bool EntityParser::loadEntitiesFromFile(const std::string& filename, database::Database& db)
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

bool EntityParser::loadEntitiesFromText(const std::string& text, database::Database& db, const std::string& base_dir)
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

bool EntityParser::loadEntities(tinyxml2::XMLDocument* doc, const std::string& base_dir, database::Database& db)
{
  ObjectIndex object_index;
  buildObjectIndex(doc, object_index, "", base_dir, "[root]", false);

  SceneMap scene_map;
  parseSceneObjects(doc, object_index, scene_map, base_dir);

  for (const auto& scene : scene_map)
  {
    auto eid = db.create();
    components::ObjectComponent obj_comp{ .id = scene.second.id };
    db.add(eid, std::move(obj_comp));

    for (const auto& comp : scene.second.components)
    {
      // Skip components that were deleted or detached
      if (!comp.xml || !comp.xml->GetDocument())
        continue;

      // Skip if the element no longer has its tag (TinyXML may leave stubs)
      if (!comp.xml->Name() || !*comp.xml->Name())
        continue;

      const std::string tag = comp.xml->Name();
      auto opt_type = sceneComponentTypeFromString(tag);  // already exists
      if (!opt_type)
        throw std::runtime_error("Unknown component type for tag <" + tag + ">");

      const size_t index = static_cast<size_t>(*opt_type);  // or via a dedicated map
      if (index >= parseFuncs.size())
        throw std::runtime_error("No parse function for tag <" + tag + ">");
      parseComponents(comp.xml, doc, db, eid, index);
    }
  }
  return true;
}

}  // namespace xml
}  // namespace sodf
