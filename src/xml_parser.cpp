#include <filesystem>
#include <iostream>
#include <set>

#include <sodf/xml_macro.h>
#include <sodf/xml_parser.h>

#include <sodf/component_type.h>
#include <sodf/components/action_map.h>
#include <sodf/components/button.h>
#include <sodf/components/container.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/finite_state_machine.h>
#include <sodf/components/fitting.h>
#include <sodf/components/grasp.h>
#include <sodf/components/joint.h>
#include <sodf/components/link.h>
#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/shape.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/transform.h>

#include "muParser.h"

using namespace tinyxml2;
using namespace sodf::components;
using namespace sodf::geometry;

namespace sodf {

std::string getDirectory(const std::string& filepath)
{
  return std::filesystem::path(filepath).parent_path().string();
}

// Helper for allowed types
bool isSupportedFluidShape(geometry::ShapeType type)
{
  return type == geometry::ShapeType::Box || type == geometry::ShapeType::Cylinder ||
         type == geometry::ShapeType::Sphere || type == geometry::ShapeType::SphericalSegment;
}

// Helper function for unit conversion:
double convertVolumeToSI(double volume, const std::string& units)
{
  if (units == "uL")
    return volume * 1e-9;
  if (units == "mL")
    return volume * 1e-6;
  if (units == "L")
    return volume * 1e-3;
  return volume;  // assume already in m^3
}

// Helper: safely parse required XML double attribute
double parseRequiredDouble(const tinyxml2::XMLElement* elem, const char* attr_name)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    throw std::runtime_error(std::string("Missing attribute '") + attr_name + "' in <Shape> at line " +
                             std::to_string(elem->GetLineNum()));

  try
  {
    mu::Parser parser;
    parser.DefineConst("pi", M_PI);
    parser.DefineConst("inf", std::numeric_limits<double>::infinity());
    parser.SetExpr(expr);
    return parser.Eval();
  }
  catch (mu::Parser::exception_type& e)
  {
    throw std::runtime_error(std::string("Error parsing expression for attribute '") + attr_name + "': " + e.GetMsg());
  }
}

// Helper: safely parse XML double attribute, with a default value if missing
inline double parseDoubleOrDefault(const tinyxml2::XMLElement* elem, const char* attr_name, double default_value)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    return default_value;

  try
  {
    mu::Parser parser;
    parser.DefineConst("pi", M_PI);
    parser.DefineConst("inf", std::numeric_limits<double>::infinity());
    parser.SetExpr(expr);
    return parser.Eval();
  }
  catch (mu::Parser::exception_type& e)
  {
    throw std::runtime_error(std::string("Error parsing expression for attribute '") + attr_name + "': " + e.GetMsg());
  }
}

bool queryRequiredDoubleAttribute(const tinyxml2::XMLElement* elem, const char* attr_name, double* out_value)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    return false;  // Attribute missing

  try
  {
    mu::Parser parser;
    parser.DefineConst("pi", M_PI);
    parser.DefineConst("inf", std::numeric_limits<double>::infinity());
    parser.SetExpr(expr);
    *out_value = parser.Eval();
    return true;
  }
  catch (mu::Parser::exception_type& e)
  {
    throw std::runtime_error(std::string("Error parsing expression for attribute '") + attr_name + "': " + e.GetMsg());
  }
  return true;
}

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

void processInclude(const XMLElement* include_elem, ObjectIndex& object_index, const std::string& including_file_dir,
                    const std::string& parent_ns)
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

// Constructor to initialize filename
XMLParser::XMLParser()
{
}

XMLParser::~XMLParser()
{
}

bool XMLParser::loadEntitiesFromFile(const std::string& filename, ginseng::database& db)
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

bool XMLParser::loadEntitiesFromText(const std::string& text, ginseng::database& db, const std::string& base_dir)
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

bool XMLParser::loadEntities(tinyxml2::XMLDocument* doc, const std::string& base_dir, ginseng::database& db)
{
  ObjectIndex object_index;
  buildObjectIndex(doc, object_index, "", base_dir, "[root]");

  SceneMap scene_map;
  parseSceneObjects(doc, object_index, scene_map);

  for (const auto& scene : scene_map)
  {
    auto eid = db.create_entity();
    ObjectComponent obj_comp{ .id = scene.second.id };
    db.add_component(eid, std::move(obj_comp));

    for (const auto& comp : scene.second.components)
    {
      size_t index = static_cast<size_t>(comp.type);
      if (index >= parseFuncs.size())
        throw std::runtime_error("No parse function for component id: " + comp.id);
      parseFuncs[index](comp.xml, db, eid);
      parseTransformComponent(comp.xml, db, eid);
      parseActionMapComponent(comp.xml, db, eid);
    }
  }
  return true;
}

void parseProductComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  // Note: the object id has already been populated prior this stage
  auto* component = getOrCreateComponent<ObjectComponent>(db, eid);

  if (auto* name = elem->FirstChildElement("Name"))
    component->name = name->GetText();
  if (auto* model = elem->FirstChildElement("Model"))
    component->model = model->GetText();
  if (auto* sn = elem->FirstChildElement("SerialNumber"))
    component->serial_number = sn->GetText();
  if (auto* vendor = elem->FirstChildElement("Vendor"))
    component->vendor = vendor->GetText();
}

void parseOriginComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  // Get id attribute or use "root"
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Origin element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));
  std::string origin_id = id;

  // Count which "case" is present
  int case_count = 0;
  const tinyxml2::XMLElement* tf_elem = nullptr;
  const tinyxml2::XMLElement* align_frames_elem = nullptr;
  const tinyxml2::XMLElement* align_pair_frames_elem = nullptr;

  // Check children: only one is allowed
  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    std::string tag = child->Name();
    if (tag == "Transform")
    {
      tf_elem = child;
      ++case_count;
    }
    else if (tag == "AlignFrames")
    {
      align_frames_elem = child;
      ++case_count;
    }
    else if (tag == "AlignPairFrames")
    {
      align_pair_frames_elem = child;
      ++case_count;
    }
    else
      throw std::runtime_error("Unknown element <" + tag + "> in <Origin> at line " +
                               std::to_string(child->GetLineNum()));
  }

  if (case_count == 0)
    throw std::runtime_error(
        "Origin must contain exactly one of <Transform>, <AlignGeometricPair>, <AlignFrames> at line " +
        std::to_string(elem->GetLineNum()));
  if (case_count > 1)
    throw std::runtime_error(
        "Origin must not contain multiple <Transform>/<AlignGeometricPair>/<AlignFrames> at line " +
        std::to_string(elem->GetLineNum()));

  // Now dispatch:
  if (tf_elem)
  {
    TransformNode frame = parseTransformNode(tf_elem);

    auto* tf_component = getOrCreateComponent<TransformComponent>(db, eid);

    // Overwrite or insert as first entry
    if (tf_component->elements.empty())
      tf_component->elements.emplace_back(origin_id, std::move(frame));
    else
      tf_component->elements[0] = std::make_pair(origin_id, std::move(frame));

    auto* origin_component = getOrCreateComponent<OriginComponent>(db, eid);
    origin_component->origin = Transform{ .parent = frame.parent, .tf = frame.local };
  }
  else if (align_frames_elem)
  {
    AlignFrames align;
    align.target_id = align_frames_elem->Attribute("with");
    const auto* src = align_frames_elem->FirstChildElement("Source");
    const auto* tgt = align_frames_elem->FirstChildElement("Target");
    align.source_tf = src && src->Attribute("name") ? src->Attribute("name") : "";
    align.target_tf = tgt && tgt->Attribute("name") ? tgt->Attribute("name") : "";

    auto* origin_component = getOrCreateComponent<OriginComponent>(db, eid);
    origin_component->origin = align;
  }
  else if (align_pair_frames_elem)
  {
    AlignPairFrames align;
    align.target_id = align_pair_frames_elem->Attribute("with");
    align.tolerance = parseRequiredDouble(align_pair_frames_elem, "tolerance");

    int found = 0;
    for (const auto* tag = align_pair_frames_elem->FirstChildElement(); tag; tag = tag->NextSiblingElement())
    {
      std::string tname = tag->Name();
      const char* name_attr = tag->Attribute("name");
      if (!name_attr)
        throw std::runtime_error("AlignGeometricPair missing 'name' attribute at line " +
                                 std::to_string(tag->GetLineNum()));
      std::string name = name_attr;

      if (tname == "Source" || tname == "SourceTransform")
      {
        if (found == 0)
          align.source_tf1 = name;
        else if (found == 1)
          align.source_tf2 = name;
        else
          throw std::runtime_error("Too many <Source> tags in <AlignGeometricPair> at line " +
                                   std::to_string(tag->GetLineNum()));
      }
      else if (tname == "Target" || tname == "TargetTransform")
      {
        if (found == 2)
          align.target_tf1 = name;
        else if (found == 3)
          align.target_tf2 = name;
        else
          throw std::runtime_error("Too many <Target> tags in <AlignGeometricPair> at line " +
                                   std::to_string(tag->GetLineNum()));
      }
      else
        throw std::runtime_error("Unexpected tag <" + tname + "> in <AlignGeometricPair> at line " +
                                 std::to_string(tag->GetLineNum()));
      found++;
    }
    if (align.source_tf1.empty() || align.source_tf2.empty() || align.target_tf1.empty() || align.target_tf2.empty())
      throw std::runtime_error("Missing source/target transforms in <AlignGeometricPair> at line " +
                               std::to_string(align_pair_frames_elem->GetLineNum()));

    auto* origin_component = getOrCreateComponent<OriginComponent>(db, eid);
    origin_component->origin = align;
  }
}

void parseTransformComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  // Skip if this element is <Origin>
  if (std::strcmp(elem->Name(), "Origin") == 0)
    return;

  if (const auto* transform_elem = elem->FirstChildElement("Transform"))
  {
    const char* id = elem->Attribute("id");
    if (!id)
      throw std::runtime_error("Transform parent element missing 'id' attribute at line " +
                               std::to_string(elem->GetLineNum()));

    std::string id_str(id);

    // Check for duplicates manually if component exists
    if (auto* transform_component = db.get_component<components::TransformComponent*>(eid))
    {
      auto it = std::find_if(transform_component->elements.begin(), transform_component->elements.end(),
                             [&](const auto& p) { return p.first == id_str; });

      if (it != transform_component->elements.end())
      {
        throw std::runtime_error("Duplicate transform id '" + id_str + "' at line " +
                                 std::to_string(elem->GetLineNum()));
      }
    }

    TransformNode frame = parseTransformNode(transform_elem);

    // store to component
    if (std::strcmp(elem->Name(), "Container") == 0)
    {
      // Add bottom transform
      std::string bottom_id = id_str + "/bottom";
      auto* transform_component = getOrCreateComponent<TransformComponent>(db, eid);
      transform_component->elements.emplace_back(bottom_id, frame);

      auto* container_component = db.get_component<ContainerComponent*>(eid);
      if (!container_component)
        throw std::runtime_error("No ContainerComponent found for Container '" + id_str + "'");

      auto container = getComponentElement(container_component->elements, id_str);
      if (!container)
        throw std::runtime_error("No Container entry found for '" + id_str + "' in ContainerComponent");

      // TODO retrieve from geometrical shape or domain shape
      auto* domain_component = db.get_component<DomainShapeComponent*>(eid);
      if (!domain_component)
        throw std::runtime_error("No DomainShapeComponent found for Container '" + id_str + "'");

      const auto domain_shapes = getComponentElement(domain_component->elements, container->domain_shape_id);
      if (!domain_shapes)
        throw std::runtime_error("DomainShapeComponent is empty");

      // // Add surface transform
      // {
      //   std::string surface_id = id_str + "/surface";
      //   double current_height = physics::getFillHeight(*domain_shapes, container->volume, 10e-04);

      //   // Add virtual joint
      //   auto joint = components::Joint{};
      //   joint.type = components::JointType::PRISMATIC;
      //   joint.actuation = components::JointActuation::VIRTUAL;
      //   joint.position = -current_height;
      //   joint.axis = container->axis_bottom;

      //   auto* joint_component = getOrCreateComponent<components::JointComponent>(db, eid);
      //   joint_component->joint_map.emplace_back("joint/" + bottom_id, std::move(joint));

      //   // Add virtual joint transform
      //   TransformNode joint_frame;
      //   joint_frame.is_static = false;  // virtual prismatic joint to simulate liquid height
      //   joint_frame.parent = bottom_id;

      //   transform_component->transform_map.emplace_back("joint/" + bottom_id, std::move(joint_frame));

      //   // Add surface transform
      //   TransformNode surface_frame;
      //   surface_frame.is_static = true;  // virtual prismatic joint to simulate liquid height
      //   surface_frame.parent = "joint/" + bottom_id;

      //   transform_component->transform_map.emplace_back(surface_id, std::move(surface_frame));
      // }

      // // Add top transform
      // {
      //   double max_height = physics::getMaxFillHeight(*domain_shapes);

      //   // The axis points towards the bottom
      //   Eigen::Vector3d top_offset = -container->axis_bottom.normalized() * max_height;
      //   TransformNode top_frame = frame;

      //   top_frame.local.translation().x() += top_offset.x();
      //   top_frame.local.translation().y() += top_offset.y();
      //   top_frame.local.translation().z() += top_offset.z();

      //   std::string top_id = id_str + "/top";
      //   transform_component->transform_map.emplace_back(top_id, std::move(top_frame));
      // }
    }
    else
    {
      auto* component = getOrCreateComponent<TransformComponent>(db, eid);
      component->elements.emplace_back(id_str, std::move(frame));
    }
  }
}

void parseLinkComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  Link link;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Link element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  // --- Visual (must exist and contain a <Shape>)
  const auto* visual = elem->FirstChildElement("Visual");
  if (!visual)
    throw std::runtime_error("Link '" + std::string(id) + "' missing required <Visual> element at line " +
                             std::to_string(elem->GetLineNum()));

  const auto* visual_shape = visual->FirstChildElement("Shape");
  if (!visual_shape)
    throw std::runtime_error("Link '" + std::string(id) + "' <Visual> is missing <Shape> element at line " +
                             std::to_string(visual->GetLineNum()));

  link.visual = parseShape(visual_shape);

  // --- Collision (must exist and contain a <Shape>)
  const auto* collision = elem->FirstChildElement("Collision");
  if (!collision)
    throw std::runtime_error("Link '" + std::string(id) + "' missing required <Collision> element at line " +
                             std::to_string(elem->GetLineNum()));

  const auto* collision_shape = collision->FirstChildElement("Shape");
  if (!collision_shape)
    throw std::runtime_error("Link '" + std::string(id) + "' <Collision> is missing <Shape> element at line " +
                             std::to_string(collision->GetLineNum()));

  link.collision = parseShape(collision_shape);

  // --- Inertial (optional but can be present)
  if (const auto* inertial = elem->FirstChildElement("Inertial"))
  {
    if (const auto* mass = inertial->FirstChildElement("Mass"))
      link.dynamics.mass = parseRequiredDouble(mass, "value");

    if (const auto* com = inertial->FirstChildElement("CenterOfMass"))
      parsePosition(com, link.dynamics.center_of_mass);

    if (const auto* I = inertial->FirstChildElement("InertiaTensor"))
    {
      link.dynamics.inertia_tensor << parseRequiredDouble(I, "ixx"), parseRequiredDouble(I, "ixy"),
          parseRequiredDouble(I, "ixz"), parseRequiredDouble(I, "ixy"), parseRequiredDouble(I, "iyy"),
          parseRequiredDouble(I, "iyz"), parseRequiredDouble(I, "ixz"), parseRequiredDouble(I, "iyz"),
          parseRequiredDouble(I, "izz");
    }
  }

  // --- Store to component
  auto* component = getOrCreateComponent<LinkComponent>(db, eid);
  component->elements.emplace_back(id, std::move(link));
}

void parseFitConstraintComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::FitConstraint fit;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("FitConstraint element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  // --- AxisInsertion (required)
  const auto* axis1 = elem->FirstChildElement("AxisInsertion");
  if (!axis1)
    throw std::runtime_error("FitConstraint '" + std::string(id) + "' missing <AxisInsertion> element at line " +
                             std::to_string(elem->GetLineNum()));
  fit.axis_insertion = parseUnitVector(axis1);

  // --- AxisReference (required)
  const auto* axis2 = elem->FirstChildElement("AxisReference");
  if (!axis2)
    throw std::runtime_error("FitConstraint '" + std::string(id) + "' missing <AxisReference> element at line " +
                             std::to_string(elem->GetLineNum()));
  fit.axis_reference = parseUnitVector(axis2);

  // validate orthonormality
  if (!areVectorsOrthonormal(fit.axis_insertion, fit.axis_reference, 1e-09))
    throw std::runtime_error("FitConstraint '" + std::string(id) + "' axis are not orthogonal at line " +
                             std::to_string(elem->GetLineNum()));

  // --- RotationalSymmetry (required)
  const auto* sym = elem->FirstChildElement("RotationalSymmetry");
  if (!sym)
    throw std::runtime_error("FitConstraint '" + std::string(id) + "' missing <RotationalSymmetry> element at line " +
                             std::to_string(elem->GetLineNum()));
  fit.rotational_symmetry = sym->UnsignedAttribute("value");

  // --- ApproachDistance (required)
  const auto* dist = elem->FirstChildElement("ApproachDistance");
  if (!dist)
    throw std::runtime_error("FitConstraint '" + std::string(id) + "' missing <ApproachDistance> element at line " +
                             std::to_string(elem->GetLineNum()));
  fit.approach_distance = parseRequiredDouble(dist, "value");

  // store to component
  auto* component = getOrCreateComponent<FitConstraintComponent>(db, eid);
  component->elements.emplace_back(id, std::move(fit));
}

void parseTouchscreenComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::Touchscreen ts;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Touchscreen element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  if (const auto* size = elem->FirstChildElement("Size"))
  {
    ts.width = parseRequiredDouble(size, "width");
    ts.height = parseRequiredDouble(size, "height");
  }

  if (const auto* normal = elem->FirstChildElement("SurfaceNormal"))
    ts.surface_normal = parseUnitVector(normal);

  if (const auto* pressure = elem->FirstChildElement("Pressure"))
  {
    ts.min_pressure = parseRequiredDouble(pressure, "min");
    ts.max_pressure = parseRequiredDouble(pressure, "max");
  }

  if (const auto* touch = elem->FirstChildElement("Touch"))
  {
    ts.touch_radius = parseRequiredDouble(touch, "radius");
    ts.multi_touch = touch->BoolAttribute("multi_touch");
    ts.allow_drag = touch->BoolAttribute("allow_drag");
  }

  if (const auto* meta = elem->FirstChildElement("Metadata"))
  {
    if (auto* model = meta->FirstChildElement("Model"))
      ts.model = model->GetText();
    if (auto* version = meta->FirstChildElement("Version"))
      ts.version = version->GetText();
    if (auto* driver = meta->FirstChildElement("Driver"))
      ts.driver = driver->GetText();
  }

  // store to component
  auto* component = getOrCreateComponent<TouchscreenComponent>(db, eid);
  component->elements.emplace_back(id, std::move(ts));
}

void parseFSMComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("FSM element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  components::FSM fsm;

  const auto* start = elem->FirstChildElement("StartState");
  if (!start || !start->Attribute("name"))
    throw std::runtime_error("FSM missing or malformed <StartState> at line " + std::to_string(elem->GetLineNum()));

  std::string start_state = start->Attribute("name");

  // Parse states
  if (const auto* states_elem = elem->FirstChildElement("States"))
  {
    for (const auto* state_elem = states_elem->FirstChildElement("State"); state_elem;
         state_elem = state_elem->NextSiblingElement("State"))
    {
      std::string name = state_elem->Attribute("name");
      fsm.state_labels.add(name);
    }
  }

  if (!fsm.state_labels.has_label(start_state))
    throw std::runtime_error("FSM start state '" + start_state + "' is not listed in <States>.");

  fsm.start_state = fsm.state_labels.to_id(start_state);
  fsm.current_state = fsm.start_state;

  // Parse actions
  if (const auto* actions_elem = elem->FirstChildElement("Actions"))
  {
    for (const auto* action_elem = actions_elem->FirstChildElement("Action"); action_elem;
         action_elem = action_elem->NextSiblingElement("Action"))
    {
      std::string name = action_elem->Attribute("name");
      fsm.action_labels.add(name);
    }
  }

  // Parse transitions
  if (const auto* transitions_elem = elem->FirstChildElement("Transitions"))
  {
    fsm.transitions = buildTransitionTableFromXML(transitions_elem, fsm.state_labels, fsm.action_labels);

    std::cout << "fsm transitions size = " << fsm.transitions.size() << std::endl;
    for (int s = 0; s < fsm.transitions.size(); ++s)
    {
      for (int a = 0; a < fsm.transitions[s].size(); ++a)
      {
        // if (fsm.transitions[s][a] >= 0)
        //   std::cout << "Transition: " << fsm.state_labels.to_string(s) << " + " << fsm.action_labels.to_string(a)
        //             << " -> " << fsm.state_labels.to_string(fsm.transitions[s][a]) << "\n";
      }
    }
  }

  // store to component
  auto* component = getOrCreateComponent<FSMComponent>(db, eid);
  component->elements.emplace_back(id, std::move(fsm));
}

void parseActionMapComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  if (!elem->FirstChildElement("ActionMap"))
    return;

  const char* component_id = elem->Attribute("id");
  if (!component_id)
    throw std::runtime_error("Component missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  for (const tinyxml2::XMLElement* action_map_elem = elem->FirstChildElement("ActionMap"); action_map_elem;
       action_map_elem = action_map_elem->NextSiblingElement("ActionMap"))
  {
    const char* fsm_id = action_map_elem->Attribute("fsm");
    if (!fsm_id)
      throw std::runtime_error("ActionMap missing 'fsm' attribute at line " +
                               std::to_string(action_map_elem->GetLineNum()));

    // Build up ActionMap locally
    components::ActionMap action_map;

    for (const tinyxml2::XMLElement* map_elem = action_map_elem->FirstChildElement("Map"); map_elem;
         map_elem = map_elem->NextSiblingElement("Map"))
    {
      components::ActionMapEntry entry;
      // Required
      const char* trigger = map_elem->Attribute("trigger");
      const char* action = map_elem->Attribute("action");
      if (!trigger || !action)
        throw std::runtime_error("Map missing 'trigger' or 'action' at line " + std::to_string(map_elem->GetLineNum()));

      entry.trigger = trigger;
      entry.action = action;

      // Optional: trigger_params
      const char* trigger_params = map_elem->Attribute("trigger_params");
      if (trigger_params)
        entry.trigger_params = trigger_params;

      entry.component_id = component_id;
      entry.component_type = componentTypeFromString(elem->Name());

      action_map.mappings.push_back(std::move(entry));
    }

    // Only after the entire ActionMap is successfully parsed, mutate ECS
    auto* component = getOrCreateComponent<components::ActionMapComponent>(db, eid);

    // If FSM id already exists, append to mappings
    auto it = std::find_if(component->elements.begin(), component->elements.end(),
                           [fsm_id](const auto& pair) { return pair.first == fsm_id; });

    if (it != component->elements.end())
    {
      it->second.mappings.insert(it->second.mappings.end(), std::make_move_iterator(action_map.mappings.begin()),
                                 std::make_move_iterator(action_map.mappings.end()));
    }
    else
    {
      component->elements.emplace_back(fsm_id, std::move(action_map));
    }
  }
}

void parseContainerComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  Container container;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Container element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  if (const auto* a = elem->FirstChildElement("AxisBottom"))
    container.axis_bottom = parseUnitVector(a);
  else
    throw std::runtime_error("Container missing AxisBottom at line " + std::to_string(elem->GetLineNum()));

  if (const auto* a = elem->FirstChildElement("AxisReference"))
    container.axis_reference = parseUnitVector(a);
  else
    throw std::runtime_error("Container missing AxisReference at line " + std::to_string(elem->GetLineNum()));

  if (!areVectorsOrthonormal(container.axis_bottom, container.axis_reference, 1e-09))
    throw std::runtime_error("Axes are not orthonormal at line " + std::to_string(elem->GetLineNum()));

  // Content (first one only)
  if (const auto* cont = elem->FirstChildElement("Content"))
  {
    if (const char* type = cont->Attribute("type"))
      container.content_type = type;
    double volume = parseDoubleOrDefault(cont, "volume", 0.0);
    const char* units = cont->Attribute("units");
    container.volume = convertVolumeToSI(volume, units ? units : "m^3");
  }

  // Parse <DomainShapeRef>
  if (const tinyxml2::XMLElement* domainShapeRefElem = elem->FirstChildElement("DomainShapeRef"))
  {
    if (const char* domain_id = domainShapeRefElem->Attribute("id"))
    {
      container.domain_shape_id = domain_id;
    }
    else
    {
      throw std::runtime_error("Missing id attribute in <DomainShapeRef> at line " +
                               std::to_string(domainShapeRefElem->GetLineNum()));
    }
  }

  // Parse <LiquidLevelJointRef>
  if (const tinyxml2::XMLElement* liquidLevelJointRefElem = elem->FirstChildElement("LiquidLevelJointRef"))
  {
    if (const char* joint_id = liquidLevelJointRefElem->Attribute("id"))
    {
      container.liquid_level_joint_id = joint_id;
    }
    else
    {
      throw std::runtime_error("Missing id attribute in <LiquidLevelJointRef> at line " +
                               std::to_string(liquidLevelJointRefElem->GetLineNum()));
    }
  }

  // store to component
  auto* component = getOrCreateComponent<ContainerComponent>(db, eid);
  component->elements.emplace_back(id, std::move(container));
}

geometry::Shape parseShape(const tinyxml2::XMLElement* elem)
{
  Shape shape;

  // Type
  const char* type_str = elem->Attribute("type");
  if (!type_str)
    throw std::runtime_error("Shape element missing 'type' attribute at line " + std::to_string(elem->GetLineNum()));
  shape.type = shapeTypeFromString(type_str);

  // --- Handle role attribute for lines ---
  std::string anchor = elem->Attribute("anchor") ? elem->Attribute("anchor") : "";

  // --- Parse by shape type ---
  switch (shape.type)
  {
    case ShapeType::Rectangle:
    {
      // Axes
      Eigen::Vector3d normal, width, height;
      if (const auto* a = elem->FirstChildElement("AxisNormal"))
        normal = parseUnitVector(a);
      else
        throw std::runtime_error("Rectangle missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisWidth"))
        width = parseUnitVector(a);
      else
        throw std::runtime_error("Rectangle missing AxisWidth.");
      if (const auto* a = elem->FirstChildElement("AxisHeight"))
        height = parseUnitVector(a);
      else
        throw std::runtime_error("Rectangle missing AxisHeight.");
      shape.axes = { normal, width, height };
      // Dimensions
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Rectangle missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "width"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      break;
    }
    case ShapeType::Circle:
    {
      Eigen::Vector3d normal, major;
      if (const auto* a = elem->FirstChildElement("AxisNormal"))
        normal = parseUnitVector(a);
      else
        throw std::runtime_error("Circle missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisMajor"))
        major = parseUnitVector(a);
      else
        throw std::runtime_error("Circle missing AxisMajor.");
      shape.axes = { normal, major };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Circle missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "radius"));
      break;
    }
    case ShapeType::Triangle:
    case ShapeType::Polygon:
    {
      Eigen::Vector3d normal, ax, ay;
      if (const auto* a = elem->FirstChildElement("AxisNormal"))
        normal = parseUnitVector(a);
      else
        throw std::runtime_error("Triangle/Polygon missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisX"))
        ax = parseUnitVector(a);
      else
        throw std::runtime_error("Triangle/Polygon missing AxisX.");
      if (const auto* a = elem->FirstChildElement("AxisY"))
        ay = parseUnitVector(a);
      else
        throw std::runtime_error("Triangle/Polygon missing AxisY.");
      shape.axes = { normal, ax, ay };
      // Vertices
      const auto* verts = elem->FirstChildElement("Vertices");
      if (!verts)
        throw std::runtime_error("Triangle/Polygon missing <Vertices>");
      for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
        shape.vertices.emplace_back(parseRequiredDouble(v, "x"), parseRequiredDouble(v, "y"), 0.0);
      if ((shape.type == ShapeType::Triangle && shape.vertices.size() != 3) ||
          (shape.type == ShapeType::Polygon && shape.vertices.size() < 3))
        throw std::runtime_error("Triangle/Polygon has invalid number of vertices.");
      break;
    }
    case ShapeType::Box:
    {
      Eigen::Vector3d ax, ay, az;
      if (const auto* a = elem->FirstChildElement("AxisWidth"))
        ax = parseUnitVector(a);
      else
        throw std::runtime_error("Box missing AxisWidth.");
      if (const auto* a = elem->FirstChildElement("AxisDepth"))
        ay = parseUnitVector(a);
      else
        throw std::runtime_error("Box missing AxisDepth.");
      if (const auto* a = elem->FirstChildElement("AxisHeight"))
        az = parseUnitVector(a);
      else
        throw std::runtime_error("Box missing AxisHeight.");
      shape.axes = { ax, ay, az };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Box missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "width"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "depth"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      break;
    }
    case ShapeType::Cylinder:
    {
      Eigen::Vector3d symmetry, ref;
      if (const auto* a = elem->FirstChildElement("AxisSymmetry"))
        symmetry = parseUnitVector(a);
      else
        throw std::runtime_error("Cylinder missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("AxisReference"))
        ref = parseUnitVector(a);
      else
        throw std::runtime_error("Cylinder missing AxisReference.");
      shape.axes = { symmetry, ref };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Cylinder missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "radius"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      break;
    }
    case ShapeType::Sphere:
    {
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Sphere missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "radius"));
      break;
    }
    case ShapeType::Cone:
    {
      Eigen::Vector3d symmetry, ref;
      if (const auto* a = elem->FirstChildElement("AxisSymmetry"))
        symmetry = parseUnitVector(a);
      else
        throw std::runtime_error("Cone missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("AxisReference"))
        ref = parseUnitVector(a);
      else
        throw std::runtime_error("Cone missing AxisReference.");
      shape.axes = { symmetry, ref };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Cone missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "base_radius"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "top_radius"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      break;
    }
    case ShapeType::SphericalSegment:
    {
      Eigen::Vector3d symmetry, ref;
      if (const auto* a = elem->FirstChildElement("AxisSymmetry"))
        symmetry = parseUnitVector(a);
      else
        throw std::runtime_error("SphericalSegment missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("AxisReference"))
        ref = parseUnitVector(a);
      else
        throw std::runtime_error("SphericalSegment missing AxisReference.");
      shape.axes = { symmetry, ref };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("SphericalSegment missing <Dimensions>");
      shape.dimensions.push_back(parseRequiredDouble(dims, "base_radius"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "top_radius"));
      shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      break;
    }
    case ShapeType::Plane:
    {
      Eigen::Vector3d normal, ax, ay;
      if (const auto* a = elem->FirstChildElement("AxisNormal"))
        normal = parseUnitVector(a);
      else
        throw std::runtime_error("Plane missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisX"))
        ax = parseUnitVector(a);
      else
        throw std::runtime_error("Plane missing AxisX.");
      if (const auto* a = elem->FirstChildElement("AxisY"))
        ay = parseUnitVector(a);
      else
        throw std::runtime_error("Plane missing AxisY.");
      shape.axes = { normal, ax, ay };
      if (const auto* dims = elem->FirstChildElement("Dimensions"))
      {
        shape.dimensions.push_back(parseRequiredDouble(dims, "width"));
        shape.dimensions.push_back(parseRequiredDouble(dims, "height"));
      }
      break;
    }
    case ShapeType::Mesh:
    {
      const auto* file = elem->FirstChildElement("File");
      if (!file || !file->Attribute("path"))
        throw std::runtime_error("Mesh missing <File path=...>");
      shape.mesh_path = file->Attribute("path");
      if (const auto* scale = elem->FirstChildElement("Scale"))
      {
        shape.scale.x() = parseDoubleOrDefault(scale, "x", 1.0);
        shape.scale.y() = parseDoubleOrDefault(scale, "y", 1.0);
        shape.scale.z() = parseDoubleOrDefault(scale, "z", 1.0);
      }
      break;
    }
    case ShapeType::Line:
    {
      const auto* axis_elem = elem->FirstChildElement("AxisDirection");
      const auto* len_elem = elem->FirstChildElement("Length");
      const auto* vtx_elem = elem->FirstChildElement("Vertex");

      bool use_axis_length = (axis_elem && len_elem);
      bool use_vertices = (vtx_elem != nullptr);

      if (use_axis_length && use_vertices)
        throw std::runtime_error("Line: Specify either (AxisDirection + Length) or two Vertex tags, not both.");

      if (use_axis_length)
      {
        // Role is required for Axis+Length, or defaults to "center"
        if (anchor.empty())
          anchor = "center";

        Eigen::Vector3d axis(0, 0, 0);
        axis = parseUnitVector(axis_elem);
        shape.axes.push_back(axis);

        double length = parseRequiredDouble(len_elem, "value");
        if (length <= 0)
          throw std::runtime_error("Line: Length must be positive.");

        if (anchor == "begin")
        {
          // Start: (0,0,0), End: axis * length
          shape.vertices.emplace_back(0.0, 0.0, 0.0);
          shape.vertices.emplace_back(axis.normalized() * length);
        }
        else if (anchor == "center")
        {
          // Centered at (0,0,0): Start: -½axis*length, End: +½axis*length
          shape.vertices.emplace_back(-0.5 * axis.normalized() * length);
          shape.vertices.emplace_back(0.5 * axis.normalized() * length);
        }
        else
        {
          throw std::runtime_error("Line: Unknown anchor '" + anchor + "'. Allowed: begin, center.");
        }
        // Optionally: Store a "is_2d" flag if z is 0 for both endpoints
      }
      else if (use_vertices)
      {
        std::vector<Eigen::Vector3d> verts;
        for (const auto* v = vtx_elem; v; v = v->NextSiblingElement("Vertex"))
        {
          double x = parseRequiredDouble(v, "x");
          double y = parseRequiredDouble(v, "y");
          double z = 0.0;
          // If z attribute present, use it (else default to 0.0)
          queryRequiredDoubleAttribute(v, "z", &z);
          verts.emplace_back(x, y, z);
        }
        if (verts.size() != 2)
          throw std::runtime_error("Line: Must have exactly 2 Vertex tags.");

        bool is2d = (verts[0].z() == 0.0 && verts[1].z() == 0.0);
        bool is3d = (verts[0].z() != 0.0 || verts[1].z() != 0.0);
        if (!is2d && !is3d)
          throw std::runtime_error("Line: Vertices must either both have z=0 or both specify z values.");
        shape.vertices = std::move(verts);
        // Optionally: Store is2d flag for later logic
      }
      else
      {
        throw std::runtime_error("Line: Must specify either (AxisDirection + Length) or two Vertex tags.");
      }
      break;
    }

    default:
      throw std::runtime_error("Unknown shape type in parseShapeElement.");
  }

  // validate axes orthonormality
  if (shape.axes.size() == 1)
  {
    if (!isUnitVector(shape.axes[0]))
      throw std::runtime_error("Shape axis must be a unit vector at line " + std::to_string(elem->GetLineNum()));
  }
  else if (shape.axes.size() == 2)
  {
    if (!areVectorsOrthonormal(shape.axes[0], shape.axes[1], 1e-09))
      throw std::runtime_error("Shape axes are not orthonormal at line " + std::to_string(elem->GetLineNum()));
  }
  else if (shape.axes.size() == 3)
  {
    if (!areVectorsOrthonormal(shape.axes[0], shape.axes[1], shape.axes[2], 1e-09))
      throw std::runtime_error("Shape axes are not orthonormal at line " + std::to_string(elem->GetLineNum()));
  }
  return shape;
}

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos)
{
  pos.x() = parseRequiredDouble(element, "x");
  pos.y() = parseRequiredDouble(element, "y");
  pos.z() = parseRequiredDouble(element, "z");
}

void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  double roll = parseRequiredDouble(element, "roll");
  double pitch = parseRequiredDouble(element, "pitch");
  double yaw = parseRequiredDouble(element, "yaw");
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  q.x() = parseRequiredDouble(element, "x");
  q.y() = parseRequiredDouble(element, "y");
  q.z() = parseRequiredDouble(element, "z");
  q.w() = parseRequiredDouble(element, "w");
}

Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* transform_elem)
{
  Eigen::Vector3d pos{ 0, 0, 0 };
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  if (const auto* pos_elem = transform_elem->FirstChildElement("Position"))
    parsePosition(pos_elem, pos);

  if (const auto* ori_elem = transform_elem->FirstChildElement("Orientation"))
    parseOrientationRPY(ori_elem, q);
  else if (const auto* quat_elem = transform_elem->FirstChildElement("Quaternion"))
  {
    parseQuaternion(quat_elem, q);
  }

  return Eigen::Translation3d(pos) * q;
}

geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* transform_elem)
{
  TransformNode frame;
  if (const char* parent = transform_elem->Attribute("parent"))
    frame.parent = std::string(parent);

  frame.local = parseIsometry3D(transform_elem);
  frame.global = Eigen::Isometry3d::Identity();
  frame.dirty = true;

  return frame;
}

Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* element, double epsilon)
{
  Eigen::Vector3d vec;

  vec.x() = parseRequiredDouble(element, "x");
  vec.y() = parseRequiredDouble(element, "y");
  vec.z() = parseRequiredDouble(element, "z");

  double norm = vec.norm();
  if (std::abs(norm - 1.0) > epsilon)
  {
    std::ostringstream oss;
    oss << "Expected unit vector but got vector with norm " << norm << " at line " << element->GetLineNum();
    throw std::runtime_error(oss.str());
  }
  return vec;
}

std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* trans_elem,
                                                          const FSMLabels& state_labels, const FSMLabels& action_labels,
                                                          int invalid_value)
{
  const int num_states = static_cast<int>(state_labels.id_to_string.size());
  const int num_actions = static_cast<int>(action_labels.id_to_string.size());

  std::vector<std::vector<int>> transitions(num_states, std::vector<int>(num_actions, invalid_value));

  for (const tinyxml2::XMLElement* tr_elem = trans_elem->FirstChildElement("Transition"); tr_elem;
       tr_elem = tr_elem->NextSiblingElement("Transition"))
  {
    std::string from = tr_elem->Attribute("from");
    std::string action = tr_elem->Attribute("action");
    std::string to = tr_elem->Attribute("to");

    if (!state_labels.has_label(from) || !state_labels.has_label(to))
      throw std::runtime_error("Transition refers to undefined state at line " + std::to_string(tr_elem->GetLineNum()));
    if (!action_labels.has_label(action))
      throw std::runtime_error("Transition refers to undefined action at line " + std::to_string(tr_elem->GetLineNum()));

    int from_id = state_labels.to_id(from);
    int to_id = state_labels.to_id(to);
    int action_id = action_labels.to_id(action);

    if (transitions[from_id][action_id] != invalid_value)
    {
      throw std::runtime_error("Duplicate transition from='" + from + "' action='" + action + "' at line " +
                               std::to_string(tr_elem->GetLineNum()));
    }

    transitions[from_id][action_id] = to_id;
  }

  return transitions;
}

ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* elem, const ShapeComponent& shape_component,
                                 const TransformComponent& transform_component, TransformNode& transform)
{
  ParallelGrasp grasp;

  // -------- Case 1: Fully Defined --------
  if (const auto* transform_elem = elem->FirstChildElement("Transform"))
  {
    transform = parseTransformNode(transform_elem);

    // Approach
    const auto* approach_elem = elem->FirstChildElement("Approach");
    if (!approach_elem || !approach_elem->Attribute("value"))
      throw std::runtime_error("ParallelGrasp: <Approach> tag with 'value' is required.");
    std::string approach_val = approach_elem->Attribute("value");
    if (approach_val == "INTERNAL")
      grasp.approach = ParallelGrasp::ApproachType::INTERNAL;
    else if (approach_val == "EXTERNAL")
      grasp.approach = ParallelGrasp::ApproachType::EXTERNAL;
    else
      throw std::runtime_error("ParallelGrasp: <Approach> value must be 'INTERNAL' or 'EXTERNAL'");

    // Gap size
    const auto* gap_elem = elem->FirstChildElement("GapSize");
    grasp.gap_size = gap_elem ? parseRequiredDouble(gap_elem, "value") : 0.0;

    // Rotational axis/symmetry
    grasp.axis_of_rotation = parseUnitVector(elem->FirstChildElement("RotationalAxis"));
    const auto* sym_elem = elem->FirstChildElement("RotationalSymmetry");
    grasp.rotational_symmetry = sym_elem ? sym_elem->UnsignedAttribute("value") : 1;

    // Real/contact surfaces (shape ids)
    grasp.contact_shape_ids.clear();
    if (const auto* reals = elem->FirstChildElement("RealSurfaces"))
    {
      for (const auto* surf = reals->FirstChildElement("Shape"); surf; surf = surf->NextSiblingElement("Shape"))
      {
        if (surf->Attribute("id"))
          grasp.contact_shape_ids.push_back(surf->Attribute("id"));
      }
    }

    // Canonical/virtual surface
    grasp.canonical_surface = parseShape(elem->FirstChildElement("VirtualSurface"));

    return grasp;
  }

  // -------- Case 2: Derived from shapes --------
  if (const auto* derived_elem = elem->FirstChildElement("DerivedFromParallelShapes"))
  {
    // Required axes
    const auto* axis_rot_first_elem = derived_elem->FirstChildElement("AxisRotationalFirstShape");
    const auto* axis_norm_first_elem = derived_elem->FirstChildElement("AxisNormalFirstShape");
    const auto* axis_rot_grasp_elem = derived_elem->FirstChildElement("AxisRotationalGrasp");
    if (!axis_rot_first_elem || !axis_norm_first_elem || !axis_rot_grasp_elem)
      throw std::runtime_error("ParallelGrasp <DerivedFromParallelShapes> missing required axis elements "
                               "(AxisRotationalFirstShape, AxisNormalFirstShape, AxisRotationalGrasp required).");

    Eigen::Vector3d axis_rotational_first = parseUnitVector(axis_rot_first_elem);
    Eigen::Vector3d axis_normal_first = parseUnitVector(axis_norm_first_elem);
    Eigen::Vector3d axis_rotational_grasp = parseUnitVector(axis_rot_grasp_elem);

    // Collect shape pointers and ids
    std::vector<const geometry::Shape*> shape_ptrs;
    std::vector<std::string> shape_ids;
    for (const auto* shape_elem = derived_elem->FirstChildElement("Shape"); shape_elem;
         shape_elem = shape_elem->NextSiblingElement("Shape"))
    {
      if (!shape_elem->Attribute("id"))
        throw std::runtime_error("ParallelGrasp <Shape> missing 'id' attribute");
      std::string shape_id = shape_elem->Attribute("id");
      auto shape = getComponentElement(shape_component.elements, shape_id);
      if (!shape)
        throw std::runtime_error("No Shape entry found for '" + shape_id + "' in ShapeComponent");
      shape_ptrs.push_back(shape);
      shape_ids.push_back(shape_id);
    }

    // ---- 2D shape pairs ----
    if (shape_ptrs.size() >= 2 && shape_ptrs.size() % 2 == 0)
    {
      const double CENTROID_TOL = 1e-6;
      size_t num_pairs = shape_ptrs.size() / 2;

      Eigen::Vector3d ref_centroid;
      for (size_t i = 0; i < shape_ptrs.size(); i += 2)
      {
        // Check both are 2D
        if (!is2DShape(*shape_ptrs[i]) || !is2DShape(*shape_ptrs[i + 1]))
          throw std::runtime_error("ParallelGrasp: Only 2D shapes allowed in even-pair case.");
        // Get transforms
        auto tf0 = getComponentElement(transform_component.elements, shape_ids[i]);
        auto tf1 = getComponentElement(transform_component.elements, shape_ids[i + 1]);
        if (!tf0 || !tf1)
          throw std::runtime_error("ParallelGrasp: Shape transform missing in TransformComponent.");

        Eigen::Vector3d c0 = tf0->local * getShapeCentroid(*shape_ptrs[i]);
        Eigen::Vector3d c1 = tf1->local * getShapeCentroid(*shape_ptrs[i + 1]);
        Eigen::Vector3d n0 = tf0->local.linear() * getShapeNormalAxis(*shape_ptrs[i]);
        Eigen::Vector3d n1 = tf1->local.linear() * getShapeNormalAxis(*shape_ptrs[i + 1]);

        // Normals must be parallel
        if (n0.cross(n1).norm() > 1e-6)
          throw std::runtime_error("2D shapes are not parallel (normals differ)");

        // Centroids must align in-plane
        Eigen::Vector3d delta = c1 - c0;
        double projected = delta.dot(n0.normalized());
        Eigen::Vector3d delta_in_plane = delta - projected * n0.normalized();
        if (delta_in_plane.norm() > CENTROID_TOL)
          throw std::runtime_error("2D shape centroids do not align in plane");

        // Save contact shapes for the first pair
        if (i == 0)
        {
          grasp.contact_shape_ids = { shape_ids[i], shape_ids[i + 1] };
          grasp.gap_size = std::abs((c1 - c0).dot(n0.normalized()));
        }

        if (i == 0)
          ref_centroid = 0.5 * (c0 + c1);
        else
        {
          Eigen::Vector3d centroid = 0.5 * (c0 + c1);
          if (!centroid.isApprox(ref_centroid, CENTROID_TOL))
            throw std::runtime_error("ParallelGrasp: Centroid mismatch for 2D shape pair");
        }
      }
      // Transform/canonical frame: use axis_rotational, axis_normal, centroid
      Eigen::Vector3d x_axis = axis_rotational_first.normalized();
      Eigen::Vector3d z_axis = axis_normal_first.normalized();
      Eigen::Vector3d y_axis = z_axis.cross(x_axis);
      Eigen::Matrix3d rot = computeOrientationFromAxes(x_axis, y_axis, z_axis);

      transform.local = Eigen::Isometry3d::Identity();
      transform.local.linear() = rot;
      transform.local.translation() = ref_centroid;

      std::cout << "tf = \n" << transform.local.matrix() << std::endl;

      grasp.axis_of_rotation = axis_rotational_grasp;
      // grasp.axis_of_rotation =
      //     rot * axis_rotational_grasp.normalized();  // Always the X axis in the grasp canonical frame
      grasp.canonical_surface = *shape_ptrs[0];  // Or construct a new "virtual" surface here if needed

      // Build axes for the canonical surface in the grasp frame
      std::vector<Eigen::Vector3d> canonical_axes_in_grasp;
      for (const Eigen::Vector3d& axis : shape_ptrs[0]->axes)
      {
        // Bring each shape axis into the grasp frame
        // If the original axis is in the parent/world, this is how you transform:
        Eigen::Vector3d axis_in_grasp = rot.transpose() * axis;
        canonical_axes_in_grasp.push_back(axis_in_grasp);
      }
      grasp.canonical_surface.axes = canonical_axes_in_grasp;

      // Rotational symmetry: pairs * 2
      grasp.rotational_symmetry = static_cast<uint32_t>(num_pairs * 2);

      // Approach: use the sign between reference centroid and contact normal
      Eigen::Vector3d to_virtual =
          ref_centroid -
          (getComponentElement(transform_component.elements, shape_ids[0])->local * getShapeCentroid(*shape_ptrs[0]));
      Eigen::Vector3d normal = rot.col(2);

      std::cout << "to_virtual: " << to_virtual.transpose() << "  normal: " << normal.transpose()
                << "  dot: " << to_virtual.dot(normal) << std::endl;

      auto tf0 = getComponentElement(transform_component.elements, shape_ids[0]);
      Eigen::Vector3d normal_contact = tf0->local.linear() * getShapeNormalAxis(*shape_ptrs[0]);

      grasp.approach = (to_virtual.dot(normal_contact) > 0) ? ParallelGrasp::ApproachType::INTERNAL :
                                                              ParallelGrasp::ApproachType::EXTERNAL;

      // grasp.approach =
      //     (to_virtual.dot(normal) > 0) ? ParallelGrasp::ApproachType::INTERNAL : ParallelGrasp::ApproachType::EXTERNAL;

      return grasp;
    }

    // ---- 3D shape, single (cylinder/box) ----
    if (shape_ptrs.size() == 1 && (shape_ptrs[0]->type == ShapeType::Cylinder || shape_ptrs[0]->type == ShapeType::Box))
    {
      grasp.contact_shape_ids = { shape_ids[0] };
      auto shape_tf = getComponentElement(transform_component.elements, shape_ids[0]);
      if (!shape_tf)
        throw std::runtime_error("No Transform entry found for '" + shape_ids[0] + "' in TransformComponent");

      Eigen::Vector3d axis_rotational_local = axis_rotational_first.normalized();
      Eigen::Vector3d axis_normal_local = axis_normal_first.normalized();
      Eigen::Vector3d axis_inplane_local = axis_normal_local.cross(axis_rotational_local).normalized();

      const Eigen::Matrix3d& shape3d_to_parent = shape_tf->local.linear();
      Eigen::Vector3d axis_rotational_root = shape3d_to_parent * axis_rotational_local;
      Eigen::Vector3d axis_normal_root = shape3d_to_parent * axis_normal_local;
      Eigen::Vector3d axis_inplane_root = shape3d_to_parent * axis_inplane_local;
      Eigen::Vector3d centroid_3dshape = getShapeCentroid(*shape_ptrs[0]);
      Eigen::Vector3d centroid_root = shape_tf->local * centroid_3dshape;

      transform.local.setIdentity();
      transform.local.linear().col(0) = axis_rotational_root;
      transform.local.linear().col(1) = axis_inplane_root;
      transform.local.linear().col(2) = axis_normal_root;
      transform.local.translation() = centroid_root;

      grasp.axis_of_rotation = axis_rotational_grasp;

      Eigen::Matrix3d grasp_to_world = transform.local.linear();
      Eigen::Matrix3d world_to_grasp = grasp_to_world.transpose();  // since R is orthogonal
      const auto& shape_axes = shape_ptrs[0]->axes;                 // {AxisWidth, AxisDepth, AxisHeight} in shape-local
      Eigen::Matrix3d shape_to_world = shape3d_to_parent;
      Eigen::Matrix3d shape_to_grasp = world_to_grasp * shape_to_world;

      // Canonical surface: for cylinder, a line; for box, a rectangle
      if (shape_ptrs[0]->type == ShapeType::Cylinder)
      {
        grasp.canonical_surface.type = ShapeType::Line;
        double length = shape_ptrs[0]->dimensions.at(1);
        grasp.canonical_surface.dimensions = { length };
        Eigen::Vector3d p0_local = -0.5 * length * Eigen::Vector3d::UnitX();
        Eigen::Vector3d p1_local = 0.5 * length * Eigen::Vector3d::UnitX();
        grasp.canonical_surface.vertices = { p0_local, p1_local };
        // grasp.canonical_surface.axes = { Eigen::Vector3d::UnitX() };
        grasp.rotational_symmetry = 0;  // infinite
        grasp.gap_size = 2.0 * shape_ptrs[0]->dimensions.at(0);

        grasp.canonical_surface.axes.clear();
        for (const auto& axis : shape_axes)
          grasp.canonical_surface.axes.push_back(shape_to_grasp * axis);
      }
      else if (shape_ptrs[0]->type == ShapeType::Box)
      {
        double width = shape_ptrs[0]->dimensions.at(0);
        double height = shape_ptrs[0]->dimensions.at(2);
        grasp.canonical_surface.type = ShapeType::Rectangle;
        grasp.canonical_surface.dimensions = { width, height };
        // Centered at origin in local grasp frame
        Eigen::Vector3d corner0(-0.5 * width, -0.5 * height, 0);
        Eigen::Vector3d corner1(0.5 * width, -0.5 * height, 0);
        Eigen::Vector3d corner2(0.5 * width, 0.5 * height, 0);
        Eigen::Vector3d corner3(-0.5 * width, 0.5 * height, 0);
        grasp.canonical_surface.vertices = { corner0, corner1, corner2, corner3 };
        // grasp.canonical_surface.axes = { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY() };
        grasp.rotational_symmetry = 4;
        grasp.gap_size = width;

        // Convert each box axis to grasp frame:
        grasp.canonical_surface.axes.clear();
        for (const auto& box_axis : shape_axes)
          grasp.canonical_surface.axes.push_back(shape_to_grasp * box_axis);
      }

      // Approach: must be specified
      const auto* approach_elem = derived_elem->FirstChildElement("Approach");
      if (!approach_elem || !approach_elem->Attribute("value"))
        throw std::runtime_error(
            "ParallelGrasp: <Approach> tag with 'value' attribute is required for single 3D shape (box/cylinder).");
      std::string approach_str = approach_elem->Attribute("value");
      if (approach_str == "INTERNAL" || approach_str == "Internal")
        grasp.approach = ParallelGrasp::ApproachType::INTERNAL;
      else if (approach_str == "EXTERNAL" || approach_str == "External")
        grasp.approach = ParallelGrasp::ApproachType::EXTERNAL;
      else
        throw std::runtime_error("ParallelGrasp: <Approach> value must be 'INTERNAL' or 'EXTERNAL'");

      return grasp;
    }

    throw std::runtime_error(
        "ParallelGrasp: Must derive from exactly two 2D shapes (parallel), or one 3D lateral shape (cylinder/box).");
  }

  throw std::runtime_error("Could not parse ParallelGrasp: missing required elements.");
}

void parseParallelGraspComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("ParallelGrasp element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  auto shape_component = db.get_component<ShapeComponent*>(eid);
  if (!shape_component)
    throw std::runtime_error("ShapeComponent does not exists, required by ParallelGraspComponent");

  auto transform_component = db.get_component<TransformComponent*>(eid);
  if (!transform_component)
    throw std::runtime_error("TransformComponent does not exists, required by ParallelGraspComponent");

  TransformNode tf;
  auto grasp = parseParallelGrasp(elem, *shape_component, *transform_component, tf);

  transform_component->elements.emplace_back(id, std::move(tf));

  auto* grasp_component = getOrCreateComponent<ParallelGraspComponent>(db, eid);
  grasp_component->elements.emplace_back(id, std::move(grasp));
}

void parseShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Shape element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  auto shape = parseShape(elem);

  auto* shape_component = getOrCreateComponent<ShapeComponent>(db, eid);
  shape_component->elements.emplace_back(id, std::move(shape));
}

StackedShapes parseStackedShape(const tinyxml2::XMLElement* stacked_elem)
{
  StackedShapes stack;
  Eigen::Vector3d stack_point = Eigen::Vector3d::Zero();  // Current stacking position
  Eigen::Vector3d stack_axis = Eigen::Vector3d::UnitZ();  // Default stacking axis (may be set per shape)
  bool first_shape = true;

  for (const tinyxml2::XMLElement* shape_elem = stacked_elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    // Parse geometry with your existing function
    Shape shape = parseShape(shape_elem);

    const auto* trans_elem = shape_elem->FirstChildElement("Transform");
    Eigen::Isometry3d local_tf = Eigen::Isometry3d::Identity();

    if (trans_elem)
    {
      // Manual position: use transform directly
      local_tf = parseIsometry3D(trans_elem);
    }
    else
    {
      // Automatic stacking
      Eigen::Vector3d axis = getShapeSymmetryAxis(shape);

      if (first_shape)
      {
        // Place base at origin
        local_tf.translation() = Eigen::Vector3d::Zero();
        local_tf.linear() = buildIsometry(Eigen::Vector3d::Zero(), axis).linear();
        // Next stack point: base + axis * height
        stack_point = axis * shapeHeight(shape);
        stack_axis = axis;
        first_shape = false;
      }
      else
      {
        // Place base at current stack_point
        local_tf.translation() = stack_point;
        local_tf.linear() = buildIsometry(stack_point, axis).linear();
        // Update stack point for next shape
        stack_point = stack_point + axis * shapeHeight(shape);
        stack_axis = axis;
      }
    }

    // Add shape to stack
    stack.push_back({ shape, local_tf });
  }

  return stack;
}

void parseStackedShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("StackedShape element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  auto stack = parseStackedShape(elem);
  if (stack.empty())
    throw std::runtime_error("StackedShape '" + std::string(id) + "' is empty at line " +
                             std::to_string(elem->GetLineNum()));

  auto* stacked_shape_component = getOrCreateComponent<StackedShapeComponent>(db, eid);
  stacked_shape_component->elements.emplace_back(id, std::move(stack));
}

Button parseButton(const tinyxml2::XMLElement* btn_elem)
{
  using namespace sodf::components;

  const char* type_str = btn_elem->Attribute("type");
  if (!type_str)
    throw std::runtime_error("Button element missing 'type' attribute at line " +
                             std::to_string(btn_elem->GetLineNum()));

  Button btn;
  btn.type = buttonTypeFromString(type_str);

  // Parse required <Link ref="..."/>
  const tinyxml2::XMLElement* link_elem = btn_elem->FirstChildElement("Link");
  if (!link_elem || !link_elem->Attribute("ref"))
    throw std::runtime_error("Button missing <Link ref=\"...\"/> element at line " +
                             std::to_string(btn_elem->GetLineNum()));
  btn.link_id = link_elem->Attribute("ref");

  // Parse required <Joint ref="..."/>
  const tinyxml2::XMLElement* joint_elem = btn_elem->FirstChildElement("Joint");
  if (!joint_elem || !joint_elem->Attribute("ref"))
    throw std::runtime_error("Button missing <Joint ref=\"...\"/> element at line " +
                             std::to_string(btn_elem->GetLineNum()));
  btn.joint_id = joint_elem->Attribute("ref");

  // Optional: <DetentSpacing value="..."/>
  if (const tinyxml2::XMLElement* detent_elem = btn_elem->FirstChildElement("DetentSpacing"))
  {
    queryRequiredDoubleAttribute(detent_elem, "value", &btn.detent_spacing);
  }

  // Optional: <RestPositions>
  if (const tinyxml2::XMLElement* rest_elem = btn_elem->FirstChildElement("RestPositions"))
  {
    for (const tinyxml2::XMLElement* pos_elem = rest_elem->FirstChildElement("Position"); pos_elem;
         pos_elem = pos_elem->NextSiblingElement("Position"))
    {
      double val = 0.0;
      if (queryRequiredDoubleAttribute(pos_elem, "value", &val))
        btn.rest_positions.push_back(val);
    }
  }

  // Optional: <StiffnessProfile>
  if (const tinyxml2::XMLElement* stiff_elem = btn_elem->FirstChildElement("StiffnessProfile"))
  {
    for (const tinyxml2::XMLElement* s_elem = stiff_elem->FirstChildElement("Stiffness"); s_elem;
         s_elem = s_elem->NextSiblingElement("Stiffness"))
    {
      double val = 0.0;
      if (queryRequiredDoubleAttribute(s_elem, "value", &val))
        btn.stiffness_profile.push_back(val);
    }
  }

  return btn;
}

void parseButtonComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Button element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  auto btn = parseButton(elem);

  // validate link + joint exists in database
  {
    auto component = db.get_component<LinkComponent*>(eid);
    if (!component)
      throw std::runtime_error("LinkComponent does not exists, required by ButtonComponent");

    auto link = getComponentElement(component->elements, btn.link_id);
    if (!link)
      throw std::runtime_error("No Link entry found for '" + btn.link_id + "' in LinkComponent");
  }
  {
    auto component = db.get_component<JointComponent*>(eid);
    if (!component)
      throw std::runtime_error("JointComponent does not exists, required by ButtonComponent");

    auto joint = getComponentElement(component->elements, btn.joint_id);
    if (!joint)
      throw std::runtime_error("No Joint entry found for '" + btn.joint_id + "' in JointComponent");
  }

  auto* btn_component = getOrCreateComponent<ButtonComponent>(db, eid);
  btn_component->elements.emplace_back(id, std::move(btn));
}

VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem)
{
  using namespace sodf::components;

  VirtualButton vbtn;

  // Required: <Shape ref="..."/>
  const tinyxml2::XMLElement* shape_elem = vb_elem->FirstChildElement("Shape");
  if (!shape_elem || !shape_elem->Attribute("id"))
    throw std::runtime_error("VirtualButton missing <Shape id=\"...\"/> at line " +
                             std::to_string(vb_elem->GetLineNum()));
  vbtn.shape_id = shape_elem->Attribute("id");

  // Required: <PressAxis x="..." y="..." z="..."/>
  const tinyxml2::XMLElement* axis_elem = vb_elem->FirstChildElement("AxisPress");
  if (!axis_elem)
    throw std::runtime_error("VirtualButton missing <AxisPress> at line " + std::to_string(vb_elem->GetLineNum()));
  double x = 0, y = 0, z = 0;
  if (!queryRequiredDoubleAttribute(axis_elem, "x", &x) || !queryRequiredDoubleAttribute(axis_elem, "y", &y) ||
      !queryRequiredDoubleAttribute(axis_elem, "z", &z))
    throw std::runtime_error("VirtualButton <AxisPress> missing or invalid x/y/z attribute at line " +
                             std::to_string(axis_elem->GetLineNum()));
  vbtn.press_axis = Eigen::Vector3d(x, y, z);

  // validate unit vector
  if (!isUnitVector(vbtn.press_axis))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  // Optional: <Label text="..."/>
  if (const tinyxml2::XMLElement* label_elem = vb_elem->FirstChildElement("Label"))
  {
    if (const char* txt = label_elem->Attribute("text"))
      vbtn.label = txt;
  }

  // Optional: <Image uri="..."/>
  if (const tinyxml2::XMLElement* img_elem = vb_elem->FirstChildElement("Image"))
  {
    if (const char* uri = img_elem->Attribute("uri"))
      vbtn.image_uri = uri;
  }

  return vbtn;
}

void parseVirtualButtonComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("VirtualButton element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  auto button = parseVirtualButton(elem);

  auto* btn_component = getOrCreateComponent<VirtualButtonComponent>(db, eid);
  btn_component->elements.emplace_back(id, std::move(button));
}

Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  Joint joint;

  // Type & actuation
  const char* type_str = joint_elem->FirstChildElement("Type")->Attribute("value");
  const char* act_str = joint_elem->FirstChildElement("Actuation")->Attribute("value");
  joint.type = jointTypeFromString(type_str);
  joint.actuation = jointActuationFromString(act_str);
  joint.initialize_for_type();

  // Axis
  const auto* axis_elem = joint_elem->FirstChildElement("Axis");
  if (!axis_elem)
    throw std::runtime_error("Missing <Axis>");
  double x = 0, y = 0, z = 0;
  queryRequiredDoubleAttribute(axis_elem, "x", &x);
  queryRequiredDoubleAttribute(axis_elem, "y", &y);
  queryRequiredDoubleAttribute(axis_elem, "z", &z);
  joint.axes.col(0) = Eigen::Vector3d(x, y, z);

  // validate unit vector
  if (!isUnitVector(joint.axes.col(0)))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  // States
  joint.position(0) = joint_elem->FirstChildElement("Position")->DoubleAttribute("value");
  // Velocity
  if (const auto* vel_elem = joint_elem->FirstChildElement("Velocity"))
    joint.velocity(0) = parseRequiredDouble(vel_elem, "value");

  // Effort
  if (const auto* eff_elem = joint_elem->FirstChildElement("Effort"))
    joint.effort(0) = parseRequiredDouble(eff_elem, "value");

  // Limits
  if (const auto* limit_elem = joint_elem->FirstChildElement("Limit"))
  {
    if (const auto* pos = limit_elem->FirstChildElement("Position"))
    {
      if (pos->Attribute("min"))
        joint.limit.min_position(0) = parseRequiredDouble(pos, "min");
      if (pos->Attribute("max"))
        joint.limit.max_position(0) = parseRequiredDouble(pos, "max");
    }
    if (const auto* vel = limit_elem->FirstChildElement("Velocity"))
    {
      if (vel->Attribute("min"))
        joint.limit.min_velocity(0) = parseRequiredDouble(vel, "min");
      if (vel->Attribute("max"))
        joint.limit.max_velocity(0) = parseRequiredDouble(vel, "max");
    }
    if (const auto* acc = limit_elem->FirstChildElement("Acceleration"))
    {
      if (acc->Attribute("min"))
        joint.limit.min_acceleration(0) = parseRequiredDouble(acc, "min");
      if (acc->Attribute("max"))
        joint.limit.max_acceleration(0) = parseRequiredDouble(acc, "max");
    }
    if (const auto* jerk = limit_elem->FirstChildElement("Jerk"))
    {
      if (jerk->Attribute("min"))
        joint.limit.min_jerk(0) = parseRequiredDouble(jerk, "min");
      if (jerk->Attribute("max"))
        joint.limit.max_jerk(0) = parseRequiredDouble(jerk, "max");
    }
    if (const auto* eff = limit_elem->FirstChildElement("Effort"))
    {
      if (eff->Attribute("min"))
        joint.limit.min_effort(0) = parseRequiredDouble(eff, "min");
      if (eff->Attribute("max"))
        joint.limit.max_effort(0) = parseRequiredDouble(eff, "max");
    }
  }

  // Dynamics
  if (const auto* dyn_elem = joint_elem->FirstChildElement("Dynamics"))
  {
    if (const auto* s = dyn_elem->FirstChildElement("Stiffness"))
      joint.dynamics.stiffness(0) = parseRequiredDouble(s, "value");
    if (const auto* d = dyn_elem->FirstChildElement("Damping"))
      joint.dynamics.damping(0) = parseRequiredDouble(d, "value");
    if (const auto* r = dyn_elem->FirstChildElement("RestPosition"))
      joint.dynamics.rest_position[0] =
          r->Attribute("value") ? std::optional<double>{ parseRequiredDouble(r, "value") } : std::nullopt;
    if (const auto* i = dyn_elem->FirstChildElement("Inertia"))
      joint.dynamics.inertia(0) = parseRequiredDouble(i, "value");
  }
  return joint;
}

Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  Joint joint;

  // Type & actuation
  const char* type_str = joint_elem->FirstChildElement("Type")->Attribute("value");
  const char* act_str = joint_elem->FirstChildElement("Actuation")->Attribute("value");
  joint.type = jointTypeFromString(type_str);
  joint.actuation = jointActuationFromString(act_str);
  int dof = joint.dof();
  joint.initialize_for_type();

  // Axes
  const auto* axes_elem = joint_elem->FirstChildElement("Axes");
  int axis_i = 0;
  for (const auto* axis_elem = axes_elem ? axes_elem->FirstChildElement("Axis") : nullptr; axis_elem && axis_i < dof;
       axis_elem = axis_elem->NextSiblingElement("Axis"), ++axis_i)
  {
    double x = 0, y = 0, z = 0;
    queryRequiredDoubleAttribute(axis_elem, "x", &x);
    queryRequiredDoubleAttribute(axis_elem, "y", &y);
    queryRequiredDoubleAttribute(axis_elem, "z", &z);
    joint.axes.col(axis_i) = Eigen::Vector3d(x, y, z);
  }

  int num_cols = joint.axes.cols();
  if (num_cols != dof)
    throw std::runtime_error("There must be " + std::to_string(dof) + " <Axis> tags for a " + type_str +
                             " joint at line " + std::to_string(axes_elem->GetLineNum()));

  // Validate axes are orthonormal as required for this joint type
  switch (joint.type)
  {
    case JointType::FIXED:
      // No motion, no axes needed
      break;
    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      if (!isUnitVector(joint.axes.col(0)))
        throw std::runtime_error(std::string("Axis for ") + type_str + " joint must be a unit vector at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;
    case JointType::SPHERICAL:
    case JointType::PLANAR:
      // Both require 3 axes
      if (!areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
        throw std::runtime_error(std::string("Axes for ") + type_str + " joint must be mutually orthonormal at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;
    case JointType::FLOATING:
      // 6-DOF check both triplets
      if (!areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !areVectorsOrthonormal(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
        throw std::runtime_error("Axes for FLOATING joint must be two mutually orthonormal triplets at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;
    default:
      throw std::runtime_error("Unknown JointType for axes validation at line " +
                               std::to_string(axes_elem->GetLineNum()));
  }

  // Helper for vector fields
  auto fill_vector = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* child_tag) {
    int i = 0;
    for (const auto* elem = parent ? parent->FirstChildElement(child_tag) : nullptr; elem && i < v.size();
         elem = elem->NextSiblingElement(child_tag), ++i)
    {
      v(i) = parseRequiredDouble(elem, "value");
    }
  };

  fill_vector(joint.position, joint_elem->FirstChildElement("Positions"), "Position");
  fill_vector(joint.velocity, joint_elem->FirstChildElement("Velocities"), "Velocity");
  fill_vector(joint.effort, joint_elem->FirstChildElement("Efforts"), "Effort");

  // Limits (read in order)
  auto fill_limits = [](Eigen::VectorXd& min_v, Eigen::VectorXd& max_v, const tinyxml2::XMLElement* limits_elem,
                        const char* tag) {
    int i = 0;
    for (const auto* elem = limits_elem ? limits_elem->FirstChildElement(tag) : nullptr; elem && i < min_v.size();
         elem = elem->NextSiblingElement(tag), ++i)
    {
      min_v(i) = parseRequiredDouble(elem, "min");
      max_v(i) = parseRequiredDouble(elem, "max");
    }
  };
  const auto* limits_elem = joint_elem->FirstChildElement("Limits");
  if (limits_elem)
  {
    fill_limits(joint.limit.min_position, joint.limit.max_position, limits_elem, "Position");
    fill_limits(joint.limit.min_velocity, joint.limit.max_velocity, limits_elem, "Velocity");
    fill_limits(joint.limit.min_acceleration, joint.limit.max_acceleration, limits_elem, "Acceleration");
    fill_limits(joint.limit.min_jerk, joint.limit.max_jerk, limits_elem, "Jerk");
    fill_limits(joint.limit.min_effort, joint.limit.max_effort, limits_elem, "Effort");
  }

  // Dynamics: one element per DOF
  auto fill_dynvec = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag) {
    int i = 0;
    for (const auto* elem = parent ? parent->FirstChildElement(tag) : nullptr; elem && i < v.size();
         elem = elem->NextSiblingElement(tag), ++i)
    {
      v(i) = parseRequiredDouble(elem, "value");
    }
  };
  const auto* dyn_elem = joint_elem->FirstChildElement("Dynamics");
  if (dyn_elem)
  {
    fill_dynvec(joint.dynamics.stiffness, dyn_elem, "Stiffness");
    fill_dynvec(joint.dynamics.damping, dyn_elem, "Damping");
    fill_dynvec(joint.dynamics.inertia, dyn_elem, "Inertia");
    int i = 0;
    for (const auto* r_elem = dyn_elem->FirstChildElement("RestPosition");
         r_elem && i < joint.dynamics.rest_position.size(); r_elem = r_elem->NextSiblingElement("RestPosition"), ++i)
    {
      joint.dynamics.rest_position[i] = parseRequiredDouble(r_elem, "value");
    }
  }

  return joint;
}

Joint parseJoint(const tinyxml2::XMLElement* joint_elem)
{
  // Heuristic: multi-DOF if <Axes> exists, single-DOF if <Axis> exists
  if (joint_elem->FirstChildElement("Axes"))
    return parseMultiDofJoint(joint_elem);
  if (joint_elem->FirstChildElement("Axis"))
    return parseSingleDofJoint(joint_elem);
  throw std::runtime_error("Joint element is missing <Axis> or <Axes>");
}

void parseJointComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Joint element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  auto joint = parseJoint(elem);

  auto* joint_component = getOrCreateComponent<JointComponent>(db, eid);
  joint_component->elements.emplace_back(id, std::move(joint));
}

void parseFluidDomainShapeComponent(const tinyxml2::XMLElement* fluidElem, ginseng::database& db, EntityID eid)
{
  auto* stack_component = db.get_component<StackedShapeComponent*>(eid);
  if (!stack_component)
    throw std::runtime_error("StackedShapeComponent missing from entity!");

  // <FluidDomainShape id="...">
  const char* domain_id = fluidElem->Attribute("id");
  if (!domain_id)
    throw std::runtime_error("FluidDomainShape is missing 'id' attribute!");

  DomainShape domain_shape;

  const char* id = fluidElem->Attribute("id");
  if (!id)
    throw std::runtime_error("FluidDomainShape element missing 'id' attribute at line " +
                             std::to_string(fluidElem->GetLineNum()));

  // <StackedShape id="..."/>
  const tinyxml2::XMLElement* stackElem = fluidElem->FirstChildElement("StackedShape");
  if (!stackElem)
    throw std::runtime_error("FluidDomainShape missing <StackedShape>");

  const char* stack_id = stackElem->Attribute("id");
  if (!stack_id)
    throw std::runtime_error("StackedShape is missing 'id' attribute!");
  domain_shape.stacked_shape_id = stack_id ? stack_id : "";

  // Find the stack in the ECS
  auto stacked_shapes_ptr = getComponentElement(stack_component->elements, domain_shape.stacked_shape_id);
  if (!stacked_shapes_ptr)
    throw std::runtime_error("No StackedShape entry for id '" + domain_shape.stacked_shape_id +
                             "' in StackedShapeComponent");

  // For each shape in the stack
  for (const StackedShapeEntry& entry : *stacked_shapes_ptr)
  {
    // if (entry.shape.shape_map.empty())
    //   throw std::runtime_error("ShapeComponent for stack entry is empty!");

    // // Use the first shape (if there's only one per entry)
    // const auto& shape_pair = *entry.shape.shape_map.begin();
    const geometry::Shape& geom = entry.shape;

    using physics::DomainShapePtr;
    DomainShapePtr shape;

    switch (geom.type)
    {
      case ShapeType::Cylinder:
      {
        double radius = geom.dimensions.at(0);
        double height = geom.dimensions.at(1);
        shape = std::make_shared<physics::FluidCylinderShape>(radius, height);
        break;
      }
      case ShapeType::Box:
      {
        double width = geom.dimensions.at(0);
        double length = geom.dimensions.at(1);  // Or "length" if that's your convention
        double height = geom.dimensions.at(2);
        shape = std::make_shared<physics::FluidBoxShape>(width, length, height);
        break;
      }
      case ShapeType::SphericalSegment:
      {
        double base_radius = geom.dimensions.at(0);
        double top_radius = geom.dimensions.at(1);  // Or "length" if that's your convention
        double height = geom.dimensions.at(2);

        shape = std::make_shared<physics::FluidSphericalSegmentShape>(base_radius, top_radius, height);
        break;
      }
      case ShapeType::Cone:
      {
        double base_radius = geom.dimensions.at(0);
        double top_radius = geom.dimensions.at(1);
        double height = geom.dimensions.at(2);
        shape = std::make_shared<physics::FluidConeShape>(base_radius, top_radius, height);
        break;
      }
      // Add more shape types as needed.
      default:
        throw std::runtime_error("Unsupported shape type for fluid domain: " +
                                 std::to_string(static_cast<int>(geom.type)));
    }

    domain_shape.domains.push_back(shape);
  }

  auto* domain_shape_component = getOrCreateComponent<DomainShapeComponent>(db, eid);
  domain_shape_component->elements.emplace_back(id, std::move(domain_shape));
}

}  // namespace sodf
