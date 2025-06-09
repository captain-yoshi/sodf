#include <filesystem>
#include <iostream>
#include <set>
#include <sodf/xml_parser.h>

#include <sodf/components/domain_shape.h>

#include <sodf/components/button.h>
#include <sodf/components/grasp.h>
#include <sodf/components/shape.h>
#include <sodf/components/container.h>
#include <sodf/components/object.h>
#include <sodf/components/transform.h>
#include <sodf/components/link.h>
#include <sodf/components/joint.h>
#include <sodf/components/fitting.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/finite_state_machine.h>

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

// Helper: Parse ButtonType from string
components::ButtonType parseButtonType(const char* type_str)
{
  if (!type_str)
    throw std::runtime_error("Button missing 'type' attribute.");
  std::string type(type_str);
  if (type == "push")
    return ButtonType::PUSH;
  if (type == "rotary")
    return ButtonType::ROTARY;
  if (type == "analog")
    return ButtonType::ANALOG;
  if (type == "virtual")
    return ButtonType::VIRTUAL;
  throw std::runtime_error("Unknown button type: " + type);
}

/// Add only Object tag childrens, e.g. no Transform tag.
#define SODF_COMPONENT_LIST(X)                                                                                         \
  X(Origin, parseOriginComponent)                                                                                      \
  X(Link, parseLinkComponent)                                                                                          \
  X(Joint, parseJointComponent)                                                                                        \
  X(FitConstraint, parseFitConstraintComponent)                                                                        \
  X(Product, parseProductComponent)                                                                                    \
  X(Touchscreen, parseTouchscreenComponent)                                                                            \
  X(FSM, parseFSMComponent)                                                                                            \
  X(Button, parseButtonComponent)                                                                                      \
  X(FluidDomainShape, parseFluidDomainShapeComponent)                                                                  \
  X(Shape, parseShapeComponent)                                                                                        \
  X(ParallelGrasp, parseParallelGraspComponent)                                                                        \
  X(Container, parseContainerComponent)

enum class SceneComponentType
{
#define X(name, func) name,
  SODF_COMPONENT_LIST(X)
#undef X
      COUNT
};

SceneComponentType sceneComponentTypeFromString(const std::string& name)
{
#define X(tag, func)                                                                                                   \
  if (name == #tag)                                                                                                    \
    return SceneComponentType::tag;
  SODF_COMPONENT_LIST(X)
#undef X
  throw std::runtime_error("Unknown ComponentType string: " + name);
}

using ParseFunc = std::function<void(const tinyxml2::XMLElement*, ginseng::database&, sodf::EntityID)>;

static const std::vector<ParseFunc> parseFuncs = {
#define X(name, func) func,
  SODF_COMPONENT_LIST(X)
#undef X
};

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
          SceneComponentType comp_type = sceneComponentTypeFromString(comp_type_str);
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
      SceneComponentType comp_type = sceneComponentTypeFromString(comp_type_str);
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
    if (tf_component->transform_map.empty())
      tf_component->transform_map.emplace_back(origin_id, std::move(frame));
    else
      tf_component->transform_map[0] = std::make_pair(origin_id, std::move(frame));

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
    align.tolerance = align_pair_frames_elem->DoubleAttribute("tolerance", 1e-3);

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
          align.source1 = name;
        else if (found == 1)
          align.source2 = name;
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
      auto it = std::find_if(transform_component->transform_map.begin(), transform_component->transform_map.end(),
                             [&](const auto& p) { return p.first == id_str; });

      if (it != transform_component->transform_map.end())
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
      transform_component->transform_map.emplace_back(bottom_id, frame);

      auto* container_component = db.get_component<ContainerComponent*>(eid);
      if (!container_component)
        throw std::runtime_error("No ContainerComponent found for Container '" + id_str + "'");

      auto container = find_in_flat_map(container_component->container_map, id_str);
      if (!container)
        throw std::runtime_error("No Container entry found for '" + id_str + "' in ContainerComponent");

      // TODO retrieve from geometrical shape or domain shape
      auto* domain_component = db.get_component<DomainShapeComponent*>(eid);
      if (!domain_component)
        throw std::runtime_error("No DomainShapeComponent found for Container '" + id_str + "'");

      const auto domain_shapes = find_in_flat_map(domain_component->domain_shape_map, container->domain_shape_id);
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
      component->transform_map.emplace_back(id_str, std::move(frame));
    }
  }
}

void parseLinkComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  Link link;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Link element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  if (const auto* visual = elem->FirstChildElement("Visual"))
  {
    if (const auto* mesh = visual->FirstChildElement("Mesh"))
      link.visual.mesh_path = mesh->Attribute("path");
    if (const auto* scale = visual->FirstChildElement("Scale"))
      parsePosition(scale, link.visual.scale);
  }

  if (const auto* collision = elem->FirstChildElement("Collision"))
  {
    if (const auto* mesh = collision->FirstChildElement("Mesh"))
      link.collision.mesh_path = mesh->Attribute("path");
    if (const auto* scale = collision->FirstChildElement("Scale"))
      parsePosition(scale, link.collision.scale);
  }

  if (const auto* mass = elem->FirstChildElement("Mass"))
    link.dynamics.mass = mass->DoubleAttribute("value");

  if (const auto* com = elem->FirstChildElement("CenterOfMass"))
    parsePosition(com, link.dynamics.center_of_mass);

  if (const auto* I = elem->FirstChildElement("InertiaTensor"))
  {
    link.dynamics.inertia_tensor << I->DoubleAttribute("ixx"), I->DoubleAttribute("ixy"), I->DoubleAttribute("ixz"),
        I->DoubleAttribute("ixy"), I->DoubleAttribute("iyy"), I->DoubleAttribute("iyz"), I->DoubleAttribute("ixz"),
        I->DoubleAttribute("iyz"), I->DoubleAttribute("izz");
  }

  // store to component
  auto* component = getOrCreateComponent<LinkComponent>(db, eid);
  component->link_map.emplace_back(id, std::move(link));
}

void parseJointComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::Joint joint;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Joint element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  if (const auto* type = elem->FirstChildElement("Type"))
  {
    std::string t = type->Attribute("value");
    if (t == "REVOLUTE")
      joint.type = components::JointType::REVOLUTE;
    else if (t == "PRISMATIC")
      joint.type = components::JointType::PRISMATIC;
    else
      joint.type = components::JointType::FIXED;
  }

  if (const auto* actuation = elem->FirstChildElement("Actuation"))
  {
    std::string a = actuation->Attribute("value");
    if (a == "PASSIVE")
      joint.actuation = components::JointActuation::PASSIVE;
    else if (a == "ACTIVE")
      joint.actuation = components::JointActuation::ACTIVE;
    else if (a == "SPRING")
      joint.actuation = components::JointActuation::SPRING;
    else if (a == "VIRTUAL")
      joint.actuation = components::JointActuation::VIRTUAL;
    else
      joint.actuation = components::JointActuation::FIXED;
  }

  if (const auto* position = elem->FirstChildElement("Position"))
    joint.position = position->DoubleAttribute("value");

  if (const auto* axis = elem->FirstChildElement("Axis"))
    joint.axis = parseUnitVector(axis);

  // if (const auto* named = elem->FirstChildElement("NamedPositions"))
  // {
  //   for (const auto* pos = named->FirstChildElement("Position"); pos; pos = pos->NextSiblingElement("Position"))
  //   {
  //     const char* name = pos->Attribute("name");
  //     if (!name)
  //       throw std::runtime_error("Named position missing 'name' attribute at line " +
  //                                std::to_string(pos->GetLineNum()));
  //     double value = pos->DoubleAttribute("value");
  //     joint.named_positions.emplace(name, value);
  //   }
  // }

  // store to component
  auto* component = getOrCreateComponent<JointComponent>(db, eid);
  component->joint_map.emplace_back(id, std::move(joint));
}

void parseFitConstraintComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::FitConstraint fit;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("FitConstraint element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  if (const auto* axis = elem->FirstChildElement("Axis"))
    fit.axis = parseUnitVector(axis);

  if (const auto* sym = elem->FirstChildElement("RotationalSymmetry"))
    fit.rotational_symmetry = sym->UnsignedAttribute("value");

  if (const auto* dist = elem->FirstChildElement("ApproachDistance"))
    fit.approach_distance = dist->DoubleAttribute("value");

  // store to component
  auto* component = getOrCreateComponent<FitConstraintComponent>(db, eid);
  component->fitting_map.emplace_back(id, std::move(fit));
}

void parseTouchscreenComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::Touchscreen ts;

  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Touchscreen element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  if (const auto* size = elem->FirstChildElement("Size"))
  {
    ts.width = size->DoubleAttribute("width");
    ts.height = size->DoubleAttribute("height");
  }

  if (const auto* normal = elem->FirstChildElement("SurfaceNormal"))
    ts.surface_normal = parseUnitVector(normal);

  if (const auto* pressure = elem->FirstChildElement("Pressure"))
  {
    ts.min_pressure = pressure->DoubleAttribute("min");
    ts.max_pressure = pressure->DoubleAttribute("max");
  }

  if (const auto* touch = elem->FirstChildElement("Touch"))
  {
    ts.touch_radius = touch->DoubleAttribute("radius");
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
  component->touchscreen_map.emplace_back(id, std::move(ts));
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
  component->fsm_map.emplace_back(id, std::move(fsm));
}

void parseActionMapComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* component_id = elem->Attribute("id");
  if (!component_id)
    return;

  components::ActionMap action_map;

  for (const tinyxml2::XMLElement* action_map_elem = elem->FirstChildElement("ActionMap"); action_map_elem;
       action_map_elem = action_map_elem->NextSiblingElement("ActionMap"))
  {
    const char* fsm_id = action_map_elem->Attribute("fsm");
    if (!fsm_id)
      throw std::runtime_error("ActionMap missing 'fsm' attribute at line " +
                               std::to_string(action_map_elem->GetLineNum()));

    action_map.action_id = component_id;

    for (const tinyxml2::XMLElement* map_elem = action_map_elem->FirstChildElement("Map"); map_elem;
         map_elem = map_elem->NextSiblingElement("Map"))
    {
      const char* trigger = map_elem->Attribute("trigger");
      const char* action = map_elem->Attribute("action");
      if (!trigger || !action)
        throw std::runtime_error("Map missing 'trigger' or 'action' attribute at line " +
                                 std::to_string(map_elem->GetLineNum()));

      components::ActionMapEntry entry;
      entry.trigger = trigger;
      entry.action = action;
      action_map.mappings.push_back(std::move(entry));
    }

    // store to component
    auto* component = getOrCreateComponent<ActionMapComponent>(db, eid);
    component->action_map.emplace_back(fsm_id, std::move(action_map));
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
    throw std::runtime_error("Container missing AxisBottom.");

  // Material
  if (const auto* mat = elem->FirstChildElement("Material"))
    if (const char* type = mat->Attribute("type"))
      container.material_type = type;

  // Content (first one only)
  if (const auto* cont = elem->FirstChildElement("Content"))
  {
    if (const char* type = cont->Attribute("type"))
      container.content_type = type;
    double volume = cont->DoubleAttribute("volume", 0.0);
    const char* units = cont->Attribute("units");
    container.volume = convertVolumeToSI(volume, units ? units : "m^3");
  }

  bool has_shape_ref = false;
  // Parse <ShapeRef>
  if (const tinyxml2::XMLElement* shapeRefElem = elem->FirstChildElement("ShapeRef"))
  {
    if (const char* shape_id = shapeRefElem->Attribute("id"))
    {
      container.shape_ref = shape_id;
      has_shape_ref = true;
    }
    else
    {
      throw std::runtime_error("Missing id attribute in <ShapeRef>");
    }
  }

  // Parse <DomainShapeRef>
  if (const tinyxml2::XMLElement* domainShapeRefElem = elem->FirstChildElement("DomainShapeRef"))
  {
    if (const char* domain_id = domainShapeRefElem->Attribute("id"))
    {
      container.domain_shape_ref = domain_id;
      has_shape_ref = true;
    }
    else
    {
      throw std::runtime_error("Missing id attribute in <DomainShapeRef>");
    }
  }

  // Throw only if neither is present
  if (!has_shape_ref)
  {
    throw std::runtime_error("Container must have at least one <ShapeRef> or <DomainShapeRef> element.");
  }

  // store to component
  auto* component = getOrCreateComponent<ContainerComponent>(db, eid);
  component->container_map.emplace_back(id, std::move(container));
}

void parseButtonComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID parent_eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Button element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  ButtonEntry entry{ ButtonType::VIRTUAL, VirtualButton{} };
  entry.type = parseButtonType(elem->Attribute("type"));

  // Parse fields shared by PushButton/VirtualButton
  double activation_force = 0.0;
  if (const auto* act = elem->FirstChildElement("ActivationForce"))
    activation_force = act->DoubleAttribute("value", 0.0);

  Eigen::Vector3d direction_axis = Eigen::Vector3d(0, 0, 1);  // default Z
  if (const auto* dir = elem->FirstChildElement("DirectionAxis"))
    direction_axis = parseUnitVector(dir);

  double min_hold_duration = 0.0;
  if (const auto* hold = elem->FirstChildElement("MinHoldDuration"))
    min_hold_duration = hold->DoubleAttribute("value", 0.0);

  // Parse Button by type
  switch (entry.type)
  {
    case ButtonType::PUSH:
    {
      PushButton btn;
      btn.activation_force = activation_force;
      btn.direction_axis = direction_axis;
      btn.min_hold_duration = min_hold_duration;
      entry.data = btn;
      break;
    }
    case ButtonType::VIRTUAL:
    {
      VirtualButton btn;
      btn.activation_force = activation_force;
      btn.direction_axis = direction_axis;
      btn.min_hold_duration = min_hold_duration;
      entry.data = btn;
      break;
    }
    case ButtonType::ROTARY:
    {
      RotaryButton btn;

      // Diameter/Depth (from <Bounds>)
      if (const auto* bounds = elem->FirstChildElement("Bounds"))
      {
        btn.diameter = bounds->DoubleAttribute("width", 0.0);
        btn.depth = bounds->DoubleAttribute("height", 0.0);
      }

      // Rotation axis
      if (const auto* dir = elem->FirstChildElement("DirectionAxis"))
        btn.rotation_axis = parseUnitVector(dir);

      // Angles & detent (optionally)
      btn.min_angle = elem->FirstChildElement("MinAngle") ?
                          elem->FirstChildElement("MinAngle")->DoubleAttribute("value", 0.0) :
                          0.0;
      btn.max_angle = elem->FirstChildElement("MaxAngle") ?
                          elem->FirstChildElement("MaxAngle")->DoubleAttribute("value", 0.0) :
                          0.0;
      btn.detent_spacing = elem->FirstChildElement("DetentSpacing") ?
                               elem->FirstChildElement("DetentSpacing")->DoubleAttribute("value", 0.0) :
                               0.0;

      entry.data = btn;
      break;
    }
    case ButtonType::ANALOG:
    {
      AnalogButton btn;

      btn.direction_axis = direction_axis;
      btn.min_pressure = elem->FirstChildElement("MinPressure") ?
                             elem->FirstChildElement("MinPressure")->DoubleAttribute("value", 0.0) :
                             0.0;
      btn.max_pressure = elem->FirstChildElement("MaxPressure") ?
                             elem->FirstChildElement("MaxPressure")->DoubleAttribute("value", 0.0) :
                             0.0;

      entry.data = btn;
      break;
    }
    default:
      throw std::runtime_error("Unhandled button type in parseButtonElement.");
  }

  // Store in ButtonComponent
  auto* component = getOrCreateComponent<ButtonComponent>(db, parent_eid);
  component->button_map.emplace_back(id, std::move(entry));
}

DomainShapes parseFluidDomainShape(const tinyxml2::XMLElement* fluidDomainShapeElem)
{
  DomainShapes shapes;

  const tinyxml2::XMLElement* stackedShapesElem = fluidDomainShapeElem->FirstChildElement("StackedShapes");
  if (!stackedShapesElem)
  {
    throw std::runtime_error("Missing <StackedShapes> element.");
  }

  for (const tinyxml2::XMLElement* shapeElem = stackedShapesElem->FirstChildElement("Shape"); shapeElem != nullptr;
       shapeElem = shapeElem->NextSiblingElement("Shape"))
  {
    std::string type = shapeElem->Attribute("type");
    if (type == "SphericalSegment")
    {
      // Try both conventions: cap_radius or base_radius/top_radius
      double base_radius = 0.0, top_radius = 0.0, height = 0.0;
      // Handle legacy "cap_radius" as base_radius
      if (shapeElem->Attribute("cap_radius"))
      {
        base_radius = shapeElem->DoubleAttribute("cap_radius");
      }
      if (shapeElem->Attribute("base_radius"))
      {
        base_radius = shapeElem->DoubleAttribute("base_radius");
      }
      if (shapeElem->Attribute("top_radius"))
      {
        top_radius = shapeElem->DoubleAttribute("top_radius");
      }
      height = shapeElem->DoubleAttribute("height");

      // If top_radius is not given, it's a cap (cap_radius, tip at height)
      auto shape = std::make_shared<physics::FluidSphericalSegmentShape>(base_radius, top_radius, height);
      shapes.push_back(shape);
    }
    else if (type == "Cone")
    {
      double base_radius = shapeElem->DoubleAttribute("base_radius");
      double top_radius = shapeElem->DoubleAttribute("top_radius");
      double height = shapeElem->DoubleAttribute("height");
      auto shape = std::make_shared<physics::FluidConeShape>(base_radius, top_radius, height);
      shapes.push_back(shape);
    }
    else if (type == "Cylinder")
    {
      double radius = shapeElem->DoubleAttribute("radius");
      double height = shapeElem->DoubleAttribute("height");
      auto shape = std::make_shared<physics::FluidCylinderShape>(radius, height);
      shapes.push_back(shape);
    }
    else if (type == "Box")
    {
      double width = shapeElem->DoubleAttribute("width");
      double length = shapeElem->DoubleAttribute("length");
      double height = shapeElem->DoubleAttribute("height");
      auto shape = std::make_shared<physics::FluidBoxShape>(width, length, height);
      shapes.push_back(shape);
    }
    else
    {
      throw std::runtime_error("Unknown Shape type: " + type);
    }
  }
  return shapes;
}

void parseFluidDomainShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("FluidDomainShape element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  auto shapes = parseFluidDomainShape(elem);

  auto* component = getOrCreateComponent<DomainShapeComponent>(db, eid);
  component->domain_shape_map.emplace_back(id, std::move(shapes));
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
      shape.dimensions.push_back(dims->DoubleAttribute("width"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
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
      shape.dimensions.push_back(dims->DoubleAttribute("radius"));
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
        shape.vertices.emplace_back(v->DoubleAttribute("x"), v->DoubleAttribute("y"), 0.0);
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
      shape.dimensions.push_back(dims->DoubleAttribute("width"));
      shape.dimensions.push_back(dims->DoubleAttribute("depth"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
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
      shape.dimensions.push_back(dims->DoubleAttribute("radius"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
      break;
    }
    case ShapeType::Sphere:
    {
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Sphere missing <Dimensions>");
      shape.dimensions.push_back(dims->DoubleAttribute("radius"));
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
      shape.dimensions.push_back(dims->DoubleAttribute("base_radius"));
      shape.dimensions.push_back(dims->DoubleAttribute("top_radius"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
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
      shape.dimensions.push_back(dims->DoubleAttribute("base_radius"));
      shape.dimensions.push_back(dims->DoubleAttribute("top_radius"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
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
        shape.dimensions.push_back(dims->DoubleAttribute("width"));
        shape.dimensions.push_back(dims->DoubleAttribute("height"));
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
        shape.scale.x() = scale->DoubleAttribute("x", 1.0);
        shape.scale.y() = scale->DoubleAttribute("y", 1.0);
        shape.scale.z() = scale->DoubleAttribute("z", 1.0);
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

        double length = len_elem->DoubleAttribute("value");
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
          double x = v->DoubleAttribute("x");
          double y = v->DoubleAttribute("y");
          double z = 0.0;
          // If z attribute present, use it (else default to 0.0)
          v->QueryDoubleAttribute("z", &z);
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

  return shape;
}

CompositeShape parseCompositeShape(const tinyxml2::XMLElement* composite_elem)
{
  CompositeShape composite;
  Eigen::Vector3d stack_point = Eigen::Vector3d::Zero();  // current stacking position
  Eigen::Vector3d stack_axis = Eigen::Vector3d::UnitZ();  // default, overwritten per-shape
  // Needed for align="tip" logic
  bool first_shape = true;

  for (const tinyxml2::XMLElement* shape_elem = composite_elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    // Parse geometry with your existing function
    Shape shape = parseShape(shape_elem);

    // Stack/manual logic
    std::string stack_attr = shape_elem->Attribute("stack") ? shape_elem->Attribute("stack") : "";
    std::string align_attr = shape_elem->Attribute("align") ? shape_elem->Attribute("align") : "";
    const auto* trans_elem = shape_elem->FirstChildElement("Transform");

    Eigen::Isometry3d local_tf = Eigen::Isometry3d::Identity();

    if (stack_attr == "manual" || trans_elem)
    {
      // Manual: use transform as given
      if (trans_elem)
        local_tf = parseIsometry3D(trans_elem);
      else
        local_tf.setIdentity();  // No transform = origin
    }
    else
    {
      // Automatic stacking
      Eigen::Vector3d axis = getShapeSymmetryAxis(shape);

      // Alignment for first shape
      if (first_shape)
      {
        if (align_attr == "tip")
        {
          // Tip at origin; base at tip + axis*height
          local_tf.translation() = Eigen::Vector3d::Zero();
          // Axis orientation (align +Z to axis)
          local_tf.linear() = buildIsometry(Eigen::Vector3d::Zero(), axis).linear();
          // For next stacking, place next at base:
          stack_point = axis * shapeHeight(shape);
        }
        else
        {
          // Default: base at origin
          local_tf.translation() = Eigen::Vector3d::Zero();
          local_tf.linear() = buildIsometry(Eigen::Vector3d::Zero(), axis).linear();
          // For next stacking, tip is at base + axis*height
          stack_point = axis * shapeHeight(shape);
        }
        stack_axis = axis;
        first_shape = false;
      }
      else
      {
        // For subsequent shapes, align as requested
        if (align_attr == "base" || align_attr.empty())
        {
          // Place base at stack_point
          local_tf.translation() = stack_point;
          local_tf.linear() = buildIsometry(stack_point, axis).linear();
          // Next stacking point is base + axis*height
          stack_point = stack_point + axis * shapeHeight(shape);
        }
        else if (align_attr == "tip")
        {
          // Place tip at stack_point (base = tip - axis*height)
          local_tf.translation() = stack_point - axis * shapeHeight(shape);
          local_tf.linear() = buildIsometry(local_tf.translation(), axis).linear();
          // Next stacking point is at tip (stack_point)
          stack_point = stack_point;
        }
        else
        {
          throw std::runtime_error("Unknown align value: " + align_attr);
        }
        stack_axis = axis;
      }
    }

    // Add to composite
    composite.shapes.push_back({ shape, local_tf });
  }
  return composite;
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

components::TransformFrame parseTransform(const tinyxml2::XMLElement* transform_elem)
{
  TransformFrame frame;
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

  vec.x() = element->DoubleAttribute("x");
  vec.y() = element->DoubleAttribute("y");
  vec.z() = element->DoubleAttribute("z");

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

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* elem,
                                             const components::ShapeComponent& shape_component,
                                             const components::TransformComponent& transform_component,
                                             TransformFrame& transform)
{
  ParallelGrasp grasp;

  // Case 1: Completely Defined
  if (const auto* transform_elem = elem->FirstChildElement("Transform"))
  {
    // Read transform
    transform = parseTransform(transform_elem);

    // Approach
    grasp.approach = (std::string(elem->FirstChildElement("Approach")->Attribute("value")) == "INTERNAL") ?
                         ParallelGraspApproach::INTERNAL :
                         ParallelGraspApproach::EXTERNAL;

    // Gap Size
    grasp.gap_size = elem->FirstChildElement("GapSize")->DoubleAttribute("value");

    // Rotational Axis/Symmetry
    Eigen::Vector3d axis_of_rotation;
    axis_of_rotation = parseUnitVector(elem->FirstChildElement("RotationalAxis"));
    grasp.axis_of_rotation = axis_of_rotation;
    grasp.rotational_symmetry = elem->FirstChildElement("RotationalSymmetry")->UnsignedAttribute("value");

    // Real surfaces (by Shape ID)
    const auto* reals = elem->FirstChildElement("RealSurfaces");
    std::vector<std::string> surface_ids;
    for (const auto* surf = reals ? reals->FirstChildElement("Shape") : nullptr; surf;
         surf = surf->NextSiblingElement("Shape"))
    {
      surface_ids.push_back(surf->Attribute("id"));
    }

    if (surface_ids.size() == 2)
    {
      // Standard: two surfaces for a parallel grasp
      grasp.real_surfaces[0] = surface_ids[0];
      grasp.real_surfaces[1] = surface_ids[1];
    }
    else if (surface_ids.size() == 1)
    {
      // Permit a single surface if the shape is Cylinder or Box

      auto shape = find_in_flat_map(shape_component.shape_map, surface_ids[0]);
      if (!shape)
        throw std::runtime_error("No Shape entry found for '" + surface_ids[0] + "' in ShapeComponent");

      if (shape->type == ShapeType::Cylinder || shape->type == ShapeType::Box)
      {
        grasp.real_surfaces[0] = surface_ids[0];
        grasp.real_surfaces[1] = "";  // or std::nullopt if using optional
      }
      else
      {
        throw std::runtime_error("ParallelGrasp: Single real surface only allowed for Cylinder or Box shapes, got '" +
                                 shapeTypeToString(shape->type) + "' for surface '" + surface_ids[0] + "'.");
      }
    }
    else
    {
      throw std::runtime_error("ParallelGrasp: Must specify exactly 2 real surfaces, "
                               "or 1 if the shape is a Cylinder or Box.");
    }

    // Virtual surface
    grasp.virtual_surface = parseShape(elem->FirstChildElement("VirtualSurface"));
    return grasp;
  }

  // Case 2: Derived from shapes
  else if (const auto* derived_elem = elem->FirstChildElement("DerivedFromParallelShapes"))
  {
    // 4. Compute virtual surface orientation using supplied axes and first shape normal
    const auto* axis_rot_elem = derived_elem->FirstChildElement("AxisRotational");
    const auto* axis_norm_elem = derived_elem->FirstChildElement("AxisNormal");

    if (!axis_rot_elem || !axis_norm_elem)
    {
      std::ostringstream oss;
      oss << "ParallelGrasp <DerivedFromShapes> is missing required ";
      if (!axis_rot_elem)
        oss << "<AxisRotational> ";
      if (!axis_norm_elem)
        oss << "<AxisNormal> ";
      oss << "tag(s) at line " << derived_elem->GetLineNum() << ".";
      throw std::runtime_error(oss.str());
    }

    // Eigen::Vector3d axis_rotational, axis_normal;
    const Eigen::Vector3d axis_rotational = parseUnitVector(axis_rot_elem);
    const Eigen::Vector3d axis_normal = parseUnitVector(axis_norm_elem);

    // 1. Collect and validate referenced shapes (must be 2D for pairs)
    std::vector<const Shape*> shape_ptrs;
    std::vector<std::string_view> shape_ids;
    for (const auto* shape_elem = derived_elem->FirstChildElement("Shape"); shape_elem;
         shape_elem = shape_elem->NextSiblingElement("Shape"))
    {
      const auto& shape_id = shape_elem->Attribute("id");
      auto shape = find_in_flat_map(shape_component.shape_map, shape_id);
      if (!shape)
        throw std::runtime_error("No Shape entry found for '" + std::string(shape_id) + "' in ShapeComponent");
      shape_ptrs.push_back(shape);
      shape_ids.push_back(shape_id);
    }
    constexpr double CENTROID_TOLERANCE = 1e-6;

    // Check for even and >=2 shapes
    if (shape_ptrs.size() >= 2 && shape_ptrs.size() % 2 == 0)
    {
      size_t num_pairs = shape_ptrs.size() / 2;
      std::vector<Eigen::Vector3d> centroids;  // Pairwise centroids
      std::vector<Eigen::Vector3d> normals;    // Pairwise normals

      // For each pair, validate and compute
      for (size_t i = 0; i < shape_ptrs.size(); i += 2)
      {
        // 2D shape check
        for (int j = 0; j < 2; ++j)
        {
          if (!is2DShape(*shape_ptrs[i + j]))
            throw std::runtime_error("Shape '" + std::string(shape_ids[i + j]) + "' is not a 2D shape.");
        }

        // Get transforms
        auto tf0 = find_in_flat_map(transform_component.transform_map, shape_ids[i]);
        if (!tf0)
          throw std::runtime_error("No Transform entry found for '" + std::string(shape_ids[i]) +
                                   "' in TransformComponent");
        auto tf1 = find_in_flat_map(transform_component.transform_map, shape_ids[i + 1]);
        if (!tf1)
          throw std::runtime_error("No Transform entry found for '" + std::string(shape_ids[i + 1]) +
                                   "' in TransformComponent");

        // Compute centroids/normals in parent frame
        Eigen::Vector3d c0 = tf0->local * getShapeCentroid(*shape_ptrs[i]);
        Eigen::Vector3d c1 = tf1->local * getShapeCentroid(*shape_ptrs[i + 1]);
        Eigen::Vector3d n0 = tf0->local.linear() * getShapeNormalAxis(*shape_ptrs[i]);
        Eigen::Vector3d n1 = tf1->local.linear() * getShapeNormalAxis(*shape_ptrs[i + 1]);

        // Normals must be parallel
        if (n0.cross(n1).norm() > 1e-6)
          throw std::runtime_error("2D shapes are not parallel (normals differ) for pair " + std::to_string(i / 2));

        // Centroids must match in plane
        Eigen::Vector3d delta = c1 - c0;
        double projected = delta.dot(n0.normalized());
        Eigen::Vector3d delta_in_plane = delta - projected * n0.normalized();

        std::cout << delta_in_plane << std::endl;

        if (delta_in_plane.norm() > CENTROID_TOLERANCE)
          throw std::runtime_error("2D shape centroids do not align (mismatch in the plane) for pair " +
                                   std::to_string(i / 2));

        // Store centroid and normal for this pair
        centroids.push_back(0.5 * (c0 + c1));
        std::cout << "centroids = " << centroids.back().matrix() << std::endl;

        normals.push_back(n0.normalized());

        // For the first pair: save surface IDs and compute the grasp gap size
        if (i == 0)
        {
          grasp.real_surfaces[0] = shape_ids[i];
          grasp.real_surfaces[1] = shape_ids[i + 1];

          grasp.gap_size = std::abs((c1 - c0).dot(n0.normalized()));
        }
      }

      // validate centroids are at the same place
      const Eigen::Vector3d& ref_centroid = centroids[0];
      for (size_t i = 1; i < centroids.size(); ++i)
      {
        if (!centroids[i].isApprox(ref_centroid, CENTROID_TOLERANCE))
        {
          std::cerr << "Centroid " << i << ": " << centroids[i].transpose()
                    << " does not match reference: " << ref_centroid.transpose() << std::endl;
          throw std::runtime_error("ParallelGrasp validation failed: Centroid mismatch for pair " + std::to_string(i));
        }
      }

      // Use first pair as reference for transform
      transform.parent = shape_ids[0];

      // Virtual surface: average centroid, first normal, axis of rotation, etc.
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (const auto& c : centroids)
        centroid += c;
      centroid /= static_cast<double>(centroids.size());

      // For orientation, use supplied axes
      Eigen::Vector3d x_axis = axis_rotational.normalized();
      Eigen::Vector3d z_axis = axis_normal.normalized();
      Eigen::Vector3d y_axis = z_axis.cross(x_axis);

      Eigen::Matrix3d rot = computeOrientationFromAxes(x_axis, y_axis, z_axis);
      transform.local = Eigen::Isometry3d::Identity();
      transform.local.linear() = rot;
      transform.local.translation() = centroid;

      std::cout << transform.local.matrix() << std::endl;

      grasp.virtual_surface = *shape_ptrs[0];
      grasp.virtual_surface.axes = { rot.col(1), rot.col(0), rot.col(2) };

      std::cout << "normal: " << rot.col(2).transpose() << std::endl;
      std::cout << "rotational: " << rot.col(0).transpose() << std::endl;
      std::cout << "in-plane: " << rot.col(1).transpose() << std::endl;

      // Deduce approach (INTERNAL/EXTERNAL) from first pair
      Eigen::Vector3d to_virtual =
          centroid -
          (find_in_flat_map(transform_component.transform_map, shape_ids[0])->local * getShapeCentroid(*shape_ptrs[0]));
      grasp.approach =
          (to_virtual.dot(normals[0]) > 0) ? ParallelGraspApproach::INTERNAL : ParallelGraspApproach::EXTERNAL;

      Eigen::Vector3d axis_of_rotation_world = transform.local.linear() * axis_rotational.normalized();
      grasp.axis_of_rotation = axis_of_rotation_world;

      // grasp.axis_of_rotation = axis_rotational;
      grasp.rotational_symmetry = static_cast<uint32_t>(num_pairs * 2);
    }

    // For a single 3D shape (e.g., cylinder, box lateral)
    else if (shape_ptrs.size() == 1 &&
             (shape_ptrs[0]->type == ShapeType::Cylinder || shape_ptrs[0]->type == ShapeType::Box))
    {
      grasp.real_surfaces[0] = shape_ids[0];
      grasp.real_surfaces[1] = "";

      // Get first shape_transform
      auto shape_tf = find_in_flat_map(transform_component.transform_map, shape_ids[0]);
      if (!shape_tf)
        throw std::runtime_error("No Transform entry found for '" + std::string(shape_ids[0]) +
                                 "' in TransformComponent");

      // Parse axes from ParallelGrasp, *in cylinder (local) frame*
      Eigen::Vector3d axis_rotational_local = axis_rotational;
      Eigen::Vector3d axis_normal_local = axis_normal;
      axis_rotational_local.normalize();
      axis_normal_local.normalize();

      // Compute the in-plane (right-handed) axis
      Eigen::Vector3d axis_inplane_local = axis_normal_local.cross(axis_rotational_local);
      axis_inplane_local.normalize();

      // Transform: Cylinder-local to root/parent frame
      const Eigen::Matrix3d& shape3d_to_parent = shape_tf->local.linear();

      // Map the axes (they remain orthogonal/unit length if the transform is pure rotation)
      Eigen::Vector3d axis_rotational_root = shape3d_to_parent * axis_rotational_local;
      Eigen::Vector3d axis_normal_root = shape3d_to_parent * axis_normal_local;
      Eigen::Vector3d axis_inplane_root = shape3d_to_parent * axis_inplane_local;

      // Get centroid in Cylinder frame, map to parent/root frame
      Eigen::Vector3d centroid_3dshape = getShapeCentroid(*shape_ptrs[0]);  // usually center or surface point
      Eigen::Vector3d centroid_root = shape_tf->local * centroid_3dshape;

      transform.local.linear().col(0) = axis_rotational_root;  // X axis (rotation/symmetry)
      transform.local.linear().col(1) = axis_inplane_root;     // Y axis (in-plane tangent)
      transform.local.linear().col(2) = axis_normal_root;      // Z axis (normal)
      transform.local.translation() = centroid_root;

      // 4. (Optional) Debug print
      std::cout << shapeTypeToString(shape_ptrs[0]->type) << " grasp transform:\n"
                << transform.local.matrix() << std::endl;

      if (shape_ptrs[0]->type == ShapeType::Cylinder)
      {
        // Virtual surface is a line along axis_rotational, centered at centroid
        grasp.virtual_surface.type = ShapeType::Line;
        double length = shape_ptrs[0]->dimensions.at(1);  // height
                                                          // Eigen::Vector3d p0_local(-0.5 * length, 0.0, 0.0);
                                                          // Eigen::Vector3d p1_local(0.5 * length, 0.0, 0.0);

        grasp.virtual_surface.dimensions.push_back(length);

        Eigen::Vector3d p0_local = -0.5 * length * Eigen::Vector3d::UnitX();  // "UnitX" of the grasp frame
        Eigen::Vector3d p1_local = 0.5 * length * Eigen::Vector3d::UnitX();
        grasp.virtual_surface.vertices = { p0_local, p1_local };
        grasp.virtual_surface.axes = { Eigen::Vector3d::UnitX() };  // always local X of the grasp frame
        grasp.rotational_symmetry = 0;                              // Infinite symmetry
        grasp.gap_size = 2.0 * shape_ptrs[0]->dimensions.at(0);     // diameter
      }
      else if (shape_ptrs[0]->type == ShapeType::Box)
      {
        // Box dimensions in the *box's local frame* (not the grasp frame!)
        double box_dims[3] = { shape_ptrs[0]->dimensions.at(0), shape_ptrs[0]->dimensions.at(1),
                               shape_ptrs[0]->dimensions.at(2) };

        // Compute the dimensions in the local grasp frame
        // Project box's local axes onto the new grasp frame axes
        // The "width" and "height" of the rectangle are the *projections* of box dims
        double width = 0.0, height = 0.0;

        // The new grasp frame's X, Y, Z in box local coordinates
        // Eigen::Matrix3d grasp_in_box = (box_to_parent.transpose() * transform.local.linear());
        Eigen::Matrix3d grasp_in_box = (shape3d_to_parent.transpose() * transform.local.linear());

        // Compute the lengths along the in-plane (Y) and rotational (X) axes
        for (int i = 0; i < 3; ++i)
        {
          // Box axis i in box frame: unit vector e_i
          Eigen::Vector3d axis_box = Eigen::Vector3d::Unit(i);
          // Project onto grasp frame axes:
          width += std::abs(axis_inplane_local.dot(axis_box)) * box_dims[i];
          height += std::abs(axis_rotational_local.dot(axis_box)) * box_dims[i];
        }

        grasp.virtual_surface.type = ShapeType::Rectangle;
        grasp.virtual_surface.dimensions = { width, height };
        grasp.virtual_surface.axes = { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                       Eigen::Vector3d::UnitZ() };  // always grasp frame axes

        // Vertices (optional): corners in grasp frame, centered at origin
        Eigen::Vector3d corner0(-0.5 * width, -0.5 * height, 0);
        Eigen::Vector3d corner1(0.5 * width, -0.5 * height, 0);
        Eigen::Vector3d corner2(0.5 * width, 0.5 * height, 0);
        Eigen::Vector3d corner3(-0.5 * width, 0.5 * height, 0);
        grasp.virtual_surface.vertices = { corner0, corner1, corner2, corner3 };

        grasp.gap_size = width;  // or set according to your symmetry convention
        grasp.rotational_symmetry = 4;
      }

      grasp.axis_of_rotation = Eigen::Vector3d::UnitX();  // X axis in grasp frame

      // Require and parse Approach tag
      const auto* approach_elem = derived_elem->FirstChildElement("Approach");
      if (!approach_elem || !approach_elem->Attribute("value"))
        throw std::runtime_error(
            "ParallelGrasp: <Approach> tag with 'value' attribute is required for single 3D shape (box/cylinder).");

      std::string approach_str = approach_elem->Attribute("value");
      if (approach_str == "Internal")
        grasp.approach = ParallelGraspApproach::INTERNAL;
      else if (approach_str == "External")
        grasp.approach = ParallelGraspApproach::EXTERNAL;
      else
        throw std::runtime_error("ParallelGrasp: <Approach> value must be 'Internal' or 'External', got: " +
                                 approach_str);
    }
    else
    {
      throw std::runtime_error(
          "ParallelGrasp: Must derive from exactly two 2D shapes (parallel), or one 3D lateral shape (cylinder/box).");
    }

    std::cout << grasp;

    return grasp;
  }
  throw std::runtime_error("Could not parse ParallelGrasp: missing required elements.");
}

// void parseParallelGraspComponent(const tinyxml2::XMLElement* elem, const components::ShapeComponent& shape_component,
//                                  const components::TransformComponent& transform_component, TransformFrame& transform)
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

  TransformFrame tf;
  auto grasp = parseParallelGrasp(elem, *shape_component, *transform_component, tf);

  transform_component->transform_map.emplace_back(id, std::move(tf));

  auto* grasp_component = getOrCreateComponent<ParallelGraspComponent>(db, eid);
  grasp_component->grasp_map.emplace_back(id, std::move(grasp));
}

void parseShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const char* id = elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Shape element missing 'id' attribute at line " + std::to_string(elem->GetLineNum()));

  auto shape = parseShape(elem);

  auto* shape_component = getOrCreateComponent<ShapeComponent>(db, eid);
  shape_component->shape_map.emplace_back(id, std::move(shape));
}

}  // namespace sodf
