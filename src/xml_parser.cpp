#include <iostream>
#include <set>
#include <sodf/xml_parser.h>

#include <sodf/components/domain_shape.h>

#include <sodf/components/button.h>
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

namespace sodf {

const DomainShapes* getDomainShapesByRef(const DomainShapeComponent& comp, const std::string& domain_shape_ref)
{
  return find_in_flat_map(comp.domain_shape_map, domain_shape_ref);
}

// Helper for allowed types
inline bool isSupportedFluidShape(ShapeType type)
{
  return type == ShapeType::Box || type == ShapeType::Cylinder || type == ShapeType::Sphere ||
         type == ShapeType::SphericalSegment;
}

// --- Helper: extract height of shape for stacking ---
double shapeHeight(const Shape& shape)
{
  // Add more shape types as needed
  switch (shape.type)
  {
    case ShapeType::SphericalSegment:
    case ShapeType::Cylinder:
    case ShapeType::Cone:
    case ShapeType::Box:
      // Assumes height is always last dimension (see your convention)
      return shape.dimensions.back();
    // For others, add as appropriate
    default:
      throw std::runtime_error("Automatic stacking: unknown height for this shape type");
  }
}

// --- Helper: extract symmetry axis from axes vector (index 0 per convention) ---
Eigen::Vector3d shapeAxis(const Shape& shape)
{
  if (shape.axes.empty())
    throw std::runtime_error("Shape missing axes info for stacking");
  return shape.axes[0].normalized();
}

// --- Helper: build isometry from position/axis ---
Eigen::Isometry3d buildIsometry(const Eigen::Vector3d& pos, const Eigen::Vector3d& axis)
{
  // Align +Z to axis (for most shapes)
  Eigen::Vector3d z = axis.normalized();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  if (std::abs(z.dot(x)) > 0.99)
    x = Eigen::Vector3d::UnitY();  // Avoid colinear
  Eigen::Vector3d y = z.cross(x).normalized();
  x = y.cross(z).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = rot;
  iso.translation() = pos;
  return iso;
}

ShapeType shapeTypeFromString(const std::string& str)
{
  if (str == "Rectangle")
    return ShapeType::Rectangle;
  if (str == "Circle")
    return ShapeType::Circle;
  if (str == "Triangle")
    return ShapeType::Triangle;
  if (str == "Polygon")
    return ShapeType::Polygon;
  if (str == "Box")
    return ShapeType::Box;
  if (str == "Cylinder")
    return ShapeType::Cylinder;
  if (str == "Sphere")
    return ShapeType::Sphere;
  if (str == "Cone")
    return ShapeType::Cone;
  if (str == "SphericalSegment")
    return ShapeType::SphericalSegment;
  if (str == "Mesh")
    return ShapeType::Mesh;
  if (str == "Plane")
    return ShapeType::Plane;
  if (str == "Line")
    return ShapeType::Line;
  throw std::runtime_error("Unknown ShapeType: " + str);
}

// Helper function for unit conversion:
inline double convertVolumeToSI(double volume, const std::string& units)
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
inline double parseRequiredDouble(const tinyxml2::XMLElement* elem, const char* attr_name)
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
  X(Container, parseContainerComponent)

enum class SceneComponentType
{
#define X(name, func) name,
  SODF_COMPONENT_LIST(X)
#undef X
      COUNT
};

inline SceneComponentType sceneComponentTypeFromString(const std::string& name)
{
#define X(tag, func)                                                                                                   \
  if (name == #tag)                                                                                                    \
    return SceneComponentType::tag;
  SODF_COMPONENT_LIST(X)
#undef X
  throw std::runtime_error("Unknown SceneComponentType string: " + name);
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

inline std::string canonical_ns(const std::string& parent_ns, const std::string& local_ns)
{
  if (parent_ns.empty())
    return local_ns;
  if (local_ns.empty())
    return parent_ns;
  return parent_ns + "::" + local_ns;
}

inline std::string canonicalize_ref(const std::string& current_ns, const std::string& ref)
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

inline std::string extract_namespace(const std::string& qualified_id)
{
  auto pos = qualified_id.rfind("::");
  if (pos == std::string::npos)
    return "";
  return qualified_id.substr(0, pos);
}

void buildObjectIndex(const tinyxml2::XMLDocument* doc, ObjectIndex& object_index, const std::string& parent_ns,
                      const std::string& base_dir, const std::string& filename);

void processInclude(const XMLElement* include_elem, ObjectIndex& object_index, const std::string& base_dir,
                    const std::string& parent_ns)
{
  std::string path = include_elem->Attribute("path");
  std::string ns = include_elem->Attribute("ns") ? include_elem->Attribute("ns") : "";
  // std::string full_ns = parent_ns.empty() ? ns : (parent_ns.empty() ? ns : parent_ns + "::" + ns);
  std::string full_ns = canonical_ns(parent_ns, ns);
  std::string inc_filename = base_dir + "/" + path;

  auto* inc_doc = new tinyxml2::XMLDocument();
  if (inc_doc->LoadFile((base_dir + "/" + path).c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("Failed to include: " + path);

  buildObjectIndex(inc_doc, object_index, full_ns, base_dir, inc_filename);  // pass the full_ns down!
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

// Load XML file
bool XMLParser::loadXML(const std::string& filename)
{
  doc = std::make_unique<tinyxml2::XMLDocument>();

  XMLError result = doc->LoadFile(filename.c_str());
  if (result != XML_SUCCESS)
  {
    std::cerr << "Error loading XML file: " << filename << std::endl;
    return false;
  }
  return true;
}

bool XMLParser::loadEntities(const std::string& rel_filepath, const std::string& abs_basepath, ginseng::database& db)
{
  if (!loadXML(abs_basepath + "/" + rel_filepath))
    return false;

  ObjectIndex object_index;
  buildObjectIndex(doc.get(), object_index, "", abs_basepath, abs_basepath + "/" + rel_filepath);

  SceneMap scene_map;
  parseSceneObjects(doc.get(), object_index, scene_map);

  for (const auto& scene : scene_map)
  {
    auto eid = db.create_entity();
    // add id to object
    ObjectComponent obj_comp{ .id = scene.second.id };
    db.add_component(eid, std::move(obj_comp));

    for (const auto& comp : scene.second.components)
    {
      // SceneComponentType type = typeFromString(comp.type);
      size_t index = static_cast<size_t>(comp.type);

      if (index >= parseFuncs.size())
        throw std::runtime_error("No parse function for component id: " + comp.id);

      parseFuncs[index](comp.xml, db, eid);

      // can reside in every component types
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

  // Case 1: <Transform> as child
  if (const auto* tf_elem = elem->FirstChildElement("Transform"))
  {
    TransformFrame frame = parseTransform(tf_elem);  // does not compute/resolve

    // special case, parent must starts with a '/' (absolute)
    if (!frame.parent.empty() && frame.parent.at(0) != '/')
      throw std::runtime_error("Transform <parent> attribute inside <Origin> must be empty or start with '/'. "
                               "Found: '" +
                               frame.parent + "' at line " + std::to_string(tf_elem->GetLineNum()));

    // store to component
    auto* component = getOrCreateComponent<TransformComponent>(db, eid);

    // Overwrite or insert as first entry
    if (component->transform_map.empty())
      component->transform_map.emplace_back(origin_id, std::move(frame));
    else
      component->transform_map[0] = std::make_pair(origin_id, std::move(frame));
  }
  // Case 2: <AlignGeometricPair> (store for deferred system)
  else if (const auto* align_elem = elem->FirstChildElement("AlignGeometricPair"))
  {
    AlignGeometricPairComponent align;
    align.target_id = align_elem->Attribute("with");
    align.tolerance = align_elem->DoubleAttribute("tolerance", 1e-3);

    int found = 0;
    for (const auto* tag = align_elem->FirstChildElement(); tag; tag = tag->NextSiblingElement())
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
          align.target1 = name;
        else if (found == 3)
          align.target2 = name;
        else
          throw std::runtime_error("Too many <Target> tags in <AlignGeometricPair> at line " +
                                   std::to_string(tag->GetLineNum()));
      }
      else
        throw std::runtime_error("Unexpected tag <" + tname + "> in <AlignGeometricPair> at line " +
                                 std::to_string(tag->GetLineNum()));
      found++;
    }
    if (align.source1.empty() || align.source2.empty() || align.target1.empty() || align.target2.empty())
      throw std::runtime_error("Missing source/target transforms in <AlignGeometricPair> at line " +
                               std::to_string(align_elem->GetLineNum()));

    // Attach to database for later use (deferred alignment system)
    db.add_component(eid, std::move(align));
  }
  // Case 3: <AlignFrames> (store for deferred system)
  else if (const auto* align_elem = elem->FirstChildElement("AlignFrames"))
  {
    components::AlignFramesComponent align;
    align.target_id = align_elem->Attribute("with");
    const auto* src = align_elem->FirstChildElement("Source");
    const auto* tgt = align_elem->FirstChildElement("Target");
    align.source = src && src->Attribute("name") ? src->Attribute("name") : "";
    align.target = tgt && tgt->Attribute("name") ? tgt->Attribute("name") : "";

    // Attach to database for later use (deferred alignment system)
    db.add_component(eid, std::move(align));
  }

  // Note: Do NOT add TransformComponent to the db here (let parseTransformElement do it)
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

    TransformFrame frame = parseTransform(transform_elem);

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
      const auto* domain_shapes = getDomainShapesByRef(*domain_component, container->domain_shape_ref);
      if (!domain_shapes)
        throw std::runtime_error("DomainShapeComponent is empty");

      // Add surface transform
      {
        std::string surface_id = id_str + "/surface";
        double current_height = physics::getFillHeight(*domain_shapes, container->volume, 10e-04);

        // Add virtual joint
        auto joint = components::Joint{};
        joint.type = components::JointType::PRISMATIC;
        joint.actuation = components::JointActuation::VIRTUAL;
        joint.position = -current_height;
        joint.axis = container->axis_bottom;

        auto* joint_component = getOrCreateComponent<components::JointComponent>(db, eid);
        joint_component->joint_map.emplace_back("joint/" + bottom_id, std::move(joint));

        // Add virtual joint transform
        TransformFrame joint_frame;
        joint_frame.is_static = false;  // virtual prismatic joint to simulate liquid height
        joint_frame.parent = bottom_id;

        transform_component->transform_map.emplace_back("joint/" + bottom_id, std::move(joint_frame));

        // Add surface transform
        TransformFrame surface_frame;
        surface_frame.is_static = true;  // virtual prismatic joint to simulate liquid height
        surface_frame.parent = "joint/" + bottom_id;

        transform_component->transform_map.emplace_back(surface_id, std::move(surface_frame));
      }

      // Add top transform
      {
        double max_height = physics::getMaxFillHeight(*domain_shapes);

        // The axis points towards the bottom
        Eigen::Vector3d top_offset = -container->axis_bottom.normalized() * max_height;
        TransformFrame top_frame = frame;

        top_frame.local.translation().x() += top_offset.x();
        top_frame.local.translation().y() += top_offset.y();
        top_frame.local.translation().z() += top_offset.z();

        std::string top_id = id_str + "/top";
        transform_component->transform_map.emplace_back(top_id, std::move(top_frame));
      }
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
    parseUnitVector(axis, joint.axis);

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
    parseUnitVector(axis, fit.axis);

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
    parseUnitVector(normal, ts.surface_normal);

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
    parseUnitVector(a, container.axis_bottom);
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
    parseUnitVector(dir, direction_axis);

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
        parseUnitVector(dir, btn.rotation_axis);

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

Shape parseShape(const tinyxml2::XMLElement* elem)
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
        parseUnitVector(a, normal);
      else
        throw std::runtime_error("Rectangle missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisWidth"))
        parseUnitVector(a, width);
      else
        throw std::runtime_error("Rectangle missing AxisWidth.");
      if (const auto* a = elem->FirstChildElement("AxisHeight"))
        parseUnitVector(a, height);
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
        parseUnitVector(a, normal);
      else
        throw std::runtime_error("Circle missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisMajor"))
        parseUnitVector(a, major);
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
        parseUnitVector(a, normal);
      else
        throw std::runtime_error("Triangle/Polygon missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisX"))
        parseUnitVector(a, ax);
      else
        throw std::runtime_error("Triangle/Polygon missing AxisX.");
      if (const auto* a = elem->FirstChildElement("AxisY"))
        parseUnitVector(a, ay);
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
      if (const auto* a = elem->FirstChildElement("AxisX"))
        parseUnitVector(a, ax);
      else
        throw std::runtime_error("Box missing AxisX.");
      if (const auto* a = elem->FirstChildElement("AxisY"))
        parseUnitVector(a, ay);
      else
        throw std::runtime_error("Box missing AxisY.");
      if (const auto* a = elem->FirstChildElement("AxisZ"))
        parseUnitVector(a, az);
      else
        throw std::runtime_error("Box missing AxisZ.");
      shape.axes = { ax, ay, az };
      const auto* dims = elem->FirstChildElement("Dimensions");
      if (!dims)
        throw std::runtime_error("Box missing <Dimensions>");
      shape.dimensions.push_back(dims->DoubleAttribute("width"));
      shape.dimensions.push_back(dims->DoubleAttribute("height"));
      shape.dimensions.push_back(dims->DoubleAttribute("depth"));
      break;
    }
    case ShapeType::Cylinder:
    {
      Eigen::Vector3d symmetry, ref;
      if (const auto* a = elem->FirstChildElement("AxisSymmetry"))
        parseUnitVector(a, symmetry);
      else
        throw std::runtime_error("Cylinder missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("ReferenceAxis"))
        parseUnitVector(a, ref);
      else
        throw std::runtime_error("Cylinder missing ReferenceAxis.");
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
        parseUnitVector(a, symmetry);
      else
        throw std::runtime_error("Cone missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("ReferenceAxis"))
        parseUnitVector(a, ref);
      else
        throw std::runtime_error("Cone missing ReferenceAxis.");
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
        parseUnitVector(a, symmetry);
      else
        throw std::runtime_error("SphericalSegment missing AxisSymmetry.");
      if (const auto* a = elem->FirstChildElement("ReferenceAxis"))
        parseUnitVector(a, ref);
      else
        throw std::runtime_error("SphericalSegment missing ReferenceAxis.");
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
        parseUnitVector(a, normal);
      else
        throw std::runtime_error("Plane missing AxisNormal.");
      if (const auto* a = elem->FirstChildElement("AxisX"))
        parseUnitVector(a, ax);
      else
        throw std::runtime_error("Plane missing AxisX.");
      if (const auto* a = elem->FirstChildElement("AxisY"))
        parseUnitVector(a, ay);
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
        parseUnitVector(axis_elem, axis);
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
      Eigen::Vector3d axis = shapeAxis(shape);

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
  pos.x() = element->DoubleAttribute("x");
  pos.y() = element->DoubleAttribute("y");
  pos.z() = element->DoubleAttribute("z");
}

void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  double roll = element->DoubleAttribute("roll");
  double pitch = element->DoubleAttribute("pitch");
  double yaw = element->DoubleAttribute("yaw");
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  q.x() = element->DoubleAttribute("x");
  q.y() = element->DoubleAttribute("y");
  q.z() = element->DoubleAttribute("z");
  q.w() = element->DoubleAttribute("w");
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

void parseUnitVector(const tinyxml2::XMLElement* element, Eigen::Vector3d& vec, double epsilon)
{
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

// geometry::BaseShapePtr parseContainerShapeElement(const tinyxml2::XMLElement* elem)
// {
//   if (!elem)
//     throw std::runtime_error("Null element passed to parseContainerShapeElement.");

//   const char* shape_type = elem->Attribute("name");
//   if (!shape_type)
//     throw std::runtime_error(std::string("Shape element missing 'name' attribute at line ") +
//                              std::to_string(elem->GetLineNum()));

//   std::string type(shape_type);

//   if (type == "Cylinder")
//   {
//     double radius = parseRequiredDouble(elem, "radius");
//     double height = parseRequiredDouble(elem, "height");
//     return std::make_shared<geometry::CylinderShape>(radius, height);
//   }
//   else if (type == "Box")
//   {
//     double width = parseRequiredDouble(elem, "width");
//     double length = parseRequiredDouble(elem, "length");
//     double height = parseRequiredDouble(elem, "height");
//     return std::make_shared<geometry::BoxShape>(width, length, height);
//   }
//   else if (type == "Cone")
//   {
//     double base_radius = parseRequiredDouble(elem, "base_radius");
//     double top_radius = parseRequiredDouble(elem, "top_radius");
//     double height = parseRequiredDouble(elem, "height");
//     return std::make_shared<geometry::ConeShape>(base_radius, top_radius, height);
//   }
//   else if (type == "SphericalCap")
//   {
//     double cap_radius = parseRequiredDouble(elem, "cap_radius");
//     double height = parseRequiredDouble(elem, "height");
//     return std::make_shared<geometry::SphericalCapShape>(cap_radius, height);
//   }
//   else
//   {
//     throw std::runtime_error("Unknown shape type '" + type + "' in <Shape> at line " +
//                              std::to_string(elem->GetLineNum()));
//   }
// }

}  // namespace sodf
