#include <iostream>
#include <set>
#include <sodf/xml_parser.h>

#include <sodf/components/object.h>
#include <sodf/components/transform.h>
#include <sodf/components/link.h>
#include <sodf/components/joint.h>
#include <sodf/components/fitting.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/finite_state_machine.h>

using namespace tinyxml2;
using namespace sodf::components;

namespace sodf {

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

void XMLParser::parseComponents(ginseng::database& db)
{
  const auto* root = doc->FirstChildElement("root");
  if (!root)
    return;

  std::set<std::string> global_ids;

  for (const auto* object_elem = root->FirstChildElement("Object"); object_elem;
       object_elem = object_elem->NextSiblingElement("Object"))
  {
    auto eid = db.create_entity();

    // Check for global uniqueness accross all entities (Object id)
    const char* obj_id = object_elem->Attribute("id");
    if (!obj_id)
      throw std::runtime_error("Object element missing 'id' attribute at line " +
                               std::to_string(object_elem->GetLineNum()));

    if (!global_ids.insert(obj_id).second)
    {
      throw std::runtime_error("Duplicate Object id '" + std::string(obj_id) + "' at line " +
                               std::to_string(object_elem->GetLineNum()));
    }

    // Check for local uniqueness accross one entity
    std::set<std::string> local_ids;
    for (const tinyxml2::XMLElement* el = object_elem->FirstChildElement(); el; el = el->NextSiblingElement())
    {
      const char* local_id = el->Attribute("id");
      if (local_id)
      {
        if (!local_ids.insert(local_id).second)
        {
          throw std::runtime_error("Duplicate component id '" + std::string(local_id) + "' within entity at line " +
                                   std::to_string(el->GetLineNum()));
        }
      }
    }

    parseObjectElement(object_elem, db, eid);

    TransformComponent transform;
    parseOriginElement(object_elem, transform, db, eid);
    parseTransformElement(object_elem, std::move(transform), db, eid);

    parseLinkElement(object_elem, db, eid);
    parseJointElement(object_elem, db, eid);
    parseTouchscreenElement(object_elem, db, eid);
    parseFSMElement(object_elem, db, eid);
    parseActionMapElement(object_elem, db, eid);
  }
}

void parseObjectElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  ObjectComponent obj{};
  // obj.id = obj_elem->Attribute("id");
  const char* id = obj_elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Object element missing 'id' attribute at line " + std::to_string(obj_elem->GetLineNum()));

  obj.id = std::string(id);

  const auto* product = obj_elem->FirstChildElement("Product");
  if (product)
  {
    if (auto* name = product->FirstChildElement("Name"))
      obj.name = name->GetText();
    if (auto* model = product->FirstChildElement("Model"))
      obj.model = model->GetText();
    if (auto* sn = product->FirstChildElement("SerialNumber"))
      obj.serial_number = sn->GetText();
    if (auto* vendor = product->FirstChildElement("Vendor"))
      obj.vendor = vendor->GetText();
  }

  db.add_component(eid, std::move(obj));
}

void parseOriginElement(const tinyxml2::XMLElement* obj_elem, TransformComponent& transform, ginseng::database& db,
                        EntityID eid)
{
  // Find <Origin>
  const auto* origin_elem = obj_elem->FirstChildElement("Origin");
  if (!origin_elem)
    return;

  // Get id attribute or use "root"
  const char* id = origin_elem->Attribute("id");
  if (!id)
    throw std::runtime_error("Origin element missing 'id' attribute at line " +
                             std::to_string(origin_elem->GetLineNum()));
  std::string origin_id = id;

  // Case 1: <Transform> as child
  if (const auto* tf_elem = origin_elem->FirstChildElement("Transform"))
  {
    TransformFrame frame;
    parseTransform(tf_elem, frame);  // does not compute/resolve

    // special case, parent must starts with a '/' (absolute)
    if (!frame.parent.empty() && frame.parent.at(0) != '/')
      throw std::runtime_error("Transform <parent> attribute inside <Origin> must be empty or start with '/'. "
                               "Found: '" +
                               frame.parent + "' at line " + std::to_string(tf_elem->GetLineNum()));

    // Overwrite or insert as first entry
    if (transform.transform_map.empty())
      transform.transform_map.emplace_back(origin_id, std::move(frame));
    else
      transform.transform_map[0] = std::make_pair(origin_id, std::move(frame));
  }
  // Case 2: <AlignGeometricPair> (store for deferred system)
  else if (const auto* align_elem = origin_elem->FirstChildElement("AlignGeometricPair"))
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
  else if (const auto* align_elem = origin_elem->FirstChildElement("AlignFrames"))
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

void parseTransformElement(const tinyxml2::XMLElement* obj_elem, TransformComponent&& transform, ginseng::database& db,
                           EntityID eid)
{
  // Parse all nested tags that contain <Transform>
  for (const tinyxml2::XMLElement* el = obj_elem->FirstChildElement(); el; el = el->NextSiblingElement())
  {
    // Skip if this element is <Origin>
    if (std::strcmp(el->Name(), "Origin") == 0)
      continue;

    if (const auto* transform_elem = el->FirstChildElement("Transform"))
    {
      const char* id = el->Attribute("id");
      if (!id)
        throw std::runtime_error("Transform parent element missing 'id' attribute at line " +
                                 std::to_string(el->GetLineNum()));

      std::string id_str(id);

      // Check for duplicates manually
      auto it = std::find_if(transform.transform_map.begin(), transform.transform_map.end(),
                             [&](const auto& p) { return p.first == id_str; });

      if (it != transform.transform_map.end())
      {
        throw std::runtime_error("Duplicate transform id '" + id_str + "' at line " + std::to_string(el->GetLineNum()));
      }

      TransformFrame frame;
      parseTransform(transform_elem, frame);
      transform.transform_map.emplace_back(id_str, std::move(frame));
    }
  }
  db.add_component(eid, std::move(transform));
}

void parseLinkElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::LinkComponent link_component;

  for (const tinyxml2::XMLElement* link_elem = obj_elem->FirstChildElement("Link"); link_elem;
       link_elem = link_elem->NextSiblingElement("Link"))
  {
    components::Link link;

    const char* id = link_elem->Attribute("id");
    if (!id)
      throw std::runtime_error("Link element missing 'id' attribute at line " + std::to_string(link_elem->GetLineNum()));

    if (const auto* visual = link_elem->FirstChildElement("Visual"))
    {
      if (const auto* mesh = visual->FirstChildElement("Mesh"))
        link.visual.mesh_path = mesh->Attribute("path");
      if (const auto* scale = visual->FirstChildElement("Scale"))
        parsePosition(scale, link.visual.scale);
    }

    if (const auto* collision = link_elem->FirstChildElement("Collision"))
    {
      if (const auto* mesh = collision->FirstChildElement("Mesh"))
        link.collision.mesh_path = mesh->Attribute("path");
      if (const auto* scale = collision->FirstChildElement("Scale"))
        parsePosition(scale, link.collision.scale);
    }

    if (const auto* mass = link_elem->FirstChildElement("Mass"))
      link.dynamics.mass = mass->DoubleAttribute("value");

    if (const auto* com = link_elem->FirstChildElement("CenterOfMass"))
      parsePosition(com, link.dynamics.center_of_mass);

    if (const auto* I = link_elem->FirstChildElement("InertiaTensor"))
    {
      link.dynamics.inertia_tensor << I->DoubleAttribute("ixx"), I->DoubleAttribute("ixy"), I->DoubleAttribute("ixz"),
          I->DoubleAttribute("ixy"), I->DoubleAttribute("iyy"), I->DoubleAttribute("iyz"), I->DoubleAttribute("ixz"),
          I->DoubleAttribute("iyz"), I->DoubleAttribute("izz");
    }

    link_component.link_map.emplace_back(id, std::move(link));
  }
  db.add_component(eid, std::move(link_component));
}

void parseJointElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::JointComponent joint_component;

  for (const tinyxml2::XMLElement* joint_elem = obj_elem->FirstChildElement("Joint"); joint_elem;
       joint_elem = joint_elem->NextSiblingElement("Joint"))
  {
    components::Joint joint;

    const char* id = joint_elem->Attribute("id");
    if (!id)
      throw std::runtime_error("Joint element missing 'id' attribute at line " +
                               std::to_string(joint_elem->GetLineNum()));

    if (const auto* type = joint_elem->FirstChildElement("Type"))
    {
      std::string t = type->Attribute("value");
      if (t == "REVOLUTE")
        joint.type = components::JointType::REVOLUTE;
      else if (t == "PRISMATIC")
        joint.type = components::JointType::PRISMATIC;
      else
        joint.type = components::JointType::FIXED;
    }

    if (const auto* actuation = joint_elem->FirstChildElement("Actuation"))
    {
      std::string a = actuation->Attribute("value");
      if (a == "PASSIVE")
        joint.actuation = components::JointActuation::PASSIVE;
      else if (a == "ACTIVE")
        joint.actuation = components::JointActuation::ACTIVE;
      else if (a == "SPRING")
        joint.actuation = components::JointActuation::SPRING;
      else
        joint.actuation = components::JointActuation::FIXED;
    }

    if (const auto* position = joint_elem->FirstChildElement("Position"))
      joint.position = position->DoubleAttribute("value");

    if (const auto* axis = joint_elem->FirstChildElement("Axis"))
      parseUnitVector(axis, joint.axis);

    // if (const auto* named = joint_elem->FirstChildElement("NamedPositions"))
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

    joint_component.joint_map.emplace_back(id, std::move(joint));
  }

  db.add_component(eid, std::move(joint_component));
}

void parseFitConstraintElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::FitConstraintComponent fit_component;

  for (const tinyxml2::XMLElement* fit_elem = obj_elem->FirstChildElement("FitConstraint"); fit_elem;
       fit_elem = fit_elem->NextSiblingElement("FitConstraint"))
  {
    components::FitConstraint fit;

    const char* id = fit_elem->Attribute("id");
    if (!id)
      throw std::runtime_error("FitConstraint element missing 'id' attribute at line " +
                               std::to_string(fit_elem->GetLineNum()));

    if (const auto* axis = fit_elem->FirstChildElement("Axis"))
      parseUnitVector(axis, fit.axis);

    if (const auto* sym = fit_elem->FirstChildElement("RotationalSymmetry"))
      fit.rotational_symmetry = sym->UnsignedAttribute("value");

    if (const auto* dist = fit_elem->FirstChildElement("ApproachDistance"))
      fit.approach_distance = dist->DoubleAttribute("value");

    fit_component.fitting_map.emplace_back(id, std::move(fit));
  }

  db.add_component(eid, std::move(fit_component));
}

void parseTouchscreenElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::TouchscreenComponent ts_component;

  for (const tinyxml2::XMLElement* ts_elem = obj_elem->FirstChildElement("Touchscreen"); ts_elem;
       ts_elem = ts_elem->NextSiblingElement("Touchscreen"))
  {
    components::Touchscreen ts;

    const char* id = ts_elem->Attribute("id");
    if (!id)
      throw std::runtime_error("Touchscreen element missing 'id' attribute at line " +
                               std::to_string(ts_elem->GetLineNum()));

    if (const auto* size = ts_elem->FirstChildElement("Size"))
    {
      ts.width = size->DoubleAttribute("width");
      ts.height = size->DoubleAttribute("height");
    }

    if (const auto* normal = ts_elem->FirstChildElement("SurfaceNormal"))
      parseUnitVector(normal, ts.surface_normal);

    if (const auto* pressure = ts_elem->FirstChildElement("Pressure"))
    {
      ts.min_pressure = pressure->DoubleAttribute("min");
      ts.max_pressure = pressure->DoubleAttribute("max");
    }

    if (const auto* touch = ts_elem->FirstChildElement("Touch"))
    {
      ts.touch_radius = touch->DoubleAttribute("radius");
      ts.multi_touch = touch->BoolAttribute("multi_touch");
      ts.allow_drag = touch->BoolAttribute("allow_drag");
    }

    if (const auto* meta = ts_elem->FirstChildElement("Metadata"))
    {
      if (auto* model = meta->FirstChildElement("Model"))
        ts.model = model->GetText();
      if (auto* version = meta->FirstChildElement("Version"))
        ts.version = version->GetText();
      if (auto* driver = meta->FirstChildElement("Driver"))
        ts.driver = driver->GetText();
    }

    ts_component.touchscreen_map.emplace_back(id, std::move(ts));
  }

  db.add_component(eid, std::move(ts_component));
}

void parseFSMElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::FSMComponent fsm_component;

  for (const tinyxml2::XMLElement* fsm_elem = obj_elem->FirstChildElement("FSM"); fsm_elem;
       fsm_elem = fsm_elem->NextSiblingElement("FSM"))
  {
    const char* id = fsm_elem->Attribute("id");
    if (!id)
      throw std::runtime_error("FSM element missing 'id' attribute at line " + std::to_string(fsm_elem->GetLineNum()));

    components::FSM fsm;

    const auto* start = fsm_elem->FirstChildElement("StartState");
    if (!start || !start->Attribute("name"))
      throw std::runtime_error("FSM missing or malformed <StartState> at line " +
                               std::to_string(fsm_elem->GetLineNum()));

    std::string start_state = start->Attribute("name");

    // Parse states
    if (const auto* states_elem = fsm_elem->FirstChildElement("States"))
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
    if (const auto* actions_elem = fsm_elem->FirstChildElement("Actions"))
    {
      for (const auto* action_elem = actions_elem->FirstChildElement("Action"); action_elem;
           action_elem = action_elem->NextSiblingElement("Action"))
      {
        std::string name = action_elem->Attribute("name");
        fsm.action_labels.add(name);
      }
    }

    // Parse transitions
    if (const auto* transitions_elem = fsm_elem->FirstChildElement("Transitions"))
    {
      fsm.transitions = buildTransitionTableFromXML(transitions_elem, fsm.state_labels, fsm.action_labels);

      std::cout << "fsm transitions size = " << fsm.transitions.size() << std::endl;
      for (int s = 0; s < fsm.transitions.size(); ++s)
      {
        for (int a = 0; a < fsm.transitions[s].size(); ++a)
        {
          if (fsm.transitions[s][a] >= 0)
            std::cout << "Transition: " << fsm.state_labels.to_string(s) << " + " << fsm.action_labels.to_string(a)
                      << " -> " << fsm.state_labels.to_string(fsm.transitions[s][a]) << "\n";
        }
      }
    }
    fsm_component.fsm_map.emplace_back(id, std::move(fsm));
  }

  db.add_component(eid, std::move(fsm_component));
}

void parseActionMapElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid)
{
  components::ActionMapComponent action_map_component;

  for (const tinyxml2::XMLElement* component_elem = obj_elem->FirstChildElement(); component_elem;
       component_elem = component_elem->NextSiblingElement())
  {
    const char* component_id = component_elem->Attribute("id");
    if (!component_id)
      continue;

    for (const tinyxml2::XMLElement* action_map_elem = component_elem->FirstChildElement("ActionMap"); action_map_elem;
         action_map_elem = action_map_elem->NextSiblingElement("ActionMap"))
    {
      const char* fsm_id = action_map_elem->Attribute("fsm");
      if (!fsm_id)
        throw std::runtime_error("ActionMap missing 'fsm' attribute at line " +
                                 std::to_string(action_map_elem->GetLineNum()));

      components::ActionMap action_map;
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

      action_map_component.action_map.emplace_back(fsm_id, std::move(action_map));
    }
  }

  db.add_component(eid, std::move(action_map_component));
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

void parseTransform(const tinyxml2::XMLElement* transform_elem, components::TransformFrame& frame)
{
  if (const char* parent = transform_elem->Attribute("parent"))
    frame.parent = std::string(parent);

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

  frame.local = Eigen::Translation3d(pos) * q;
  frame.global = Eigen::Isometry3d::Identity();
  frame.dirty = true;
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

// // Clone a container by copying from an existing one
// void XMLParser::cloneContainer(Container& target, const std::string& sourceId,
//                                const ContainerCollection& containerCollection)
// {
//   for (const auto& containerPair : containerCollection.container_map)
//   {
//     if (containerPair.first == sourceId)
//     {
//       target = containerPair.second;
//       target.volume = containerPair.second.volume;  // Retain original volume
//       return;
//     }
//   }
//   std::cerr << "Error: Clone source '" << sourceId << "' not found." << std::endl;
// }

// // Parse a Shape element and return a BaseVolumePtr object
// BaseVolumePtr XMLParser::parseShape(XMLElement* shapeElement)
// {
//   std::string shapeName = shapeElement->Attribute("name");

//   if (shapeName == "SphericalCap")
//   {
//     double cap_radius, height;
//     shapeElement->QueryDoubleAttribute("cap_radius", &cap_radius);
//     shapeElement->QueryDoubleAttribute("height", &height);
//     return std::make_shared<SphericalCapVolume>(cap_radius, height);
//   }
//   else if (shapeName == "TruncatedCone")
//   {
//     double base_radius, top_radius, height;
//     shapeElement->QueryDoubleAttribute("base_radius", &base_radius);
//     shapeElement->QueryDoubleAttribute("top_radius", &top_radius);
//     shapeElement->QueryDoubleAttribute("height", &height);
//     return std::make_shared<TruncatedConeVolume>(base_radius, top_radius, height);
//   }
//   else if (shapeName == "Cylinder")
//   {
//     double radius, height;
//     shapeElement->QueryDoubleAttribute("radius", &radius);
//     shapeElement->QueryDoubleAttribute("height", &height);
//     return std::make_shared<CylinderVolume>(radius, height);
//   }

//   return nullptr;
// }

// // Handle container operations: remove, overwrite, or update
// void XMLParser::handleContainerOperation(const std::string& id, const std::string& operation,
//                                          XMLElement* containerElement, ContainerCollection& containerCollection)
// {
//   if (operation == "remove")
//   {
//     auto it = std::remove_if(containerCollection.container_map.begin(), containerCollection.container_map.end(),
//                              [&id](const std::pair<std::string, Container>& pair) { return pair.first == id; });
//     if (it != containerCollection.container_map.end())
//     {
//       containerCollection.container_map.erase(it, containerCollection.container_map.end());
//       std::cout << "Container '" << id << "' removed successfully.\n";
//     }
//     else
//     {
//       std::cerr << "Error: Container '" << id << "' not found for removal.\n";
//     }
//   }
//   else if (operation == "overwrite")
//   {
//     Container container;
//     containerElement->QueryDoubleAttribute("volume", &container.volume);

//     for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
//          shapeElement = shapeElement->NextSiblingElement("Shape"))
//     {
//       container.shape.push_back(parseShape(shapeElement));
//     }

//     containerCollection.container_map.push_back(std::make_pair(id, container));
//   }
//   else if (operation == "update")
//   {
//     for (auto& containerPair : containerCollection.container_map)
//     {
//       if (containerPair.first == id)
//       {
//         updateContainer(containerPair.second, containerElement);
//         return;
//       }
//     }
//     std::cerr << "Error: Cannot update. Container '" << id << "' not found.\n";
//   }
//   else
//   {
//     std::cerr << "Warning: Unknown operation '" << operation << "' for container '" << id << "'.\n";
//   }
// }

// // Update container attributes or add/overwrite shapes
// void XMLParser::updateContainer(Container& target, XMLElement* containerElement)
// {
//   containerElement->QueryDoubleAttribute("volume", &target.volume);

//   for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
//        shapeElement = shapeElement->NextSiblingElement("Shape"))
//   {
//     BaseVolumePtr newShape = parseShape(shapeElement);
//     bool shapeExists = false;

//     // Check if shape already exists, update if found
//     for (auto& shape : target.shape)
//     {
//       if (shape->getHeight(1.0) == newShape->getHeight(1.0))
//       {                    // Compare using height as identifier
//         shape = newShape;  // Overwrite existing shape
//         shapeExists = true;
//         break;
//       }
//     }

//     // Add new shape if not found
//     if (!shapeExists)
//     {
//       target.shape.push_back(newShape);
//     }
//   }
// }

// // Parse <Containers> section
// void XMLParser::parseContainers(ContainerCollection& containerCollection)
// {
//   XMLElement* root = doc.RootElement();
//   XMLElement* object = root->FirstChildElement("Object");
//   XMLElement* containersElement = object->FirstChildElement("Containers");

//   for (XMLElement* containerElement = containersElement->FirstChildElement("Container"); containerElement != nullptr;
//        containerElement = containerElement->NextSiblingElement("Container"))
//   {
//     std::string id = containerElement->Attribute("id");

//     // Check for operation attribute
//     const char* operationAttr = containerElement->Attribute("operation");
//     if (operationAttr)
//     {
//       std::string operation = operationAttr;
//       handleContainerOperation(id, operation, containerElement, containerCollection);
//       continue;  // Skip further parsing if operation is handled
//     }

//     // Handle cloned container
//     Container container;
//     const char* clone = containerElement->Attribute("clone");
//     if (clone)
//     {
//       cloneContainer(container, clone, containerCollection);
//     }
//     else
//     {
//       containerElement->QueryDoubleAttribute("volume", &container.volume);

//       // Parse all <Shape> elements inside the container
//       for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
//            shapeElement = shapeElement->NextSiblingElement("Shape"))
//       {
//         container.shape.push_back(parseShape(shapeElement));
//       }
//     }

//     containerCollection.container_map.push_back(std::make_pair(id, container));
//   }
// }

// // Parse <Transforms> section
// void XMLParser::parseTransforms(Transforms& transforms)
// {
//   XMLElement* root = doc.RootElement();
//   XMLElement* object = root->FirstChildElement("Object");
//   XMLElement* transformsElement = object->FirstChildElement("Transforms");

//   for (XMLElement* transformElement = transformsElement->FirstChildElement("Transform"); transformElement != nullptr;
//        transformElement = transformElement->NextSiblingElement("Transform"))
//   {
//     Transform transform;
//     transform.name = transformElement->Attribute("name");
//     transform.parent = transformElement->Attribute("parent");

//     // Parse Frame
//     XMLElement* frameElement = transformElement->FirstChildElement("Frame");
//     if (frameElement)
//     {
//       frameElement->QueryDoubleAttribute("x", &transform.transform.translation().x());
//       frameElement->QueryDoubleAttribute("y", &transform.transform.translation().y());
//       frameElement->QueryDoubleAttribute("z", &transform.transform.translation().z());
//       frameElement->QueryDoubleAttribute("qx", &transform.transform.rotation().x());
//       frameElement->QueryDoubleAttribute("qy", &transform.transform.rotation().y());
//       frameElement->QueryDoubleAttribute("qz", &transform.transform.rotation().z());
//       frameElement->QueryDoubleAttribute("qw", &transform.transform.rotation().w());
//     }
//     transforms.transforms.push_back(transform.transform);
//   }
// }

}  // namespace sodf
