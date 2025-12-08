#include "sodf/xml/utils.h"
#include "sodf/xml/element_parser.h"
#include "sodf/xml/component_parser.h"
#include "sodf/xml/expression_parser.h"

#include <sodf/component_type.h>
#include <sodf/components/action_map.h>
#include <sodf/components/button.h>
#include <sodf/components/container.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/finite_state_machine.h>
#include <sodf/components/insertion.h>
#include <sodf/components/grasp.h>
#include <sodf/components/joint.h>
#include <sodf/components/link.h>
#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/shape.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/transform.h>

#include <sodf/physics/domain_shape.h>
#include <sodf/geometry/frame.h>

namespace sodf {
namespace xml {

using namespace components;

std::string evalElementIdRequired(const tinyxml2::XMLElement* elem)
{
  std::string id;
  if (!tryEvalTextAttribute(elem, "id", &id))
    throw std::runtime_error(std::string(elem->Name()) + " element missing 'id' attribute at line " +
                             std::to_string(elem->GetLineNum()));

  if (id.empty())
    throw std::runtime_error(std::string(elem->Name()) + " 'id' attribute cannot be empty at line " +
                             std::to_string(elem->GetLineNum()));
  return id;
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
    std::string from = evalTextAttributeRequired(tr_elem, "from");
    std::string action = evalTextAttributeRequired(tr_elem, "action");
    std::string to = evalTextAttributeRequired(tr_elem, "to");

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

std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* trans_elem,
                                                          const FSMLabels& state_labels, const FSMLabels& action_labels)
{
  return buildTransitionTableFromXML(trans_elem, state_labels, action_labels, -1);
}

void parseStackedShapeComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem,
                                database::Database& db, database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  auto stack = parseStackedShape(doc, elem);
  if (stack.shapes.empty())
    throw std::runtime_error("StackedShape '" + std::string(id) + "' is empty at line " +
                             std::to_string(elem->GetLineNum()));

  auto* stacked_shape_component = db.get_or_add<StackedShapeComponent>(eid);
  stacked_shape_component->elements.emplace_back(id, std::move(stack));
}

void parseParallelGraspComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem,
                                 database::Database& db, database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  if (const auto* derived_elem = elem->FirstChildElement("DerivedFromParallelShapes"))
  {
    parseDerivedFromParallelShapes(doc, derived_elem, db, eid);
    // automatically added to the database
  }
  else
  {
    auto grasp = parseParallelGrasp(doc, elem);

    auto* grasp_component = db.get_or_add<ParallelGraspComponent>(eid);
    grasp_component->elements.emplace_back(id, std::move(grasp));
  }
}

void parseShapeComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                         database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  auto shape = parseShape(doc, elem);

  auto* shape_component = db.get_or_add<ShapeComponent>(eid);
  shape_component->elements.emplace_back(id, std::move(shape));
}

void parseDomainShapeComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem,
                               database::Database& db, database::EntityID eid)
{
  using namespace components;
  (void)doc;
  if (!elem)
    throw std::runtime_error("parseDomainShapeComponent: null <DomainShape> element");

  // ----- id (required) -----
  const std::string id = evalElementIdRequired(elem);

  // ----- type (optional, default: Fluid) -----
  auto parseDomainType = [](const char* t) -> physics::DomainType {
    if (!t || !*t)
      return physics::DomainType::Fluid;
    std::string s(t);
    for (auto& c : s)
      c = static_cast<char>(std::tolower(c));
    if (s == "fluid")
      return physics::DomainType::Fluid;
    throw std::runtime_error("Unsupported DomainShape type='" + std::string(t) + "'");
  };
  const physics::DomainType dtype = parseDomainType(elem->Attribute("type"));

  // ----- <StackedShapeRef id="..."/> (optional but validated if present) -----
  std::string stacked_shape_id;
  const auto* ssr = elem->FirstChildElement("StackedShapeRef");
  if (!ssr)
    throw std::runtime_error("StackedShapeRef element required in <DomainShape> element");

  stacked_shape_id = evalTextAttributeRequired(ssr, "id");
  if (stacked_shape_id.empty())
    throw std::runtime_error("Empty 'id' in <StackedShapeRef> for DomainShape '" + id + "' at line " +
                             std::to_string(ssr->GetLineNum()));

  auto* stacked_shape = db.get_element<StackedShapeComponent>(eid, stacked_shape_id);
  if (!stacked_shape)
    throw std::runtime_error("No StackedShape entry found for '" + stacked_shape_id +
                             "' in StackedShapeComponent (referenced by DomainShape '" + id + "')");

  // ----- Build component payload -----
  physics::DomainShape ds(*stacked_shape, dtype);
  ds.type = dtype;
  ds.stacked_shape_id = stacked_shape_id;

  // ----- Store to ECS -----
  auto* comp = db.get_or_add<components::DomainShapeComponent>(eid);
  if (!comp)
    throw std::runtime_error("get_or_add<DomainShapeComponent> failed for DomainShape '" + id + "'");
  comp->elements.emplace_back(id, std::move(ds));
}

void parseLinkComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                        database::EntityID eid)
{
  Link link;
  const std::string id = evalElementIdRequired(elem);

  // BoundingBox (ptional)
  const tinyxml2::XMLElement* bbox = elem->FirstChildElement("BoundingBox");
  if (bbox)
    if (const auto* shape = bbox->FirstChildElement("Shape"))
      link.bbox = parseShape(doc, shape);

  // Collision (required)
  const auto* collision = elem->FirstChildElement("Collision");
  if (!collision)
    throw std::runtime_error("Link '" + std::string(id) + "' missing required <Collision> element at line " +
                             std::to_string(elem->GetLineNum()));

  const auto* collision_shape = collision->FirstChildElement("Shape");
  if (!collision_shape)
    throw std::runtime_error("Link '" + std::string(id) + "' <Collision> is missing <Shape> element at line " +
                             std::to_string(collision->GetLineNum()));

  link.collision = parseShape(doc, collision_shape);

  // Visual (optional)
  const auto* visual = elem->FirstChildElement("Visual");
  if (visual)
  {
    const auto* visual_shape = visual->FirstChildElement("Shape");
    if (!visual_shape)
      throw std::runtime_error("Link '" + std::string(id) + "' <Visual> is missing <Shape> element at line " +
                               std::to_string(visual->GetLineNum()));

    link.visual = parseShape(doc, visual_shape);
  }

  // Inertial (optional)
  const tinyxml2::XMLElement* inertial = elem->FirstChildElement("Inertial");
  if (inertial)
  {
    if (const auto* mass = inertial->FirstChildElement("Mass"))
      link.dynamics.mass = evalNumberAttributeRequired(mass, "value");

    if (const auto* com = inertial->FirstChildElement("CenterOfMass"))
      parsePosition(com, link.dynamics.center_of_mass);

    if (const auto* I = inertial->FirstChildElement("Tensor"))
    {
      link.dynamics.inertia_tensor << evalNumberAttributeRequired(I, "ixx"), evalNumberAttributeRequired(I, "ixy"),
          evalNumberAttributeRequired(I, "ixz"), evalNumberAttributeRequired(I, "ixy"),
          evalNumberAttributeRequired(I, "iyy"), evalNumberAttributeRequired(I, "iyz"),
          evalNumberAttributeRequired(I, "ixz"), evalNumberAttributeRequired(I, "iyz"),
          evalNumberAttributeRequired(I, "izz");
    }
  }

  auto* tf_component = db.get_or_add<TransformComponent>(eid);

  auto add_child_tf = [&](const tinyxml2::XMLElement* parent_elem, const char* suffix) {
    const std::string child_id = id + "/" + suffix;

    geometry::TransformNode node;
    node.parent = id;  // always relative to the link
    node.local = Eigen::Isometry3d::Identity();
    node.global = Eigen::Isometry3d::Identity();
    node.dirty = true;

    if (parent_elem)
    {
      if (const auto* tf = parent_elem->FirstChildElement("Transform"))
      {
        node = parseTransformNode(tf);

        if (node.parent.empty())
          node.parent = id;  // always relative to the link
      }
    }

    tf_component->elements.emplace_back(child_id, std::move(node));
  };

  if (bbox)
    add_child_tf(bbox, "bbox");
  if (collision)
    add_child_tf(collision, "collision");
  if (visual)
    add_child_tf(visual, "visual");
  if (inertial)
    add_child_tf(inertial, "inertial");

  // Store to component
  auto* component = db.get_or_add<LinkComponent>(eid);
  auto [it, inserted] = component->elements.emplace_back(id, std::move(link));
}

void parseInsertionComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                             database::EntityID eid)
{
  auto parseRole = [&](const tinyxml2::XMLElement* e, const std::string& insertion_id) -> InsertionRole {
    const char* role_cstr = e->Attribute("role");
    if (!role_cstr)
      return InsertionRole::Receptacle;  // default if absent
    const std::string v(role_cstr);
    if (v == "Receptacle")
      return InsertionRole::Receptacle;
    if (v == "Insert")
      return InsertionRole::Insert;
    throw std::runtime_error("Insertion '" + insertion_id + "' has invalid role='" + v + "' at line " +
                             std::to_string(e->GetLineNum()) + " (allowed: Receptacle or Insert).");
  };

  components::Insertion insertion;
  const std::string id = evalElementIdRequired(elem);

  insertion.role = parseRole(elem, id);

  // AxisInsertion (required)
  const auto* axis1 = elem->FirstChildElement("AxisInsertion");
  if (!axis1)
  {
    throw std::runtime_error("Insertion '" + id + "' missing <AxisInsertion> element at line " +
                             std::to_string(elem->GetLineNum()));
  }
  insertion.axis_insertion = parseUnitVector(axis1);

  // AxisReference (required)
  const auto* axis2 = elem->FirstChildElement("AxisReference");
  if (!axis2)
  {
    throw std::runtime_error("Insertion '" + id + "' missing <AxisReference> element at line " +
                             std::to_string(elem->GetLineNum()));
  }
  insertion.axis_reference = parseUnitVector(axis2);

  // Validate orthonormality (axis_insertion ⟂ axis_reference, both unit)
  if (!geometry::areVectorsOrthonormal(insertion.axis_insertion, insertion.axis_reference, 1e-09))
  {
    throw std::runtime_error("Insertion '" + id + "' axes are not orthonormal at line " +
                             std::to_string(elem->GetLineNum()));
  }

  // RotationalSymmetry (required)
  //
  // <RotationalSymmetry value="2"/>
  //
  const auto* sym = elem->FirstChildElement("RotationalSymmetry");
  if (!sym)
  {
    throw std::runtime_error("Insertion '" + id + "' missing <RotationalSymmetry> element at line " +
                             std::to_string(elem->GetLineNum()));
  }
  insertion.rotational_symmetry = evalUIntAttributeRequired(sym, "value");

  // MaxDepth (required)
  const auto* max_depth = elem->FirstChildElement("MaxDepth");
  if (!max_depth)
  {
    throw std::runtime_error("Insertion '" + id + "' missing <MaxDepth> element at line " +
                             std::to_string(elem->GetLineNum()));
  }
  insertion.max_depth = evalNumberAttributeRequired(max_depth, "value");

  if (const auto* shape_ref = elem->FirstChildElement("StackedShapeRef"))
  {
    insertion.stacked_shape_id = evalTextAttributeRequired(shape_ref, "id");

    // Optional payload-local frame (preferred base for rendering the domain)
    if (const auto* tf_elem = shape_ref->FirstChildElement("Transform"))
    {
      const std::string frame_id = evalTextAttributeRequired(tf_elem, "id");
      const std::string parent = evalTextAttributeRequired(tf_elem, "parent");

      geometry::TransformNode node;
      node.parent = parent;
      node.local = parseIsometry3D(tf_elem);
      node.is_static = true;

      // Register this frame in the entity's TransformComponent
      auto* tfc = db.get_or_add<TransformComponent>(eid);
      if (!tfc)
        throw std::runtime_error("get_or_add<TransformComponent> failed for Insertion '" + id + "'");
      tfc->elements.emplace_back(frame_id, std::move(node));

      insertion.stacked_shape_frame_id = frame_id;
    }
  }

  // Store to component
  auto* component = db.get_or_add<InsertionComponent>(eid);
  component->elements.emplace_back(id, std::move(insertion));
}

void parseTouchscreenComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem,
                               database::Database& db, database::EntityID eid)
{
  components::Touchscreen ts;
  const std::string id = evalElementIdRequired(elem);

  if (const auto* size = elem->FirstChildElement("Size"))
  {
    ts.width = evalNumberAttributeRequired(size, "width");
    ts.height = evalNumberAttributeRequired(size, "height");
  }

  if (const auto* normal = elem->FirstChildElement("SurfaceNormal"))
    ts.surface_normal = parseUnitVector(normal);

  if (const auto* pressure = elem->FirstChildElement("Pressure"))
  {
    ts.min_pressure = evalNumberAttributeRequired(pressure, "min");
    ts.max_pressure = evalNumberAttributeRequired(pressure, "max");
  }

  if (const auto* touch = elem->FirstChildElement("Touch"))
  {
    ts.touch_radius = evalNumberAttributeRequired(touch, "radius");
    ts.multi_touch = evalBoolAttribute(touch, "multi_touch", false);
    ts.allow_drag = evalBoolAttribute(touch, "allow_drag", false);
  }

  if (const auto* meta = elem->FirstChildElement("Metadata"))
  {
    if (auto* model = meta->FirstChildElement("Model"))
      ts.model = evalTextNode(model);
    if (auto* version = meta->FirstChildElement("Version"))
      ts.version = evalTextNode(version);
    if (auto* driver = meta->FirstChildElement("Driver"))
      ts.driver = evalTextNode(driver);
  }

  // store to component
  auto* component = db.get_or_add<TouchscreenComponent>(eid);
  component->elements.emplace_back(id, std::move(ts));
}  // namespace xml

void parseFSMComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                       database::EntityID eid)
{
  components::FSM fsm;
  const std::string id = evalElementIdRequired(elem);

  const auto* start = elem->FirstChildElement("StartState");
  if (!start || !start->Attribute("name"))
    throw std::runtime_error("FSM missing or malformed <StartState> at line " + std::to_string(elem->GetLineNum()));

  std::string start_state = evalTextAttributeRequired(start, "name");

  // Parse states
  if (const auto* states_elem = elem->FirstChildElement("States"))
  {
    for (const auto* state_elem = states_elem->FirstChildElement("State"); state_elem;
         state_elem = state_elem->NextSiblingElement("State"))
    {
      std::string name = evalTextAttributeRequired(state_elem, "name");
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
      std::string name = evalTextAttributeRequired(action_elem, "name");
      fsm.action_labels.add(name);
    }
  }

  // Parse transitions
  if (const auto* transitions_elem = elem->FirstChildElement("Transitions"))
  {
    fsm.transitions = buildTransitionTableFromXML(transitions_elem, fsm.state_labels, fsm.action_labels);

    // std::cout << "fsm transitions size = " << fsm.transitions.size() << std::endl;
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
  auto* component = db.get_or_add<FSMComponent>(eid);
  component->elements.emplace_back(id, std::move(fsm));
}

void parseActionMapComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                             database::EntityID eid)
{
  if (!elem->FirstChildElement("ActionMap"))
    return;

  const std::string component_id = evalElementIdRequired(elem);
  for (const tinyxml2::XMLElement* action_map_elem = elem->FirstChildElement("ActionMap"); action_map_elem;
       action_map_elem = action_map_elem->NextSiblingElement("ActionMap"))
  {
    const std::string fsm_id = evalTextAttributeRequired(action_map_elem, "fsm");

    // Build up ActionMap locally
    components::ActionMap action_map;

    for (const tinyxml2::XMLElement* map_elem = action_map_elem->FirstChildElement("Map"); map_elem;
         map_elem = map_elem->NextSiblingElement("Map"))
    {
      components::ActionMapEntry entry;
      // Required
      entry.trigger = evalTextAttributeRequired(map_elem, "trigger");
      entry.action = evalTextAttributeRequired(map_elem, "action");

      // Optional: trigger_params
      std::string params;
      if (tryEvalTextAttribute(map_elem, "trigger_params", &params))
        entry.trigger_params = std::move(params);

      entry.component_id = component_id;
      entry.component_type = componentTypeFromString(elem->Name());

      action_map.mappings.push_back(std::move(entry));
    }

    // Only after the entire ActionMap is successfully parsed, mutate ECS
    auto* component = db.get_or_add<components::ActionMapComponent>(eid);

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

void parseContainerComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                             database::EntityID eid)
{
  (void)doc;
  if (!elem)
    throw std::runtime_error("parseContainerComponent: null <Container> element");

  using namespace sodf;
  using namespace sodf::components;

  Container container;

  const std::string id = evalElementIdRequired(elem);
  if (id.empty())
    throw std::runtime_error("Container missing id at line " + std::to_string(elem->GetLineNum()));

  // Optional direct attributes
  container.stacked_shape_id = evalTextAttribute(elem, "stacked_shape", "");
  container.material_id = evalTextAttribute(elem, "material", "");

  auto* tfc = db.get_or_add<TransformComponent>(eid);
  if (!tfc)
    throw std::runtime_error("get_or_add<TransformComponent> failed for Container '" + id + "'");

  // <Content>
  if (const auto* cont = elem->FirstChildElement("Content"))
  {
    container.content_type = evalTextAttribute(cont, "type", "");
    const double vol_raw = evalNumberAttribute(cont, "volume", 0.0);
    const std::string units = evalTextAttribute(cont, "units", "m^3");
    container.payload.volume = convertVolumeToSI(vol_raw, units.c_str());
  }

  // <StackedShapeID> (optional usage-site reference)
  if (const auto* ssid = elem->FirstChildElement("StackedShapeID"))
  {
    container.stacked_shape_id = evalTextAttributeRequired(ssid, "id");
  }

  // <MaterialID> (optional usage-site reference)
  if (const auto* mid = elem->FirstChildElement("MaterialID"))
  {
    container.material_id = evalTextAttributeRequired(mid, "id");
  }

  // <PayloadDomain>
  const auto* pd = elem->FirstChildElement("PayloadDomain");
  if (pd)
  {
    // Optional override of volume at payload level
    if (pd->Attribute("volume"))
    {
      const double vol_raw = evalNumberAttribute(pd, "volume", container.payload.volume);
      const std::string units = evalTextAttribute(pd, "units", "m^3");
      container.payload.volume = convertVolumeToSI(vol_raw, units.c_str());
    }

    // REQUIRED for fluid logic: <DomainShapeID id="..."/>
    if (const auto* dsid = pd->FirstChildElement("DomainShapeID"))
    {
      container.payload.domain_shape_id = evalTextAttributeRequired(dsid, "id");
    }

    // Optional live liquid-level frame id (authored frame name)
    if (const auto* lvl = pd->FirstChildElement("LiquidLevelFrame"))
    {
      container.fluid.liquid_level_frame_id = evalTextAttributeRequired(lvl, "id");
    }
  }

  // Without a domain id we can't build a fluid joint.
  // Still allow the container to exist as a pure logical/static container.
  if (container.payload.domain_shape_id.empty())
  {
    auto* cc = db.get_or_add<ContainerComponent>(eid);
    if (!cc)
      throw std::runtime_error("get_or_add<ContainerComponent> failed for Container '" + id + "'");
    cc->elements.emplace_back(id, std::move(container));
    return;
  }

  // --- Use the DomainShape instance to build a PRISMATIC(VIRTUAL) joint ---

  auto* dsc = db.get_element<DomainShapeComponent>(eid, container.payload.domain_shape_id);
  if (!dsc)
  {
    throw std::runtime_error("DomainShape '" + container.payload.domain_shape_id + "' not found for Container '" + id +
                             "'");
  }

  // If this domain shape references a stacked shape, compute total height from it.
  double q_max = 0.0;

  if (!dsc->stacked_shape_id.empty())
  {
    auto* stack = db.get_element<StackedShapeComponent>(eid, dsc->stacked_shape_id);
    if (!stack)
    {
      throw std::runtime_error("StackedShape '" + dsc->stacked_shape_id + "' not found for DomainShape '" +
                               container.payload.domain_shape_id + "'");
    }

    double H = 0.0;
    for (const auto& e : stack->shapes)
    {
      H += sodf::geometry::getShapeHeight(e.shape);
    }
    q_max = H;
  }

  Joint joint;
  joint.type = JointType::PRISMATIC;
  joint.actuation = JointActuation::VIRTUAL;
  joint.initialize_for_type();  // 1 DOF

  // Canonical convention: Domain height axis is +X in the domain local frame.
  // We store the prismatic axis in joint local frame.
  joint.axes.col(0) = Eigen::Vector3d(-1.0, 0.0, 0.0);

  const double q_min = 0.0;
  joint.position[0] = q_min;
  joint.limit.min_position[0] = q_min;
  joint.limit.max_position[0] = q_max;

  // Build a stable joint id.
  std::string joint_id;
  if (!container.fluid.liquid_level_frame_id.empty())
    joint_id = "joint/" + container.fluid.liquid_level_frame_id;
  else
    joint_id = "joint/" + id + "/liquid_level";

  container.fluid.liquid_level_joint_id = joint_id;

  auto* jc = db.get_or_add<JointComponent>(eid);
  if (!jc)
    throw std::runtime_error("get_or_add<JointComponent> failed for Container '" + id + "'");
  jc->elements.emplace_back(joint_id, std::move(joint));

  // Joint frame under the DOMAIN frame (not under the container)
  {
    geometry::TransformNode jtf;
    jtf.parent = id;
    jtf.local = Eigen::Isometry3d::Identity();
    jtf.is_static = false;
    tfc->elements.emplace_back(joint_id, std::move(jtf));
  }

  // Liquid level frame under joint (if authored)
  if (!container.fluid.liquid_level_frame_id.empty())
  {
    geometry::TransformNode ltf;
    ltf.parent = joint_id;
    ltf.local = Eigen::Isometry3d::Identity();
    ltf.is_static = true;
    tfc->elements.emplace_back(container.fluid.liquid_level_frame_id, std::move(ltf));
  }

  // Persist container
  auto* cc = db.get_or_add<ContainerComponent>(eid);
  if (!cc)
    throw std::runtime_error("get_or_add<ContainerComponent> failed for Container '" + id + "'");
  cc->elements.emplace_back(id, std::move(container));
}

void parseButtonComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                          database::EntityID eid)
{
  auto btn = parseButton(elem);

  // validate link + joint exists in database
  {
    auto* component = db.get<components::LinkComponent>(eid);
    if (!component)
      throw std::runtime_error("LinkComponent does not exists, required by ButtonComponent");

    auto* link = database::get_element(component->elements, btn.link_id);
    if (!link)
      throw std::runtime_error("No Link entry found for '" + btn.link_id + "' in LinkComponent");
  }
  {
    auto* component = db.get<components::JointComponent>(eid);
    if (!component)
      throw std::runtime_error("JointComponent does not exists, required by ButtonComponent");

    auto* joint = database::get_element(component->elements, btn.joint_id);
    if (!joint)
      throw std::runtime_error("No Joint entry found for '" + btn.joint_id + "' in JointComponent");
  }

  auto* btn_component = db.get_or_add<ButtonComponent>(eid);
  const std::string id = evalElementIdRequired(elem);
  btn_component->elements.emplace_back(id, std::move(btn));
}

void parseVirtualButtonComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem,
                                 database::Database& db, database::EntityID eid)
{
  auto button = parseVirtualButton(elem);

  auto* btn_component = db.get_or_add<VirtualButtonComponent>(eid);
  const std::string id = evalElementIdRequired(elem);
  btn_component->elements.emplace_back(id, std::move(button));
}

void parseJointComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                         database::EntityID eid)
{
  auto joint = parseJoint(elem);

  auto* joint_component = db.get_or_add<JointComponent>(eid);
  const std::string id = evalElementIdRequired(elem);
  joint_component->elements.emplace_back(id, std::move(joint));
}

void parseProductComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                           database::EntityID eid)
{
  // Note: the object id has already been populated prior this stage
  auto* component = db.get_or_add<ObjectComponent>(eid);

  if (auto* name = elem->FirstChildElement("Name"))
    component->name = evalTextNode(name);
  if (auto* model = elem->FirstChildElement("Model"))
    component->model = evalTextNode(model);
  if (auto* sn = elem->FirstChildElement("SerialNumber"))
    component->serial_number = evalTextNode(sn);
  if (auto* vendor = elem->FirstChildElement("Vendor"))
    component->vendor = evalTextNode(vendor);
}

void parseOriginComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                          database::EntityID eid)
{
  using namespace components;

  const std::string id = evalElementIdRequired(elem);

  // Build/append OriginComponent
  auto* origin_component = db.get_or_add<OriginComponent>(eid);
  if (!origin_component)
    throw std::runtime_error("get_or_add<OriginComponent> failed at line " + std::to_string(elem->GetLineNum()));

  // Optional scoping from attributes (child <Scope> can override below)
  origin_component->host_object = evalTextAttribute(elem, "host_object", origin_component->host_object);
  origin_component->guest_object = evalTextAttribute(elem, "guest_object", origin_component->guest_object);

  // Helpers -------------------------------------------------------------------

  auto getEither = [&](const tinyxml2::XMLElement* node, const char* a, const char* b,
                       const char* fallback_name) -> std::string {
    // Try 'a' first, then 'b'. If neither present, throw with helpful name.
    if (const char* v = node->Attribute(a))
      return std::string(v);
    if (const char* v = node->Attribute(b))
      return std::string(v);
    throw std::runtime_error(std::string("Missing attribute '") + a + "' (or '" + b + "') in <" + node->Name() +
                             "> at line " + std::to_string(node->GetLineNum()));
  };

  auto parseScopeChild = [&](const tinyxml2::XMLElement* scope) {
    origin_component->host_object = evalTextAttributeRequired(scope, "host");
    origin_component->guest_object = evalTextAttributeRequired(scope, "guest");
  };

  auto& program = origin_component->constraints;

  auto pushTransform = [&](const tinyxml2::XMLElement* tf_elem) {
    // Also create/overwrite the root frame in TransformComponent (like before)
    geometry::TransformNode frame = parseTransformNode(tf_elem);

    auto* tf_component = db.get_or_add<TransformComponent>(eid);
    if (!tf_component)
      throw std::runtime_error("get_or_add<TransformComponent> failed in <Origin>/<Transform>");

    if (tf_component->elements.empty())
      tf_component->elements.emplace_back(id, std::move(frame));
    else
      tf_component->elements[0] = std::make_pair(id, std::move(frame));

    // Push as absolute placement in the program
    geometry::Transform abs{};
    abs.parent = tf_component->elements[0].second.parent;
    abs.tf = tf_component->elements[0].second.local;
    program.emplace_back(abs);
  };

  // Constraint leaf pushers ---------------------------------------------------

  auto pushCoincident = [&](const tinyxml2::XMLElement* node) {
    Coincident c;
    c.host = evalTextAttributeRequired(node, "host");
    c.guest = evalTextAttributeRequired(node, "guest");
    program.emplace_back(c);
  };

  auto pushConcentric = [&](const tinyxml2::XMLElement* node) {
    Concentric c;
    // Accept both spellings: host/guest (preferred) OR host_axis/guest_axis (legacy)
    c.host = getEither(node, "host", "host_axis", "host");
    c.guest = getEither(node, "guest", "guest_axis", "guest");
    program.emplace_back(c);
  };

  auto pushParallel = [&](const tinyxml2::XMLElement* node) {
    Parallel p;
    p.host = getEither(node, "host", "host_axis", "host");
    p.guest = getEither(node, "guest", "guest_axis", "guest");
    program.emplace_back(p);
  };

  auto pushAngle = [&](const tinyxml2::XMLElement* node) {
    Angle a;
    a.host = getEither(node, "host", "host_axis", "host");
    a.guest = getEither(node, "guest", "guest_axis", "guest");
    a.radians = evalNumberAttributeRequired(node, "radians");
    program.emplace_back(a);
  };

  auto pushDistance = [&](const tinyxml2::XMLElement* node) {
    Distance d;
    d.host = evalTextAttributeRequired(node, "host");
    d.guest = evalTextAttributeRequired(node, "guest");
    d.value = evalNumberAttributeRequired(node, "value");
    program.emplace_back(d);
  };

  auto pushSeatConeOnCylinder = [&](const tinyxml2::XMLElement* node) {
    SeatConeOnCylinder s;
    s.host_cyl = evalTextAttributeRequired(node, "host_cyl");
    s.guest_cone = evalTextAttributeRequired(node, "guest_cone");
    s.tol = evalNumberAttribute(node, "tol", 1e-9);
    s.max_it = static_cast<int>(evalNumberAttribute(node, "max_it", 60));
    program.emplace_back(s);
  };

  // Parse children in order ---------------------------------------------------

  for (const auto* child = elem->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();

    if (tag == "Scope")
    {
      parseScopeChild(child);
    }
    else if (tag == "Transform")
    {
      pushTransform(child);
    }
    else if (tag == "Coincident")
    {
      pushCoincident(child);
    }
    else if (tag == "Concentric")
    {
      pushConcentric(child);
    }
    else if (tag == "Parallel")
    {
      pushParallel(child);
    }
    else if (tag == "Angle")
    {
      pushAngle(child);
    }
    else if (tag == "Distance")
    {
      pushDistance(child);
    }
    else if (tag == "SeatConeOnCylinder")
    {
      pushSeatConeOnCylinder(child);
    }

    else
    {
      throw std::runtime_error("Unknown element <" + tag + "> in <Origin> at line " +
                               std::to_string(child->GetLineNum()));
    }
  }

  if (program.empty())
    throw std::runtime_error("Origin '" + id + "' does not contain any valid steps at line " +
                             std::to_string(elem->GetLineNum()));
}

void parseTransformComponent(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem, database::Database& db,
                             database::EntityID eid)
{
  // Skip if this element is <Origin>
  if (std::strcmp(elem->Name(), "Origin") == 0)
    return;

  if (const auto* transform_elem = elem->FirstChildElement("Transform"))
  {
    const std::string id = evalElementIdRequired(elem);

    // Check for duplicates manually if component exists
    if (auto* transform_component = db.get<components::TransformComponent>(eid))
    {
      auto it = std::find_if(transform_component->elements.begin(), transform_component->elements.end(),
                             [&](const auto& p) { return p.first == id; });

      if (it != transform_component->elements.end())
      {
        throw std::runtime_error("Duplicate transform id '" + id + "' at line " + std::to_string(elem->GetLineNum()));
      }
    }

    geometry::TransformNode frame = parseTransformNode(transform_elem);

    // Joints are not static frames
    if (std::strcmp(elem->Name(), "Joint") == 0)
    {
      frame.is_static = false;
      frame.dirty = true;  // ensure first propagate recomputes from joint state
    }

    auto* component = db.get_or_add<components::TransformComponent>(eid);
    component->elements.emplace_back(id, std::move(frame));
  }
}

void parseDerivedFromParallelShapes(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* derived_elem,
                                    database::Database& db, database::EntityID eid)
{
  // --- Resolve parent grasp element id ----------------------------------------------------------
  const tinyxml2::XMLElement* parent = derived_elem->Parent() ? derived_elem->Parent()->ToElement() : nullptr;
  if (!parent)
    throw std::runtime_error(std::string(derived_elem->Name()) + " has no parent element at line " +
                             std::to_string(derived_elem->GetLineNum()));
  const std::string id = evalElementIdRequired(parent);

  // --- Required components ----------------------------------------------------------------------
  auto* shape_component = db.get<components::ShapeComponent>(eid);
  if (!shape_component)
    throw std::runtime_error(std::string("ShapeComponent does not exist; required by ") + derived_elem->Name());
  auto* transform_component = db.get<components::TransformComponent>(eid);
  if (!transform_component)
    throw std::runtime_error(std::string("TransformComponent does not exist; required by ") + derived_elem->Name());

  // --- Parse required axes from XML --------------------------------------------------------------
  const auto* axis_rot_first_elem = derived_elem->FirstChildElement("AxisRotationalFirstShape");
  const auto* axis_norm_first_elem = derived_elem->FirstChildElement("AxisNormalFirstShape");
  const auto* axis_rot_grasp_elem = derived_elem->FirstChildElement("AxisRotationalGrasp");
  if (!axis_rot_first_elem || !axis_norm_first_elem || !axis_rot_grasp_elem)
    throw std::runtime_error("ParallelGrasp <DerivedFromParallelShapes> missing required axis elements "
                             "(AxisRotationalFirstShape, AxisNormalFirstShape, AxisRotationalGrasp required).");

  Eigen::Vector3d axis_rotational_first = parseUnitVector(axis_rot_first_elem);  // intended grasp +X
  Eigen::Vector3d axis_normal_first = parseUnitVector(axis_norm_first_elem);     // intended grasp +Z
  Eigen::Vector3d axis_rotational_grasp = parseUnitVector(axis_rot_grasp_elem);  // metadata (kept as-is)

  // --- Collect referenced shapes ----------------------------------------------------------------
  std::vector<const geometry::Shape*> shape_ptrs;
  std::vector<std::string> shape_ids;
  for (const auto* shape_elem = derived_elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    const std::string sid = evalTextAttributeRequired(shape_elem, "id");
    if (sid.empty())
      throw std::runtime_error("ParallelGrasp <Shape> has empty 'id' at line " +
                               std::to_string(shape_elem->GetLineNum()));
    auto* sp = database::get_element(shape_component->elements, sid);
    if (!sp)
      throw std::runtime_error("No Shape entry found for '" + sid + "' in ShapeComponent");
    shape_ptrs.push_back(sp);
    shape_ids.push_back(sid);
  }

  geometry::TransformNode transform;  // to be pushed for grasp
  ParallelGrasp grasp;                // to be pushed for grasp

  // ==============================================================================================
  // Case A) 2D shape PAIRS (even count >= 2): keep canonical surface as first 2D shape,
  //         with its axes mapped into the grasp frame.
  // ==============================================================================================
  if (shape_ptrs.size() >= 2 && (shape_ptrs.size() % 2 == 0))
  {
    const double CENTROID_TOL = 1e-6;
    size_t num_pairs = shape_ptrs.size() / 2;

    Eigen::Vector3d ref_centroid = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < shape_ptrs.size(); i += 2)
    {
      // Both must be 2D
      if (!is2DShape(*shape_ptrs[i]) || !is2DShape(*shape_ptrs[i + 1]))
        throw std::runtime_error("ParallelGrasp: Only 2D shapes allowed in even-pair case.");

      // Fetch transforms for each 2D shape instance
      auto* tf0 = database::get_element(transform_component->elements, shape_ids[i]);
      auto* tf1 = database::get_element(transform_component->elements, shape_ids[i + 1]);
      if (!tf0 || !tf1)
        throw std::runtime_error("ParallelGrasp: Shape transform missing in TransformComponent.");

      // World-space centroids and normals
      Eigen::Vector3d c0 = tf0->local * getShapeCentroid(*shape_ptrs[i]);
      Eigen::Vector3d c1 = tf1->local * getShapeCentroid(*shape_ptrs[i + 1]);
      Eigen::Vector3d n0 = tf0->local.linear() * getShapePrimaryAxis(*shape_ptrs[i]);
      Eigen::Vector3d n1 = tf1->local.linear() * getShapePrimaryAxis(*shape_ptrs[i + 1]);

      // Normals must be parallel (same or opposite direction)
      if (n0.cross(n1).norm() > 1e-6)
        throw std::runtime_error("2D shapes are not parallel (normals differ)");

      // Centroids must align in-plane (their delta has no in-plane component)
      Eigen::Vector3d delta = c1 - c0;
      Eigen::Vector3d n0hat = n0.normalized();
      double projected = delta.dot(n0hat);
      Eigen::Vector3d delta_in_plane = delta - projected * n0hat;
      if (delta_in_plane.norm() > CENTROID_TOL)
        throw std::runtime_error("2D shape centroids do not align in plane");

      // Save contacts and gap from the first pair
      if (i == 0)
      {
        grasp.contact_shape_ids = { shape_ids[i], shape_ids[i + 1] };
        grasp.gap_size = std::abs((c1 - c0).dot(n0hat));
        ref_centroid = 0.5 * (c0 + c1);
      }
      else
      {
        Eigen::Vector3d centroid = 0.5 * (c0 + c1);
        if (!centroid.isApprox(ref_centroid, CENTROID_TOL))
          throw std::runtime_error("ParallelGrasp: Centroid mismatch for 2D shape pair");
      }
    }

    // Build a clean, right-handed grasp frame from the provided axes
    Eigen::Vector3d Xg = axis_rotational_first;  // intended +X of grasp
    Eigen::Vector3d Zg = axis_normal_first;      // intended +Z of grasp
    Eigen::Vector3d Yg = Zg.cross(Xg);           // Y = Z × X for right-handed
    Eigen::Matrix3d R = geometry::makeOrthonormalRightHanded(Xg, Yg, Zg);

    // Grasp pose in world
    transform.local = Eigen::Isometry3d::Identity();
    transform.local.linear() = R;                  // grasp -> world
    transform.local.translation() = ref_centroid;  // at mid-plane centroid

    // Store user-specified rotation axis (kept in world if that’s your convention)
    grasp.rotation_axis = axis_rotational_grasp;

    // Canonical surface = the FIRST 2D shape, but store its axes in the GRASP frame
    grasp.canonical_surface = *shape_ptrs[0];

    std::vector<Eigen::Vector3d> axes_in_grasp;
    axes_in_grasp.reserve(grasp.canonical_surface.axes.size());
    Eigen::Matrix3d world_to_grasp = R.transpose();
    for (const Eigen::Vector3d& aw : shape_ptrs[0]->axes)
      axes_in_grasp.push_back(world_to_grasp * aw);  // map world-ish axis into grasp frame
    grasp.canonical_surface.axes = std::move(axes_in_grasp);

    // Rotational symmetry: pairs * 2
    grasp.rotational_symmetry = static_cast<uint32_t>(num_pairs * 2);

    // Determine INTERNAL vs EXTERNAL from contact normal and centroid direction
    auto* tf0 = database::get_element(transform_component->elements, shape_ids[0]);
    Eigen::Vector3d c0_world = tf0->local * getShapeCentroid(*shape_ptrs[0]);
    Eigen::Vector3d to_virtual = ref_centroid - c0_world;  // towards the mid-plane
    Eigen::Vector3d n0_world = tf0->local.linear() * getShapePrimaryAxis(*shape_ptrs[0]);

    grasp.grasp_type =
        (to_virtual.dot(n0_world) > 0) ? ParallelGrasp::GraspType::INTERNAL : ParallelGrasp::GraspType::EXTERNAL;
  }
  // ==============================================================================================
  // Case B) Single 3D shape (Cylinder or Box): build a virtual canonical surface in the GRASP frame
  // ==============================================================================================
  else if (shape_ptrs.size() == 1 &&
           (shape_ptrs[0]->type == geometry::ShapeType::Cylinder || shape_ptrs[0]->type == geometry::ShapeType::Box))
  {
    const geometry::Shape& shape3d = *shape_ptrs[0];
    const std::string& sid3d = shape_ids[0];
    grasp.contact_shape_ids = { sid3d };

    auto* shape_tf = database::get_element(transform_component->elements, sid3d);
    if (!shape_tf)
      throw std::runtime_error("No Transform entry found for '" + sid3d + "' in TransformComponent");

    // Interpret provided axes as defined in the shape's LOCAL frame; map to WORLD:
    const Eigen::Matrix3d& S2W = shape_tf->local.linear();
    Eigen::Vector3d Xw = S2W * axis_rotational_first.normalized();  // intended +Xᵍ in world
    Eigen::Vector3d Zw = S2W * axis_normal_first.normalized();      // intended +Zᵍ in world
    Eigen::Vector3d Yw = Zw.cross(Xw);                              // enforce right-handed
    Eigen::Matrix3d R = geometry::makeOrthonormalRightHanded(Xw, Yw, Zw);

    // Place grasp at the 3D shape centroid (Box: usually origin; Cylinder: mid-height on symmetry)
    Eigen::Vector3d centroid_world = shape_tf->local * getShapeCentroid(shape3d);

    transform.local = Eigen::Isometry3d::Identity();
    transform.local.linear() = R;  // grasp -> world
    transform.local.translation() = centroid_world;

    grasp.rotation_axis = axis_rotational_grasp;  // keep as metadata (world)

    // Build the canonical surface in the GRASP frame
    if (shape3d.type == geometry::ShapeType::Cylinder)
    {
      // A line along grasp +X (length = cylinder height)
      const double radius = dim(shape3d).at(geometry::DimRole::Radius);
      const double height = dim(shape3d).at(geometry::DimRole::Height);

      grasp.canonical_surface.type = geometry::ShapeType::Line;
      grasp.canonical_surface.dimensions = { height };
      grasp.canonical_surface.vertices = { -0.5 * height * Eigen::Vector3d::UnitX(),
                                           0.5 * height * Eigen::Vector3d::UnitX() };
      // Axes in grasp frame: direction and reference
      grasp.canonical_surface.axes.clear();
      grasp.canonical_surface.axes.emplace_back(Eigen::Vector3d::UnitX());  // direction
      grasp.canonical_surface.axes.emplace_back(Eigen::Vector3d::UnitZ());  // reference (normal)
      grasp.rotational_symmetry = 0;                                        // treat as infinite
      grasp.gap_size = 2.0 * radius;
    }
    else  // Box -> Rectangle in grasp frame
    {
      // Box dimensions are [width, depth, height]
      const double width = shape3d.dimensions.at(0);
      const double height = shape3d.dimensions.at(2);

      grasp.canonical_surface.type = geometry::ShapeType::Rectangle;
      grasp.canonical_surface.dimensions = { width, height };
      grasp.canonical_surface.vertices = {
        { -0.5 * width, -0.5 * height, 0 },
        { 0.5 * width, -0.5 * height, 0 },
        { 0.5 * width, 0.5 * height, 0 },
        { -0.5 * width, 0.5 * height, 0 },
      };
      // Axes in grasp frame, canonical rectangle order = [Height, Width, Normal] = [Xᵍ, Yᵍ, Zᵍ]
      grasp.canonical_surface.axes.clear();
      grasp.canonical_surface.axes.emplace_back(Eigen::Vector3d::UnitX());  // Height
      grasp.canonical_surface.axes.emplace_back(Eigen::Vector3d::UnitY());  // Width
      grasp.canonical_surface.axes.emplace_back(Eigen::Vector3d::UnitZ());  // Normal

      grasp.rotational_symmetry = 4;
      grasp.gap_size = width;
    }

    // GraspType must be provided for the 3D path
    const auto* approach_elem = derived_elem->FirstChildElement("GraspType");
    if (!approach_elem || !approach_elem->Attribute("value"))
      throw std::runtime_error(
          "ParallelGrasp: <GraspType> with 'value' is required for single 3D shape (box/cylinder).");
    const std::string grasp_type_str = evalTextAttributeRequired(approach_elem, "value");
    if (grasp_type_str == "Internal")
      grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
    else if (grasp_type_str == "External")
      grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
    else
      throw std::runtime_error("ParallelGrasp: <GraspType> value must be 'Internal' or 'External'");
  }
  else
  {
    throw std::runtime_error("ParallelGrasp: Must derive from exactly pairs of two 2D shapes (parallel), "
                             "or one 3D lateral shape (cylinder/box).");
  }

  transform_component->elements.emplace_back(id, std::move(transform));
  auto* grasp_component = db.get_or_add<components::ParallelGraspComponent>(eid);
  grasp_component->elements.emplace_back(id, std::move(grasp));
}

}  // namespace xml
}  // namespace sodf
