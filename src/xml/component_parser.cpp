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

std::string evalElementIdRequired(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx)
{
  std::string id;
  if (!tryEvalTextAttribute(elem, "id", &id, ctx))
    throw_xml_error(elem, ctx, std::string(elem ? elem->Name() : "<null>") + " element missing required 'id' attribute");

  if (id.empty())
    throw_xml_error(elem, ctx, std::string(elem ? elem->Name() : "<null>") + " 'id' attribute cannot be empty");
  return id;
}

std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* trans_elem,
                                                          const FSMLabels& state_labels, const FSMLabels& action_labels,
                                                          int invalid_value, const XMLParseContext& ctx)
{
  const int num_states = static_cast<int>(state_labels.id_to_string.size());
  const int num_actions = static_cast<int>(action_labels.id_to_string.size());

  std::vector<std::vector<int>> transitions(num_states, std::vector<int>(num_actions, invalid_value));

  for (const tinyxml2::XMLElement* tr_elem = trans_elem->FirstChildElement("Transition"); tr_elem;
       tr_elem = tr_elem->NextSiblingElement("Transition"))
  {
    std::string from = evalTextAttributeRequired(tr_elem, "from", ctx);
    std::string action = evalTextAttributeRequired(tr_elem, "action", ctx);
    std::string to = evalTextAttributeRequired(tr_elem, "to", ctx);

    if (!state_labels.has_label(from) || !state_labels.has_label(to))
    {
      throw_xml_error(tr_elem, ctx, "Transition refers to undefined state (from='" + from + "', to='" + to + "')");
    }
    if (!action_labels.has_label(action))
    {
      throw_xml_error(tr_elem, ctx, "Transition refers to undefined action (action='" + action + "')");
    }

    int from_id = state_labels.to_id(from);
    int to_id = state_labels.to_id(to);
    int action_id = action_labels.to_id(action);

    if (transitions[from_id][action_id] != invalid_value)
    {
      {
        throw_xml_error(tr_elem, ctx, "Duplicate transition from='" + from + "' action='" + action + "'");
      }
    }

    transitions[from_id][action_id] = to_id;
  }

  return transitions;
}

std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* elem,
                                                          const FSMLabels& state_labels, const FSMLabels& action_labels,
                                                          const XMLParseContext& ctx)
{
  return buildTransitionTableFromXML(elem, state_labels, action_labels, -1, ctx);
}

void parseStackedShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem, ctx);

  auto stack = parseStackedShape(elem, ctx);
  if (stack.shapes.empty())
  {
    throw_xml_error(elem, ctx, "StackedShape '" + id + "' must contain at least one shape");
  }

  auto* stacked_shape_component = db.get_or_add<StackedShapeComponent>(eid);
  stacked_shape_component->elements.emplace_back(id, std::move(stack));
}

void parseStackedShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                   database::EntityID eid)
{
  // (void)doc;
  const std::string id = evalElementIdRequired(elem, ctx);

  std::string ref_id = evalTextAttributeRequired(elem, "ref", ctx);
  if (ref_id.empty())
  {
    throw_xml_error(elem, ctx, "Empty 'ref' attribute in <StackedShapeRef> for id='" + id + "'");
  }

  auto* stacked_shape_component = db.get_or_add<components::StackedShapeComponent>(eid);
  stacked_shape_component->elements.emplace_back(id, std::move(ref_id));
}

void parseParallelGraspComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                 database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem, ctx);

  if (const auto* derived_elem = elem->FirstChildElement("DerivedFromParallelShapes"))
  {
    parseDerivedFromParallelShapes(derived_elem, ctx, db, eid);
    // automatically added to the database
  }
  else
  {
    auto grasp = parseParallelGrasp(elem, ctx);

    auto* grasp_component = db.get_or_add<ParallelGraspComponent>(eid);
    grasp_component->elements.emplace_back(id, std::move(grasp));
  }
}

void parseShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                         database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem, ctx);

  auto shape = parseShape(elem, ctx);

  auto* shape_component = db.get_or_add<ShapeComponent>(eid);
  shape_component->elements.emplace_back(id, std::move(shape));
}

void parseShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                            database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem, ctx);

  std::string ref_id = evalTextAttributeRequired(elem, "ref", ctx);
  if (ref_id.empty())
  {
    throw_xml_error(elem, ctx, "Empty 'ref' attribute in <ShapeRef> for id='" + id + "'");
  }

  auto* shape_component = db.get_or_add<components::ShapeComponent>(eid);
  shape_component->elements.emplace_back(id, std::move(ref_id));
}

void parseDomainShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                               database::EntityID eid)
{
  using namespace components;
  if (!elem)
  {
    throw_xml_error(nullptr, ctx, "parseDomainShapeComponent: null <DomainShape> element");
  }

  // ----- id (required) -----
  const std::string id = evalElementIdRequired(elem, ctx);

  // ----- type (optional, default: Fluid) -----
  auto parseDomainType = [&](const char* t) -> physics::DomainType {
    if (!t || !*t)
      return physics::DomainType::Fluid;
    std::string s(t);
    for (auto& c : s)
      c = static_cast<char>(std::tolower(c));
    if (s == "fluid")
      return physics::DomainType::Fluid;
    throw_xml_error(elem, ctx, "Unsupported DomainShape type='" + std::string(t) + "' for id='" + id + "'");
  };
  const physics::DomainType dtype = parseDomainType(elem->Attribute("type"));

  // ----- <StackedShapeRef id="..."/> (optional but validated if present) -----
  std::string stacked_shape_id;
  const auto* ssr = elem->FirstChildElement("StackedShapeID");
  if (!ssr)
  {
    throw_xml_error(elem, ctx, "<DomainShape id='" + id + "'> requires a <StackedShapeID> child element");
  }

  stacked_shape_id = evalTextAttributeRequired(ssr, "id", ctx);
  if (stacked_shape_id.empty())
  {
    throw_xml_error(ssr, ctx, "Empty 'id' in <StackedShapeID> for DomainShape '" + id + "'");
  }

  auto* stacked_shape = db.get_element<StackedShapeComponent>(eid, stacked_shape_id);
  if (!stacked_shape)
  {
    throw_xml_error(ssr, ctx,
                    "StackedShape '" + stacked_shape_id +
                        "' not found in StackedShapeComponent "
                        "(referenced by DomainShape '" +
                        id + "')");
  }

  // ----- Build component payload -----
  physics::DomainShape ds(*stacked_shape, dtype);
  ds.type = dtype;
  ds.stacked_shape_id = stacked_shape_id;

  // ----- Store to ECS -----
  auto* comp = db.get_or_add<components::DomainShapeComponent>(eid);
  if (!comp)
  {
    throw_xml_error(elem, ctx, "Failed to create DomainShapeComponent for id='" + id + "'");
  }
  comp->elements.emplace_back(id, std::move(ds));
}

void parseDomainShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                  database::EntityID eid)
{
  const std::string id = evalElementIdRequired(elem, ctx);

  std::string ref_id = evalTextAttributeRequired(elem, "ref", ctx);
  if (ref_id.empty())
  {
    throw_xml_error(elem, ctx, "Empty 'ref' attribute in <DomainShapeRef> for id='" + id + "'");
  }

  auto* domain_shape_component = db.get_or_add<components::DomainShapeComponent>(eid);
  domain_shape_component->elements.emplace_back(id, std::move(ref_id));
}

void parseLinkComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                        database::EntityID eid)
{
  Link link;
  const std::string id = evalElementIdRequired(elem, ctx);

  // BoundingBox (ptional)
  const tinyxml2::XMLElement* bbox = elem->FirstChildElement("BoundingBox");
  if (bbox)
    if (const auto* shape = bbox->FirstChildElement("Shape"))
      link.bbox = parseShape(shape, ctx);

  // Collision (required)
  const auto* collision = elem->FirstChildElement("Collision");
  if (!collision)
  {
    throw_xml_error(elem, ctx, "Link '" + id + "' is missing required <Collision> element");
  }

  const auto* collision_shape = collision->FirstChildElement("Shape");
  if (!collision_shape)
  {
    throw_xml_error(collision, ctx, "Link '" + id + "' <Collision> is missing required <Shape> element");
  }

  link.collision = parseShape(collision_shape, ctx);

  // Visual (optional)
  const auto* visual = elem->FirstChildElement("Visual");
  if (visual)
  {
    const auto* visual_shape = visual->FirstChildElement("Shape");
    if (!visual_shape)
    {
      throw_xml_error(visual, ctx, "Link '" + id + "' <Visual> is missing required <Shape> element");
    }

    link.visual = parseShape(visual_shape, ctx);
  }

  // Inertial (optional)
  const tinyxml2::XMLElement* inertial = elem->FirstChildElement("Inertial");
  if (inertial)
  {
    if (const auto* mass = inertial->FirstChildElement("Mass"))
      link.dynamics.mass = evalNumberAttributeRequired(mass, "value", ctx);

    if (const auto* com = inertial->FirstChildElement("CenterOfMass"))
      parsePosition(com, ctx, link.dynamics.center_of_mass);

    if (const auto* I = inertial->FirstChildElement("Tensor"))
    {
      link.dynamics.inertia_tensor << evalNumberAttributeRequired(I, "ixx", ctx),
          evalNumberAttributeRequired(I, "ixy", ctx), evalNumberAttributeRequired(I, "ixz", ctx),
          evalNumberAttributeRequired(I, "ixy", ctx), evalNumberAttributeRequired(I, "iyy", ctx),
          evalNumberAttributeRequired(I, "iyz", ctx), evalNumberAttributeRequired(I, "ixz", ctx),
          evalNumberAttributeRequired(I, "iyz", ctx), evalNumberAttributeRequired(I, "izz", ctx);
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
        node = parseTransformNode(tf, ctx);

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

void parseInsertionComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid)
{
  auto parseRole = [&](const tinyxml2::XMLElement* e, const std::string& insertion_id) -> InsertionRole {
    const char* role_cstr = e->Attribute("role");
    if (!role_cstr)
    {
      throw_xml_error(e, ctx,
                      "Insertion '" + insertion_id +
                          "' missing required attribute 'role' "
                          "(allowed: Receptacle or Insert)");
    }

    const std::string v(role_cstr);

    if (v == "Receptacle")
      return InsertionRole::Receptacle;

    if (v == "Insert")
      return InsertionRole::Insert;

    throw_xml_error(e, ctx,
                    "Insertion '" + insertion_id + "' has invalid role='" + v + "' (allowed: Receptacle or Insert)");
  };

  components::Insertion insertion;
  const std::string id = evalElementIdRequired(elem, ctx);

  insertion.role = parseRole(elem, id);

  // AxisInsertion (required)
  const auto* axis1 = elem->FirstChildElement("AxisInsertion");
  if (!axis1)
  {
    throw_xml_error(elem, ctx, "Insertion '" + id + "' missing required <AxisInsertion> element");
  }
  insertion.axis_insertion = parseUnitVector(axis1, ctx);

  // AxisReference (required)
  const auto* axis2 = elem->FirstChildElement("AxisReference");
  if (!axis2)
  {
    throw_xml_error(elem, ctx, "Insertion '" + id + "' missing required <AxisReference> element");
  }
  insertion.axis_reference = parseUnitVector(axis2, ctx);

  // Validate orthonormality (axis_insertion ⟂ axis_reference, both unit)
  if (!geometry::areVectorsOrthonormal(insertion.axis_insertion, insertion.axis_reference, 1e-09))
  {
    throw_xml_error(elem, ctx, "Insertion '" + id + "' axes must be orthonormal");
  }

  // RotationalSymmetry (required)
  //
  // <RotationalSymmetry value="2"/>
  //
  const auto* sym = elem->FirstChildElement("RotationalSymmetry");
  if (!sym)
  {
    throw_xml_error(elem, ctx, "Insertion '" + id + "' missing required <RotationalSymmetry> element");
  }
  insertion.rotational_symmetry = evalUIntAttributeRequired(sym, "value", ctx);

  // MaxDepth (required)
  const auto* max_depth = elem->FirstChildElement("MaxDepth");
  if (!max_depth)
  {
    throw_xml_error(elem, ctx, "Insertion '" + id + "' missing required <MaxDepth> element");
  }
  insertion.max_depth = evalNumberAttributeRequired(max_depth, "value", ctx);

  if (const auto* shape_ref = elem->FirstChildElement("StackedShapeID"))
  {
    insertion.stacked_shape_id = evalTextAttributeRequired(shape_ref, "id", ctx);
  }

  // Store to component
  auto* component = db.get_or_add<InsertionComponent>(eid);
  component->elements.emplace_back(id, std::move(insertion));
}

void parseTouchscreenComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                               database::EntityID eid)
{
  components::Touchscreen ts;
  const std::string id = evalElementIdRequired(elem, ctx);

  if (const auto* size = elem->FirstChildElement("Size"))
  {
    ts.width = evalNumberAttributeRequired(size, "width", ctx);
    ts.height = evalNumberAttributeRequired(size, "height", ctx);
  }

  if (const auto* normal = elem->FirstChildElement("SurfaceNormal"))
    ts.surface_normal = parseUnitVector(normal, ctx);

  if (const auto* pressure = elem->FirstChildElement("Pressure"))
  {
    ts.min_pressure = evalNumberAttributeRequired(pressure, "min", ctx);
    ts.max_pressure = evalNumberAttributeRequired(pressure, "max", ctx);
  }

  if (const auto* touch = elem->FirstChildElement("Touch"))
  {
    ts.touch_radius = evalNumberAttributeRequired(touch, "radius", ctx);
    ts.multi_touch = evalBoolAttribute(touch, "multi_touch", false, ctx);
    ts.allow_drag = evalBoolAttribute(touch, "allow_drag", false, ctx);
  }

  if (const auto* meta = elem->FirstChildElement("Metadata"))
  {
    if (auto* model = meta->FirstChildElement("Model"))
      ts.model = evalTextNode(model, ctx);
    if (auto* version = meta->FirstChildElement("Version"))
      ts.version = evalTextNode(version, ctx);
    if (auto* driver = meta->FirstChildElement("Driver"))
      ts.driver = evalTextNode(driver, ctx);
  }

  // store to component
  auto* component = db.get_or_add<TouchscreenComponent>(eid);
  component->elements.emplace_back(id, std::move(ts));
}  // namespace xml

void parseFSMComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                       database::EntityID eid)
{
  components::FSM fsm;
  const std::string id = evalElementIdRequired(elem, ctx);

  const auto* start = elem->FirstChildElement("StartState");
  if (!start)
  {
    throw_xml_error(elem, ctx, "FSM '" + id + "' missing required <StartState> element");
  }

  if (!start->Attribute("name"))
  {
    throw_xml_error(start, ctx, "FSM '" + id + "' <StartState> missing required attribute 'name'");
  }

  std::string start_state = evalTextAttributeRequired(start, "name", ctx);

  // Parse states
  if (const auto* states_elem = elem->FirstChildElement("States"))
  {
    for (const auto* state_elem = states_elem->FirstChildElement("State"); state_elem;
         state_elem = state_elem->NextSiblingElement("State"))
    {
      std::string name = evalTextAttributeRequired(state_elem, "name", ctx);
      fsm.state_labels.add(name);
    }
  }

  if (!fsm.state_labels.has_label(start_state))
  {
    throw_xml_error(start, ctx, "FSM '" + id + "' start state '" + start_state + "' is not declared inside <States>");
  }

  fsm.start_state = fsm.state_labels.to_id(start_state);
  fsm.current_state = fsm.start_state;

  // Parse actions
  if (const auto* actions_elem = elem->FirstChildElement("Actions"))
  {
    for (const auto* action_elem = actions_elem->FirstChildElement("Action"); action_elem;
         action_elem = action_elem->NextSiblingElement("Action"))
    {
      std::string name = evalTextAttributeRequired(action_elem, "name", ctx);
      fsm.action_labels.add(name);
    }
  }

  // Parse transitions
  if (const auto* transitions_elem = elem->FirstChildElement("Transitions"))
  {
    fsm.transitions = buildTransitionTableFromXML(transitions_elem, fsm.state_labels, fsm.action_labels, ctx);

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

void parseActionMapComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid)
{
  if (!elem->FirstChildElement("ActionMap"))
    return;

  const std::string component_id = evalElementIdRequired(elem, ctx);
  for (const tinyxml2::XMLElement* action_map_elem = elem->FirstChildElement("ActionMap"); action_map_elem;
       action_map_elem = action_map_elem->NextSiblingElement("ActionMap"))
  {
    const std::string fsm_id = evalTextAttributeRequired(action_map_elem, "fsm", ctx);

    // Build up ActionMap locally
    components::ActionMap action_map;

    for (const tinyxml2::XMLElement* map_elem = action_map_elem->FirstChildElement("Map"); map_elem;
         map_elem = map_elem->NextSiblingElement("Map"))
    {
      components::ActionMapEntry entry;
      // Required
      entry.trigger = evalTextAttributeRequired(map_elem, "trigger", ctx);
      entry.action = evalTextAttributeRequired(map_elem, "action", ctx);

      // Optional: trigger_params
      std::string params;
      if (tryEvalTextAttribute(map_elem, "trigger_params", &params, ctx))
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

void parseContainerComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid)
{
  if (!elem)
  {
    throw_xml_error(nullptr, ctx, "parseContainerComponent: null <Container> element");
  }

  using namespace sodf;
  using namespace sodf::components;

  Container container;

  const std::string id = evalElementIdRequired(elem, ctx);

  // Optional direct attributes
  container.stacked_shape_id = evalTextAttribute(elem, "stacked_shape", "", ctx);
  container.material_id = evalTextAttribute(elem, "material", "", ctx);

  auto* tfc = db.get_or_add<TransformComponent>(eid);
  if (!tfc)
  {
    throw_xml_error(elem, ctx, "Failed to create TransformComponent for Container '" + id + "'");
  }

  // <Content>
  if (const auto* cont = elem->FirstChildElement("Content"))
  {
    container.content_type = evalTextAttribute(cont, "type", "", ctx);
    const double vol_raw = evalNumberAttribute(cont, "volume", 0.0, ctx);
    const std::string units = evalTextAttribute(cont, "units", "m^3", ctx);
    container.payload.volume = convertVolumeToSI(vol_raw, units.c_str());
  }

  // <StackedShapeID> (optional usage-site reference)
  if (const auto* ssid = elem->FirstChildElement("StackedShapeID"))
  {
    container.stacked_shape_id = evalTextAttributeRequired(ssid, "id", ctx);
  }

  // <MaterialID> (optional usage-site reference)
  if (const auto* mid = elem->FirstChildElement("MaterialID"))
  {
    container.material_id = evalTextAttributeRequired(mid, "id", ctx);
  }

  // <PayloadDomain>
  const auto* pd = elem->FirstChildElement("PayloadDomain");
  if (pd)
  {
    // Optional override of volume at payload level
    if (pd->Attribute("volume"))
    {
      const double vol_raw = evalNumberAttribute(pd, "volume", container.payload.volume, ctx);
      const std::string units = evalTextAttribute(pd, "units", "m^3", ctx);
      container.payload.volume = convertVolumeToSI(vol_raw, units.c_str());
    }

    // REQUIRED for fluid logic: <DomainShapeID id="..."/>
    if (const auto* dsid = pd->FirstChildElement("DomainShapeID"))
    {
      container.payload.domain_shape_id = evalTextAttributeRequired(dsid, "id", ctx);
    }

    // Optional live liquid-level frame id (authored frame name)
    if (const auto* lvl = pd->FirstChildElement("LiquidLevelFrame"))
    {
      container.fluid.liquid_level_frame_id = evalTextAttributeRequired(lvl, "id", ctx);
    }
  }

  // Without a domain id we can't build a fluid joint.
  // Still allow the container to exist as a pure logical/static container.
  if (container.payload.domain_shape_id.empty())
  {
    auto* cc = db.get_or_add<ContainerComponent>(eid);
    if (!cc)
    {
      throw_xml_error(elem, ctx, "Failed to create ContainerComponent for Container '" + id + "'");
    }
    cc->elements.emplace_back(id, std::move(container));
    return;
  }

  // --- Use the DomainShape instance to build a PRISMATIC(VIRTUAL) joint ---

  auto* dsc = db.get_element<DomainShapeComponent>(eid, container.payload.domain_shape_id);
  if (!dsc)
  {
    throw_xml_error(elem, ctx,
                    "DomainShape '" + container.payload.domain_shape_id + "' not found for Container '" + id + "'");
  }

  // If this domain shape references a stacked shape, compute total height from it.
  double q_max = 0.0;

  if (!dsc->stacked_shape_id.empty())
  {
    auto* stack = db.get_element<StackedShapeComponent>(eid, dsc->stacked_shape_id);
    if (!stack)
    {
      throw_xml_error(elem, ctx,
                      "StackedShape '" + dsc->stacked_shape_id + "' not found for DomainShape '" +
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
  {
    throw_xml_error(elem, ctx, "Failed to create JointComponent for Container '" + id + "'");
  }
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
  {
    throw_xml_error(elem, ctx, "Failed to create ContainerComponent for Container '" + id + "'");
  }
  cc->elements.emplace_back(id, std::move(container));
}

void parseButtonComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                          database::EntityID eid)
{
  auto btn = parseButton(elem, ctx);
  const std::string id = evalElementIdRequired(elem, ctx);

  // validate link + joint exists in database
  {
    auto* component = db.get<components::LinkComponent>(eid);
    if (!component)
    {
      throw_xml_error(elem, ctx, "Button '" + id + "' requires LinkComponent to be present on entity");
    }

    auto* link = database::get_element(component->elements, btn.link_id);
    if (!link)
    {
      throw_xml_error(elem, ctx, "Button '" + id + "' references unknown Link '" + btn.link_id + "'");
    }
  }
  {
    auto* component = db.get<components::JointComponent>(eid);
    if (!component)
    {
      throw_xml_error(elem, ctx, "Button '" + id + "' requires JointComponent to be present on entity");
    }

    auto* joint = database::get_element(component->elements, btn.joint_id);
    if (!joint)
    {
      throw_xml_error(elem, ctx, "Button '" + id + "' references unknown Joint '" + btn.joint_id + "'");
    }
  }

  auto* btn_component = db.get_or_add<ButtonComponent>(eid);
  btn_component->elements.emplace_back(id, std::move(btn));
}

void parseVirtualButtonComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                 database::EntityID eid)
{
  auto button = parseVirtualButton(elem, ctx);

  auto* btn_component = db.get_or_add<VirtualButtonComponent>(eid);
  const std::string id = evalElementIdRequired(elem, ctx);
  btn_component->elements.emplace_back(id, std::move(button));
}

void parseJointComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                         database::EntityID eid)
{
  auto joint = parseJoint(elem, ctx);

  auto* joint_component = db.get_or_add<JointComponent>(eid);
  const std::string id = evalElementIdRequired(elem, ctx);
  joint_component->elements.emplace_back(id, std::move(joint));
}

void parseProductComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                           database::EntityID eid)
{
  // Note: the object id has already been populated prior this stage
  auto* component = db.get_or_add<ObjectComponent>(eid);

  if (auto* name = elem->FirstChildElement("Name"))
    component->name = evalTextNode(name, ctx);
  if (auto* model = elem->FirstChildElement("Model"))
    component->model = evalTextNode(model, ctx);
  if (auto* sn = elem->FirstChildElement("SerialNumber"))
    component->serial_number = evalTextNode(sn, ctx);
  if (auto* vendor = elem->FirstChildElement("Vendor"))
    component->vendor = evalTextNode(vendor, ctx);
}

void parseOriginComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                          database::EntityID eid)
{
  using namespace components;

  const std::string id = evalElementIdRequired(elem, ctx);

  // Build/append OriginComponent
  auto* origin_component = db.get_or_add<OriginComponent>(eid);
  if (!origin_component)
  {
    throw_xml_error(elem, ctx, "Failed to create OriginComponent for Origin '" + id + "'");
  }

  // Optional scoping from attributes (child <Scope> can override below)
  origin_component->host_object = evalTextAttribute(elem, "host_object", origin_component->host_object, ctx);
  origin_component->guest_object = evalTextAttribute(elem, "guest_object", origin_component->guest_object, ctx);

  // Helpers -------------------------------------------------------------------

  auto getEither = [&](const tinyxml2::XMLElement* node, const char* a, const char* b, const char *
                       /*fallback_name*/) -> std::string {
    // Try 'a' first, then 'b'. If neither present, throw with helpful name.
    if (const char* v = node->Attribute(a))
      return std::string(v);
    if (const char* v = node->Attribute(b))
      return std::string(v);
    throw_xml_error(node, ctx, "Missing attribute '" + std::string(a) + "' (or '" + b + "') in <" + node->Name() + ">");
  };

  auto parseScopeChild = [&](const tinyxml2::XMLElement* scope) {
    origin_component->host_object = evalTextAttributeRequired(scope, "host", ctx);
    origin_component->guest_object = evalTextAttributeRequired(scope, "guest", ctx);
  };

  auto& program = origin_component->constraints;

  auto pushTransform = [&](const tinyxml2::XMLElement* tf_elem) {
    // Also create/overwrite the root frame in TransformComponent (like before)
    geometry::TransformNode frame = parseTransformNode(tf_elem, ctx);

    auto* tf_component = db.get_or_add<TransformComponent>(eid);
    if (!tf_component)
    {
      throw_xml_error(tf_elem, ctx, "Failed to create TransformComponent in <Origin>/<Transform>");
    }

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
    c.host = evalTextAttributeRequired(node, "host", ctx);
    c.guest = evalTextAttributeRequired(node, "guest", ctx);
    program.emplace_back(c);
  };

  auto pushCoincidentPoint = [&](const tinyxml2::XMLElement* node) {
    CoincidentPoint c;
    c.host = evalTextAttributeRequired(node, "host", ctx);
    c.guest = evalTextAttributeRequired(node, "guest", ctx);
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
    a.radians = evalNumberAttributeRequired(node, "radians", ctx);
    program.emplace_back(a);
  };

  auto pushDistance = [&](const tinyxml2::XMLElement* node) {
    Distance d;
    d.host = evalTextAttributeRequired(node, "host", ctx);
    d.guest = evalTextAttributeRequired(node, "guest", ctx);
    d.value = evalNumberAttributeRequired(node, "value", ctx);
    program.emplace_back(d);
  };

  auto pushSeatConeOnCylinder = [&](const tinyxml2::XMLElement* node) {
    SeatConeOnCylinder s;
    s.host_cyl = evalTextAttributeRequired(node, "host_cyl", ctx);
    s.guest_cone = evalTextAttributeRequired(node, "guest_cone", ctx);
    s.tol = evalNumberAttribute(node, "tol", 1e-9, ctx);
    s.max_it = static_cast<int>(evalNumberAttribute(node, "max_it", 60, ctx));
    program.emplace_back(s);
  };

  auto pushFrame = [&](const tinyxml2::XMLElement* node) {
    Frame f;

    f.host = evalTextAttributeRequired(node, "host", ctx);
    f.guest = evalTextAttributeRequired(node, "guest", ctx);

    // Mode (default FULL)
    std::string mode_str = evalTextAttribute(node, "mode", "FULL", ctx);

    if (mode_str == "FULL")
      f.mode = Frame::Mode::FULL;
    else if (mode_str == "POSITION_ONLY")
      f.mode = Frame::Mode::POSITION_ONLY;
    else if (mode_str == "ORIENTATION_ONLY")
      f.mode = Frame::Mode::ORIENTATION_ONLY;
    else if (mode_str == "AXIS_ONLY")
      f.mode = Frame::Mode::AXIS_ONLY;
    else
    {
      throw_xml_error(node, ctx,
                      "Invalid mode '" + mode_str +
                          "' in <Frame> (allowed: FULL, POSITION_ONLY, ORIENTATION_ONLY, AXIS_ONLY)");
    }

    // Optional Offset child
    for (const tinyxml2::XMLElement* child = node->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      if (std::string(child->Name()) == "Offset")
      {
        geometry::TransformNode offset_node = parseTransformNode(child, ctx);

        f.host_offset.parent = offset_node.parent;
        f.host_offset.tf = offset_node.local;
      }
      else
      {
        throw_xml_error(child, ctx, "Unknown child <" + std::string(child->Name()) + "> inside <Frame>");
      }
    }

    program.emplace_back(f);
  };

  auto pushInsertionMate = [&](const tinyxml2::XMLElement* node) {
    InsertionMate m;

    m.host = evalTextAttributeRequired(node, "host", ctx);
    m.guest = evalTextAttributeRequired(node, "guest", ctx);

    // --------------------------------------------
    // Depth mode (STRICT: UPPERCASE only)
    // --------------------------------------------
    std::string mode_str = evalTextAttribute(node, "depth_mode", "AUTO", ctx);

    if (mode_str == "NONE")
    {
      m.depth_mode = InsertionDepthMode::NONE;
    }
    else if (mode_str == "EXPLICIT")
    {
      m.depth_mode = InsertionDepthMode::EXPLICIT;
    }
    else if (mode_str == "AUTO")
    {
      m.depth_mode = InsertionDepthMode::AUTO;
    }
    else
    {
      throw_xml_error(node, ctx, "InsertionMate: depth_mode must be one of {NONE, EXPLICIT, AUTO}");
    }

    // --------------------------------------------
    // Depth value (only meaningful if EXPLICIT)
    // --------------------------------------------
    m.depth = evalNumberAttribute(node, "depth", 0.0, ctx);

    if (m.depth_mode == InsertionDepthMode::EXPLICIT)
    {
      if (!node->Attribute("depth"))
      {
        throw_xml_error(node, ctx, "InsertionMate: 'depth' attribute required when depth_mode=EXPLICIT");
      }
    }

    // --------------------------------------------
    // Optional flags
    // --------------------------------------------
    m.clamp_to_min_depth = evalBoolAttribute(node, "clamp_to_min_depth", true, ctx);

    m.align_reference_axis = evalBoolAttribute(node, "align_reference_axis", true, ctx);

    program.emplace_back(m);
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
    else if (tag == "CoincidentPoint")
    {
      pushCoincidentPoint(child);
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
    else if (tag == "Frame")
    {
      pushFrame(child);
    }
    else if (tag == "InsertionMate")
    {
      pushInsertionMate(child);
    }
    else
    {
      throw_xml_error(child, ctx, "Unknown element <" + tag + "> inside <Origin>");
    }
  }

  if (program.empty())
  {
    throw_xml_error(elem, ctx, "Origin '" + id + "' does not contain any valid steps");
  }
}

void parseTransformComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid)
{
  // Skip if this element is <Origin>
  if (std::strcmp(elem->Name(), "Origin") == 0)
    return;

  if (const auto* transform_elem = elem->FirstChildElement("Transform"))
  {
    const std::string id = evalElementIdRequired(elem, ctx);

    // Check for duplicates manually if component exists
    if (auto* transform_component = db.get<components::TransformComponent>(eid))
    {
      auto it = std::find_if(transform_component->elements.begin(), transform_component->elements.end(),
                             [&](const auto& p) { return p.first == id; });

      if (it != transform_component->elements.end())
      {
        throw_xml_error(elem, ctx, "Duplicate Transform id '" + id + "'");
      }
    }

    geometry::TransformNode frame = parseTransformNode(transform_elem, ctx);

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

void parseDerivedFromParallelShapes(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx,
                                    database::Database& db, database::EntityID eid)
{
  // --- Resolve parent grasp element id ----------------------------------------------------------
  const tinyxml2::XMLElement* parent = elem->Parent() ? elem->Parent()->ToElement() : nullptr;
  if (!parent)
  {
    throw_xml_error(elem, ctx, std::string(elem->Name()) + " has no parent element");
  }

  const std::string id = evalElementIdRequired(parent, ctx);

  // --- Required components ----------------------------------------------------------------------
  auto* shape_component = db.get<components::ShapeComponent>(eid);
  if (!shape_component)
  {
    throw_xml_error(elem, ctx,
                    "ShapeComponent required by <DerivedFromParallelShapes> "
                    "is missing on entity");
  }
  auto* transform_component = db.get<components::TransformComponent>(eid);
  if (!transform_component)
  {
    throw_xml_error(elem, ctx,
                    "TransformComponent required by <DerivedFromParallelShapes> "
                    "is missing on entity");
  }

  // --- Parse required axes from XML --------------------------------------------------------------
  const auto* axis_rot_first_elem = elem->FirstChildElement("AxisRotationalFirstShape");
  const auto* axis_norm_first_elem = elem->FirstChildElement("AxisNormalFirstShape");
  const auto* axis_rot_grasp_elem = elem->FirstChildElement("AxisRotationalGrasp");
  if (!axis_rot_first_elem || !axis_norm_first_elem || !axis_rot_grasp_elem)
  {
    throw_xml_error(elem, ctx,
                    "<DerivedFromParallelShapes> missing required axis elements "
                    "(AxisRotationalFirstShape, AxisNormalFirstShape, AxisRotationalGrasp)");
  }

  Eigen::Vector3d axis_rotational_first = parseUnitVector(axis_rot_first_elem, ctx);  // intended grasp +X
  Eigen::Vector3d axis_normal_first = parseUnitVector(axis_norm_first_elem, ctx);     // intended grasp +Z
  Eigen::Vector3d axis_rotational_grasp = parseUnitVector(axis_rot_grasp_elem, ctx);  // metadata (kept as-is)

  // --- Collect referenced shapes ----------------------------------------------------------------
  std::vector<const geometry::Shape*> shape_ptrs;
  std::vector<std::string> shape_ids;
  for (const auto* shape_elem = elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    const std::string sid = evalTextAttributeRequired(shape_elem, "id", ctx);
    if (sid.empty())
    {
      throw_xml_error(shape_elem, ctx,
                      "<Shape> inside <DerivedFromParallelShapes> "
                      "has empty 'id'");
    }
    auto* sp = database::get_element(shape_component->elements, sid);
    if (!sp)
    {
      throw_xml_error(shape_elem, ctx, "Shape '" + sid + "' not found in ShapeComponent");
    }
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
      {
        throw_xml_error(elem, ctx,
                        "ParallelGrasp: only 2D shapes allowed "
                        "in even-pair mode");
      }

      // Fetch transforms for each 2D shape instance
      auto* tf0 = database::get_element(transform_component->elements, shape_ids[i]);
      auto* tf1 = database::get_element(transform_component->elements, shape_ids[i + 1]);
      if (!tf0 || !tf1)
      {
        throw_xml_error(elem, ctx,
                        "ParallelGrasp: shape transform missing "
                        "in TransformComponent");
      }

      // World-space centroids and normals
      Eigen::Vector3d c0 = tf0->local * getShapeCentroid(*shape_ptrs[i]);
      Eigen::Vector3d c1 = tf1->local * getShapeCentroid(*shape_ptrs[i + 1]);
      Eigen::Vector3d n0 = tf0->local.linear() * getShapePrimaryAxis(*shape_ptrs[i]);
      Eigen::Vector3d n1 = tf1->local.linear() * getShapePrimaryAxis(*shape_ptrs[i + 1]);

      // Normals must be parallel (same or opposite direction)
      if (n0.cross(n1).norm() > 1e-6)
      {
        throw_xml_error(elem, ctx,
                        "2D shapes are not parallel "
                        "(normals differ)");
      }

      // Centroids must align in-plane (their delta has no in-plane component)
      Eigen::Vector3d delta = c1 - c0;
      Eigen::Vector3d n0hat = n0.normalized();
      double projected = delta.dot(n0hat);
      Eigen::Vector3d delta_in_plane = delta - projected * n0hat;
      if (delta_in_plane.norm() > CENTROID_TOL)
      {
        throw_xml_error(elem, ctx,
                        "2D shape centroids do not align "
                        "in-plane");
      }

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
        {
          throw_xml_error(elem, ctx,
                          "ParallelGrasp: centroid mismatch "
                          "between 2D pairs");
        }
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
    {
      throw_xml_error(elem, ctx, "Transform for shape '" + sid3d + "' not found in TransformComponent");
    }

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
    const auto* approach_elem = elem->FirstChildElement("GraspType");
    if (!approach_elem || !approach_elem->Attribute("value"))
    {
      throw_xml_error(elem, ctx,
                      "<GraspType value=\"...\"> required "
                      "for single 3D shape grasp");
    }
    const std::string grasp_type_str = evalTextAttributeRequired(approach_elem, "value", ctx);
    if (grasp_type_str == "Internal")
      grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
    else if (grasp_type_str == "External")
      grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
    else
    {
      throw_xml_error(approach_elem, ctx,
                      "<GraspType> must be "
                      "'Internal' or 'External'");
    }
  }
  else
  {
    throw_xml_error(elem, ctx,
                    "ParallelGrasp must derive from "
                    "pairs of 2D shapes or "
                    "a single 3D cylinder/box");
  }

  transform_component->elements.emplace_back(id, std::move(transform));
  auto* grasp_component = db.get_or_add<components::ParallelGraspComponent>(eid);
  grasp_component->elements.emplace_back(id, std::move(grasp));
}

}  // namespace xml
}  // namespace sodf
