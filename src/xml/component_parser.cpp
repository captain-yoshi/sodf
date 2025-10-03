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

void parseStackedShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  auto stack = parseStackedShape(elem);
  if (stack.shapes.empty())
    throw std::runtime_error("StackedShape '" + std::string(id) + "' is empty at line " +
                             std::to_string(elem->GetLineNum()));

  auto* stacked_shape_component = getOrCreateComponent<StackedShapeComponent>(db, eid);
  stacked_shape_component->elements.emplace_back(id, std::move(stack));
}

void parseParallelGraspComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  if (const auto* derived_elem = elem->FirstChildElement("DerivedFromParallelShapes"))
  {
    parseDerivedFromParallelShapes(derived_elem, db, eid);
    // automatically added to the database
  }
  else
  {
    auto grasp = parseParallelGrasp(elem);

    auto* grasp_component = getOrCreateComponent<ParallelGraspComponent>(db, eid);
    grasp_component->elements.emplace_back(id, std::move(grasp));
  }
}

void parseShapeComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

  auto shape = parseShape(elem);

  auto* shape_component = getOrCreateComponent<ShapeComponent>(db, eid);
  shape_component->elements.emplace_back(id, std::move(shape));
}

void parseLinkComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  Link link;
  const std::string id = evalElementIdRequired(elem);

  // BoundingBox (ptional)
  const tinyxml2::XMLElement* bbox = elem->FirstChildElement("BoundingBox");
  if (bbox)
    if (const auto* shape = bbox->FirstChildElement("Shape"))
      link.bbox = parseBoxShape(shape);

  // Collision (required)
  const auto* collision = elem->FirstChildElement("Collision");
  if (!collision)
    throw std::runtime_error("Link '" + std::string(id) + "' missing required <Collision> element at line " +
                             std::to_string(elem->GetLineNum()));

  const auto* collision_shape = collision->FirstChildElement("Shape");
  if (!collision_shape)
    throw std::runtime_error("Link '" + std::string(id) + "' <Collision> is missing <Shape> element at line " +
                             std::to_string(collision->GetLineNum()));

  link.collision = parseShape(collision_shape);

  // Visual (required)
  const auto* visual = elem->FirstChildElement("Visual");
  if (!visual)
    throw std::runtime_error("Link '" + std::string(id) + "' missing required <Visual> element at line " +
                             std::to_string(elem->GetLineNum()));

  const auto* visual_shape = visual->FirstChildElement("Shape");
  if (!visual_shape)
    throw std::runtime_error("Link '" + std::string(id) + "' <Visual> is missing <Shape> element at line " +
                             std::to_string(visual->GetLineNum()));

  link.visual = parseShape(visual_shape);

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

  auto* tf_component = getOrCreateComponent<TransformComponent>(db, eid);

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
  auto* component = getOrCreateComponent<LinkComponent>(db, eid);
  component->elements.emplace_back(id, std::move(link));
}

void parseInsertionComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  components::Insertion insertion;
  const std::string id = evalElementIdRequired(elem);

  // AxisInsertion (required)
  const auto* axis1 = elem->FirstChildElement("AxisInsertion");
  if (!axis1)
    throw std::runtime_error("Insertion '" + std::string(id) + "' missing <AxisInsertion> element at line " +
                             std::to_string(elem->GetLineNum()));
  insertion.axis_insertion = parseUnitVector(axis1);

  // AxisReference (required)
  const auto* axis2 = elem->FirstChildElement("AxisReference");
  if (!axis2)
    throw std::runtime_error("Insertion '" + std::string(id) + "' missing <AxisReference> element at line " +
                             std::to_string(elem->GetLineNum()));
  insertion.axis_reference = parseUnitVector(axis2);

  // validate orthonormality
  if (!geometry::areVectorsOrthonormal(insertion.axis_insertion, insertion.axis_reference, 1e-09))
    throw std::runtime_error("Insertion '" + std::string(id) + "' axis are not orthogonal at line " +
                             std::to_string(elem->GetLineNum()));

  // RotationalSymmetry (required)
  const auto* sym = elem->FirstChildElement("RotationalSymmetry");
  if (!sym)
    throw std::runtime_error("Insertion '" + std::string(id) + "' missing <RotationalSymmetry> element at line " +
                             std::to_string(elem->GetLineNum()));
  insertion.rotational_symmetry = evalUIntAttributeRequired(sym, "value");

  // ApproachDistance (required)
  const auto* dist = elem->FirstChildElement("ApproachDistance");
  if (!dist)
    throw std::runtime_error("Insertion '" + std::string(id) + "' missing <ApproachDistance> element at line " +
                             std::to_string(elem->GetLineNum()));
  insertion.approach_distance = evalNumberAttributeRequired(dist, "value");

  // store to component
  auto* component = getOrCreateComponent<InsertionComponent>(db, eid);
  component->elements.emplace_back(id, std::move(insertion));
}

void parseTouchscreenComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
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
  auto* component = getOrCreateComponent<TouchscreenComponent>(db, eid);
  component->elements.emplace_back(id, std::move(ts));
}  // namespace xml

void parseFSMComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
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
  auto* component = getOrCreateComponent<FSMComponent>(db, eid);
  component->elements.emplace_back(id, std::move(fsm));
}

void parseActionMapComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
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
  const std::string id = evalElementIdRequired(elem);

  if (const auto* a = elem->FirstChildElement("AxisBottom"))
    container.axis_bottom = parseUnitVector(a);
  else
    throw std::runtime_error("Container missing AxisBottom at line " + std::to_string(elem->GetLineNum()));

  if (const auto* a = elem->FirstChildElement("AxisReference"))
    container.axis_reference = parseUnitVector(a);
  else
    throw std::runtime_error("Container missing AxisReference at line " + std::to_string(elem->GetLineNum()));

  if (!geometry::areVectorsOrthonormal(container.axis_bottom, container.axis_reference, 1e-09))
    throw std::runtime_error("Axes are not orthonormal at line " + std::to_string(elem->GetLineNum()));

  // Content (first one only)
  if (const auto* cont = elem->FirstChildElement("Content"))
  {
    container.content_type = evalTextAttribute(cont, "type", "");
    const double volume = evalNumberAttribute(cont, "volume", 0.0);
    const std::string units = evalTextAttribute(cont, "units", "m^3");
    container.volume = convertVolumeToSI(volume, units.c_str());
  }

  // Parse <DomainShapeRef>
  if (const tinyxml2::XMLElement* domainShapeRefElem = elem->FirstChildElement("DomainShapeRef"))
  {
    container.domain_shape_id = evalTextAttributeRequired(domainShapeRefElem, "id");
    if (container.domain_shape_id.empty())
    {
      throw std::runtime_error("Empty 'id' in <DomainShapeRef> at line " +
                               std::to_string(domainShapeRefElem->GetLineNum()));
    }
  }

  // Parse <LiquidLevelJointRef>
  if (const tinyxml2::XMLElement* liquidLevelJointRefElem = elem->FirstChildElement("LiquidLevelJointRef"))
  {
    container.liquid_level_joint_id = evalTextAttributeRequired(liquidLevelJointRefElem, "id");
    if (container.liquid_level_joint_id.empty())
    {
      throw std::runtime_error("Empty 'id' in <LiquidLevelJointRef> at line " +
                               std::to_string(liquidLevelJointRefElem->GetLineNum()));
    }
  }

  auto* tf_component = getOrCreateComponent<TransformComponent>(db, eid);

  auto* domain_shape = getComponentElement<components::DomainShapeComponent>(db, eid, container.domain_shape_id);
  if (!domain_shape)
    throw std::runtime_error("DomainShape id '" + container.domain_shape_id + "' not found in component");

  // Create bottom transform
  {
    geometry::TransformNode tf_node;
    tf_node.parent = id;
    tf_component->elements.emplace_back(std::string(id) + "/bottom", std::move(tf_node));
  }

  // Retrieve max fill height from domain shape
  double max_height = getMaxFillHeight(domain_shape->domains);
  double curr_height = getFillHeight(domain_shape->domains, container.volume, 1e-09);
  auto upward_axis = -container.axis_bottom;

  // Create virtual prismatic joint
  if (!container.liquid_level_joint_id.empty())
  {
    auto* joint_component = getOrCreateComponent<JointComponent>(db, eid);
    components::Joint joint;
    joint.type = components::JointType::PRISMATIC;
    joint.actuation = components::JointActuation::VIRTUAL;
    joint.initialize_for_type();  // 1 DOF
    joint.axes.col(0) = upward_axis;
    joint.position[0] = curr_height;
    joint.limit.min_position[0] = 0.0;
    joint.limit.max_position[0] = max_height;

    joint_component->elements.emplace_back("joint/" + std::string(id), std::move(joint));

    // Add TransformNode under the joint
    geometry::TransformNode joint_tf;
    joint_tf.parent = std::string(id) + "/bottom";
    joint_tf.local = Eigen::Isometry3d::Identity();
    joint_tf.is_static = false;
    tf_component->elements.emplace_back("joint/" + std::string(id), std::move(joint_tf));

    // Add TransformNode under the joint
    geometry::TransformNode liquid_tf;
    liquid_tf.parent = "joint/" + std::string(id);
    liquid_tf.local = Eigen::Isometry3d::Identity();
    tf_component->elements.emplace_back(std::string(id) + "/liquid_level", std::move(liquid_tf));
  }

  // Add top TransformNode
  {
    geometry::TransformNode top_node;
    top_node.parent = std::string(id) + "/bottom";
    top_node.local.linear() = Eigen::Matrix3d::Identity();
    top_node.local.translation() = upward_axis * max_height;
    tf_component->elements.emplace_back(std::string(id) + "/top", std::move(top_node));
  }

  // store to component
  auto* component = getOrCreateComponent<ContainerComponent>(db, eid);
  component->elements.emplace_back(id, std::move(container));
}

void parseButtonComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  auto btn = parseButton(elem);

  // validate link + joint exists in database
  {
    auto component = db.get_component<components::LinkComponent*>(eid);
    if (!component)
      throw std::runtime_error("LinkComponent does not exists, required by ButtonComponent");

    auto link = getComponentElement(component->elements, btn.link_id);
    if (!link)
      throw std::runtime_error("No Link entry found for '" + btn.link_id + "' in LinkComponent");
  }
  {
    auto component = db.get_component<components::JointComponent*>(eid);
    if (!component)
      throw std::runtime_error("JointComponent does not exists, required by ButtonComponent");

    auto joint = getComponentElement(component->elements, btn.joint_id);
    if (!joint)
      throw std::runtime_error("No Joint entry found for '" + btn.joint_id + "' in JointComponent");
  }

  auto* btn_component = getOrCreateComponent<ButtonComponent>(db, eid);
  const std::string id = evalElementIdRequired(elem);
  btn_component->elements.emplace_back(id, std::move(btn));
}

void parseVirtualButtonComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  auto button = parseVirtualButton(elem);

  auto* btn_component = getOrCreateComponent<VirtualButtonComponent>(db, eid);
  const std::string id = evalElementIdRequired(elem);
  btn_component->elements.emplace_back(id, std::move(button));
}

void parseJointComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  auto joint = parseJoint(elem);

  auto* joint_component = getOrCreateComponent<JointComponent>(db, eid);
  const std::string id = evalElementIdRequired(elem);
  joint_component->elements.emplace_back(id, std::move(joint));
}

void parseFluidDomainShapeComponent(const tinyxml2::XMLElement* fluidElem, ginseng::database& db, EntityID eid)
{
  auto* stack_component = db.get_component<components::StackedShapeComponent*>(eid);
  if (!stack_component)
    throw std::runtime_error("StackedShapeComponent missing from entity!");

  const std::string id = evalElementIdRequired(fluidElem);

  // <StackedShape id="..."/>
  const tinyxml2::XMLElement* stackElem = fluidElem->FirstChildElement("StackedShape");
  if (!stackElem)
    throw std::runtime_error("FluidDomainShape missing <StackedShape>");

  const std::string stack_id = evalTextAttributeRequired(stackElem, "id");
  if (stack_id.empty())
    throw std::runtime_error("Empty 'id' in <StackedShape> at line " + std::to_string(stackElem->GetLineNum()));

  DomainShape domain_shape;
  domain_shape.stacked_shape_id = stack_id;

  // Find the stack in the ECS
  auto stacked_shapes_ptr = getComponentElement(stack_component->elements, domain_shape.stacked_shape_id);
  if (!stacked_shapes_ptr)
    throw std::runtime_error("No StackedShape entry for id '" + domain_shape.stacked_shape_id +
                             "' in StackedShapeComponent");

  // For each shape in the stack
  for (const StackedShapeEntry& entry : (*stacked_shapes_ptr).shapes)
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
      case geometry::ShapeType::Cylinder:
      {
        double radius = geom.dimensions.at(0);
        double height = geom.dimensions.at(1);
        shape = std::make_shared<physics::FluidCylinderShape>(radius, height);
        break;
      }
      case geometry::ShapeType::Box:
      {
        double width = geom.dimensions.at(0);
        double length = geom.dimensions.at(1);  // Or "length" if that's your convention
        double height = geom.dimensions.at(2);
        shape = std::make_shared<physics::FluidBoxShape>(width, length, height);
        break;
      }
      case geometry::ShapeType::SphericalSegment:
      {
        double base_radius = geom.dimensions.at(0);
        double top_radius = geom.dimensions.at(1);  // Or "length" if that's your convention
        double height = geom.dimensions.at(2);

        shape = std::make_shared<physics::FluidSphericalSegmentShape>(base_radius, top_radius, height);
        break;
      }
      case geometry::ShapeType::Cone:
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

void parseProductComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  // Note: the object id has already been populated prior this stage
  auto* component = getOrCreateComponent<ObjectComponent>(db, eid);

  if (auto* name = elem->FirstChildElement("Name"))
    component->name = evalTextNode(name);
  if (auto* model = elem->FirstChildElement("Model"))
    component->model = evalTextNode(model);
  if (auto* sn = elem->FirstChildElement("SerialNumber"))
    component->serial_number = evalTextNode(sn);
  if (auto* vendor = elem->FirstChildElement("Vendor"))
    component->vendor = evalTextNode(vendor);
}

void parseOriginComponent(const tinyxml2::XMLElement* elem, ginseng::database& db, EntityID eid)
{
  const std::string id = evalElementIdRequired(elem);

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
    geometry::TransformNode frame = parseTransformNode(tf_elem);

    auto* tf_component = getOrCreateComponent<TransformComponent>(db, eid);

    // Overwrite or insert as first entry
    if (tf_component->elements.empty())
      tf_component->elements.emplace_back(id, std::move(frame));
    else
      tf_component->elements[0] = std::make_pair(id, std::move(frame));

    auto* origin_component = getOrCreateComponent<OriginComponent>(db, eid);
    origin_component->origin = geometry::Transform{ .parent = frame.parent, .tf = frame.local };
  }
  else if (align_frames_elem)
  {
    geometry::AlignFrames align;
    align.target_id = evalTextAttributeRequired(align_frames_elem, "with");
    if (const auto* src = align_frames_elem->FirstChildElement("Source"))
      align.source_tf = evalTextAttribute(src, "name", "");
    if (const auto* tgt = align_frames_elem->FirstChildElement("Target"))
      align.target_tf = evalTextAttribute(tgt, "name", "");

    auto* origin_component = getOrCreateComponent<OriginComponent>(db, eid);
    origin_component->origin = align;
  }
  else if (align_pair_frames_elem)
  {
    geometry::AlignPairFrames align;
    align.target_id = evalTextAttributeRequired(align_pair_frames_elem, "with");
    align.tolerance = evalNumberAttributeRequired(align_pair_frames_elem, "tolerance");

    int found = 0;
    for (const auto* tag = align_pair_frames_elem->FirstChildElement(); tag; tag = tag->NextSiblingElement())
    {
      const std::string tname = tag->Name();
      const std::string name = evalTextAttributeRequired(tag, "name");

      if (tname == "Source" || tname == "SourceTransform")
        (found == 0 ? align.source_tf1 :
                      found == 1 ?
                      align.source_tf2 :
                      throw std::runtime_error("Too many <Source> ... at line " + std::to_string(tag->GetLineNum())));
      else if (tname == "Target" || tname == "TargetTransform")
        (found == 2 ? align.target_tf1 :
                      found == 3 ?
                      align.target_tf2 :
                      throw std::runtime_error("Too many <Target> ... at line " + std::to_string(tag->GetLineNum())));
      else
        throw std::runtime_error("Unexpected tag <" + tname + "> in <AlignGeometricPair> at line " +
                                 std::to_string(tag->GetLineNum()));
      ++found;
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
    const std::string id = evalElementIdRequired(elem);

    // Check for duplicates manually if component exists
    if (auto* transform_component = db.get_component<components::TransformComponent*>(eid))
    {
      auto it = std::find_if(transform_component->elements.begin(), transform_component->elements.end(),
                             [&](const auto& p) { return p.first == id; });

      if (it != transform_component->elements.end())
      {
        throw std::runtime_error("Duplicate transform id '" + id + "' at line " + std::to_string(elem->GetLineNum()));
      }
    }

    geometry::TransformNode frame = parseTransformNode(transform_elem);

    auto* component = getOrCreateComponent<components::TransformComponent>(db, eid);
    component->elements.emplace_back(id, std::move(frame));
  }
}

void parseDerivedFromParallelShapes(const tinyxml2::XMLElement* derived_elem, ginseng::database& db, EntityID eid)
{
  const tinyxml2::XMLElement* parent = derived_elem->Parent()->ToElement();
  if (!parent)
    throw std::runtime_error(std::string(derived_elem->Name()) + " has not parent element at line " +
                             std::to_string(derived_elem->GetLineNum()));

  const std::string id = evalElementIdRequired(parent);

  auto shape_component = db.get_component<components::ShapeComponent*>(eid);
  if (!shape_component)
    throw std::runtime_error(std::string("ShapeComponent does not exists, required by ") + derived_elem->Name());

  auto transform_component = db.get_component<components::TransformComponent*>(eid);
  if (!transform_component)
    throw std::runtime_error(std::string("TransformComponent does not exists, required by ") + derived_elem->Name());

  geometry::TransformNode transform;

  ParallelGrasp grasp;
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
    const std::string shape_id = evalTextAttributeRequired(shape_elem, "id");
    if (shape_id.empty())
      throw std::runtime_error("ParallelGrasp <Shape> has empty 'id' at line " +
                               std::to_string(shape_elem->GetLineNum()));

    auto shape = getComponentElement(shape_component->elements, shape_id);
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
      auto tf0 = getComponentElement(transform_component->elements, shape_ids[i]);
      auto tf1 = getComponentElement(transform_component->elements, shape_ids[i + 1]);
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
    Eigen::Matrix3d rot = geometry::computeOrientationFromAxes(x_axis, y_axis, z_axis);

    transform.local = Eigen::Isometry3d::Identity();
    transform.local.linear() = rot;
    transform.local.translation() = ref_centroid;

    grasp.rotation_axis = axis_rotational_grasp;
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
        (getComponentElement(transform_component->elements, shape_ids[0])->local * getShapeCentroid(*shape_ptrs[0]));
    Eigen::Vector3d normal = rot.col(2);

    auto tf0 = getComponentElement(transform_component->elements, shape_ids[0]);
    Eigen::Vector3d normal_contact = tf0->local.linear() * getShapeNormalAxis(*shape_ptrs[0]);

    grasp.grasp_type =
        (to_virtual.dot(normal_contact) > 0) ? ParallelGrasp::GraspType::INTERNAL : ParallelGrasp::GraspType::EXTERNAL;
  }

  // ---- 3D shape, single (cylinder/box) ----
  else if (shape_ptrs.size() == 1 &&
           (shape_ptrs[0]->type == geometry::ShapeType::Cylinder || shape_ptrs[0]->type == geometry::ShapeType::Box))
  {
    grasp.contact_shape_ids = { shape_ids[0] };
    auto shape_tf = getComponentElement(transform_component->elements, shape_ids[0]);
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

    grasp.rotation_axis = axis_rotational_grasp;

    Eigen::Matrix3d grasp_to_world = transform.local.linear();
    Eigen::Matrix3d world_to_grasp = grasp_to_world.transpose();  // since R is orthogonal
    const auto& shape_axes = shape_ptrs[0]->axes;                 // {AxisWidth, AxisDepth, AxisHeight} in shape-local
    Eigen::Matrix3d shape_to_world = shape3d_to_parent;
    Eigen::Matrix3d shape_to_grasp = world_to_grasp * shape_to_world;

    // Canonical surface: for cylinder, a line; for box, a rectangle
    if (shape_ptrs[0]->type == geometry::ShapeType::Cylinder)
    {
      grasp.canonical_surface.type = geometry::ShapeType::Line;
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
    else if (shape_ptrs[0]->type == geometry::ShapeType::Box)
    {
      double width = shape_ptrs[0]->dimensions.at(0);
      double height = shape_ptrs[0]->dimensions.at(2);
      grasp.canonical_surface.type = geometry::ShapeType::Rectangle;
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

    // GraspType: must be specified
    const auto* approach_elem = derived_elem->FirstChildElement("GraspType");
    if (!approach_elem || !approach_elem->Attribute("value"))
      throw std::runtime_error(
          "ParallelGrasp: <GraspType> tag with 'value' attribute is required for single 3D shape (box/cylinder).");

    const std::string grasp_type_str = evalTextAttributeRequired(approach_elem, "value");
    if (grasp_type_str == "Internal")
      grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
    else if (grasp_type_str == "External")
      grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
    else
      throw std::runtime_error("ParallelGrasp: <GraspType> value must be 'Internal' or 'External'");
  }
  else
    throw std::runtime_error("ParallelGrasp: Must derive from exactly pairs of two 2D shapes (parallel), or one 3D "
                             "lateral shape (cylinder/box).");

  transform_component->elements.emplace_back(id, std::move(transform));

  auto* grasp_component = getOrCreateComponent<ParallelGraspComponent>(db, eid);
  grasp_component->elements.emplace_back(id, std::move(grasp));
}

}  // namespace xml
}  // namespace sodf
