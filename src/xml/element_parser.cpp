#include "sodf/xml/element_parser.h"
#include <sodf/xml/expression_parser.h>

namespace sodf {
namespace xml {

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos)
{
  pos.x() = parseRequiredDoubleExpression(element, "x");
  pos.y() = parseRequiredDoubleExpression(element, "y");
  pos.z() = parseRequiredDoubleExpression(element, "z");
}

void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  double roll = parseRequiredDoubleExpression(element, "roll");
  double pitch = parseRequiredDoubleExpression(element, "pitch");
  double yaw = parseRequiredDoubleExpression(element, "yaw");
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  q.x() = parseRequiredDoubleExpression(element, "x");
  q.y() = parseRequiredDoubleExpression(element, "y");
  q.z() = parseRequiredDoubleExpression(element, "z");
  q.w() = parseRequiredDoubleExpression(element, "w");
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
  geometry::TransformNode frame;
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

  vec.x() = parseRequiredDoubleExpression(element, "x");
  vec.y() = parseRequiredDoubleExpression(element, "y");
  vec.z() = parseRequiredDoubleExpression(element, "z");

  double norm = vec.norm();
  if (std::abs(norm - 1.0) > epsilon)
  {
    std::ostringstream oss;
    oss << "Expected unit vector but got vector with norm " << norm << " at line " << element->GetLineNum();
    throw std::runtime_error(oss.str());
  }
  return vec;
}

void validateAxesOrthonormal(const geometry::Shape& shape, const tinyxml2::XMLElement* elem)
{
  const int ln = elem->GetLineNum();
  if (shape.axes.size() == 1)
  {
    if (!geometry::isUnitVector(shape.axes[0]))
      throw std::runtime_error("Shape axis must be a unit vector at line " + std::to_string(ln));
  }
  else if (shape.axes.size() == 2)
  {
    if (!geometry::areVectorsOrthonormal(shape.axes[0], shape.axes[1], 1e-9))
      throw std::runtime_error("Shape axes are not orthonormal at line " + std::to_string(ln));
  }
  else if (shape.axes.size() == 3)
  {
    if (!geometry::areVectorsOrthonormal(shape.axes[0], shape.axes[1], shape.axes[2], 1e-9))
      throw std::runtime_error("Shape axes are not orthonormal at line " + std::to_string(ln));
  }
}

const tinyxml2::XMLElement* reqChild(const tinyxml2::XMLElement* parent, const char* tag, const char* ctx)
{
  if (const auto* e = parent->FirstChildElement(tag))
    return e;
  throw std::runtime_error(std::string(ctx) + " missing <" + tag + "> at line " + std::to_string(parent->GetLineNum()));
}

geometry::Shape parseRectangleShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Rectangle;
  Eigen::Vector3d normal = parseUnitVector(reqChild(elem, "AxisNormal", "Rectangle"));
  Eigen::Vector3d width = parseUnitVector(reqChild(elem, "AxisWidth", "Rectangle"));
  Eigen::Vector3d height = parseUnitVector(reqChild(elem, "AxisHeight", "Rectangle"));
  s.axes = { normal, width, height };

  const auto* dims = reqChild(elem, "Dimensions", "Rectangle");
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "width"));
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "height"));

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseCircleShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Circle;
  Eigen::Vector3d normal = parseUnitVector(reqChild(elem, "AxisNormal", "Circle"));
  Eigen::Vector3d major = parseUnitVector(reqChild(elem, "AxisMajor", "Circle"));
  s.axes = { normal, major };

  const auto* dims = reqChild(elem, "Dimensions", "Circle");
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "radius"));

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseTriangleShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Triangle;
  Eigen::Vector3d normal = parseUnitVector(reqChild(elem, "AxisNormal", "Triangle"));
  Eigen::Vector3d ax = parseUnitVector(reqChild(elem, "AxisX", "Triangle"));
  Eigen::Vector3d ay = parseUnitVector(reqChild(elem, "AxisY", "Triangle"));
  s.axes = { normal, ax, ay };

  const auto* verts = reqChild(elem, "Vertices", "Triangle");
  for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    s.vertices.emplace_back(parseRequiredDoubleExpression(v, "x"), parseRequiredDoubleExpression(v, "y"), 0.0);
  if (s.vertices.size() != 3)
    throw std::runtime_error("Triangle/Polygon has invalid number of vertices.");

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parsePolygonShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Polygon;
  Eigen::Vector3d normal = parseUnitVector(reqChild(elem, "AxisNormal", "Polygon"));
  Eigen::Vector3d ax = parseUnitVector(reqChild(elem, "AxisX", "Polygon"));
  Eigen::Vector3d ay = parseUnitVector(reqChild(elem, "AxisY", "Polygon"));
  s.axes = { normal, ax, ay };

  const auto* verts = reqChild(elem, "Vertices", "Polygon");
  for (const auto* v = verts->FirstChildElement("Vertex"); v; v = v->NextSiblingElement("Vertex"))
    s.vertices.emplace_back(parseRequiredDoubleExpression(v, "x"), parseRequiredDoubleExpression(v, "y"), 0.0);
  if (s.vertices.size() < 3)
    throw std::runtime_error("Triangle/Polygon has invalid number of vertices.");

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseBoxShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Box;
  Eigen::Vector3d ax = parseUnitVector(reqChild(elem, "AxisWidth", "Box"));
  Eigen::Vector3d ay = parseUnitVector(reqChild(elem, "AxisDepth", "Box"));
  Eigen::Vector3d az = parseUnitVector(reqChild(elem, "AxisHeight", "Box"));
  s.axes = { ax, ay, az };

  const auto* dims = reqChild(elem, "Dimensions", "Box");
  const double width = parseRequiredDoubleExpression(dims, "width");
  const double depth = parseRequiredDoubleExpression(dims, "depth");
  const double height = parseRequiredDoubleExpression(dims, "height");
  s.dimensions.push_back(width);
  s.dimensions.push_back(depth);
  s.dimensions.push_back(height);

  // Anchor (default: center)
  // std::string anchor = elem->Attribute("anchor") ? elem->Attribute("anchor") : "center";
  // // Allowed: center, bottom, top
  // //  - center:  box spans [-w/2,+w/2], [-d/2,+d/2], [-h/2,+h/2] about origin
  // //  - bottom:  origin at bottom face center -> center is +h/2 along height axis
  // //  - top:     origin at top face center    -> center is -h/2 along height axis
  // if (anchor == "center")
  // {
  //   s.origin_offset = Eigen::Vector3d::Zero();
  // }
  // else if (anchor == "bottom")
  // {
  //   s.origin_offset = 0.5 * height * az;
  // }
  // else if (anchor == "top")
  // {
  //   s.origin_offset = -0.5 * height * az;
  // }
  // else
  // {
  //   throw std::runtime_error("Box: Unknown anchor '" + anchor +
  //                            "'. Allowed: center, bottom, top. "
  //                            "At line " +
  //                            std::to_string(elem->GetLineNum()));
  // }

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseCylinderShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Cylinder;
  Eigen::Vector3d symmetry = parseUnitVector(reqChild(elem, "AxisSymmetry", "Cylinder"));
  Eigen::Vector3d ref = parseUnitVector(reqChild(elem, "AxisReference", "Cylinder"));
  s.axes = { symmetry, ref };

  const auto* dims = reqChild(elem, "Dimensions", "Cylinder");
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "radius"));
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "height"));

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseSphereShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Sphere;
  const auto* dims = reqChild(elem, "Dimensions", "Sphere");
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "radius"));
  // no axes to validate
  return s;
}

geometry::Shape parseConeShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Cone;
  Eigen::Vector3d symmetry = parseUnitVector(reqChild(elem, "AxisSymmetry", "Cone"));
  Eigen::Vector3d ref = parseUnitVector(reqChild(elem, "AxisReference", "Cone"));
  s.axes = { symmetry, ref };

  const auto* dims = reqChild(elem, "Dimensions", "Cone");
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "base_radius"));
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "top_radius"));
  s.dimensions.push_back(parseRequiredDoubleExpression(dims, "height"));

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseSphericalSegmentShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::SphericalSegment;
  Eigen::Vector3d symmetry = parseUnitVector(reqChild(elem, "AxisSymmetry", "SphericalSegment"));
  Eigen::Vector3d ref = parseUnitVector(reqChild(elem, "AxisReference", "SphericalSegment"));
  s.axes = { symmetry, ref };

  const auto* dims = reqChild(elem, "Dimensions", "SphericalSegment");

  const bool has_r1 = dims->Attribute("base_radius");
  const bool has_r2 = dims->Attribute("top_radius");
  const bool has_h = dims->Attribute("height");
  const int count = static_cast<int>(has_r1) + static_cast<int>(has_r2) + static_cast<int>(has_h);
  if (count < 2)
    throw std::runtime_error(
        "SphericalSegment <Dimensions> must specify at least two of: base_radius, top_radius, height");

  double r1 = has_r1 ? parseRequiredDoubleExpression(dims, "base_radius") : 0.0;
  double r2 = has_r2 ? parseRequiredDoubleExpression(dims, "top_radius") : 0.0;
  double h = has_h ? parseRequiredDoubleExpression(dims, "height") : 0.0;

  if (count == 2)
  {
    if (!has_h)
      h = geometry::inferSegmentHeightFromRadii(r1, r2);
    else if (!has_r2)
      r2 = geometry::inferTopRadiusFromHeight(r1, h);
    else if (!has_r1)
      r1 = geometry::inferTopRadiusFromHeight(r2, h);  // reversed assumption
  }
  else
  {
    if (!geometry::isValidSegment(r1, r2, h))
      throw std::runtime_error("SphericalSegment dimensions are incompatible: no valid sphere exists.");
  }

  s.dimensions = { r1, r2, h };

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parsePlaneShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Plane;
  Eigen::Vector3d normal = parseUnitVector(reqChild(elem, "AxisNormal", "Plane"));
  Eigen::Vector3d ax = parseUnitVector(reqChild(elem, "AxisX", "Plane"));
  Eigen::Vector3d ay = parseUnitVector(reqChild(elem, "AxisY", "Plane"));
  s.axes = { normal, ax, ay };

  if (const auto* dims = elem->FirstChildElement("Dimensions"))
  {
    s.dimensions.push_back(parseRequiredDoubleExpression(dims, "width"));
    s.dimensions.push_back(parseRequiredDoubleExpression(dims, "height"));
  }

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseMeshShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Mesh;
  const auto* file = elem->FirstChildElement("Resource");
  if (!file || !file->Attribute("uri"))
    throw std::runtime_error("Mesh missing <Resource uri=...> at line " + std::to_string(elem->GetLineNum()));
  s.mesh_uri = file->Attribute("uri");

  if (const auto* scale = elem->FirstChildElement("Scale"))
  {
    s.scale.x() = parseDoubleExpression(scale, "x", 1.0);
    s.scale.y() = parseDoubleExpression(scale, "y", 1.0);
    s.scale.z() = parseDoubleExpression(scale, "z", 1.0);
  }
  // no axes to validate
  return s;
}

geometry::Shape parseLineShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Line;

  std::string anchor = elem->Attribute("anchor") ? elem->Attribute("anchor") : "";

  const auto* axis_elem = elem->FirstChildElement("AxisDirection");
  const auto* len_elem = elem->FirstChildElement("Length");
  const auto* vtx_elem = elem->FirstChildElement("Vertex");

  const bool use_axis_length = (axis_elem && len_elem);
  const bool use_vertices = (vtx_elem != nullptr);

  if (use_axis_length && use_vertices)
    throw std::runtime_error("Line: Specify either (AxisDirection + Length) or two Vertex tags, not both.");

  if (use_axis_length)
  {
    if (anchor.empty())
      anchor = "center";

    Eigen::Vector3d axis = parseUnitVector(axis_elem);
    s.axes.push_back(axis);

    double length = parseRequiredDoubleExpression(len_elem, "value");
    if (length <= 0)
      throw std::runtime_error("Line: Length must be positive.");

    const Eigen::Vector3d n = axis.normalized();
    if (anchor == "begin")
    {
      s.vertices.emplace_back(0.0, 0.0, 0.0);
      s.vertices.emplace_back(n * length);
    }
    else if (anchor == "center")
    {
      s.vertices.emplace_back(-0.5 * n * length);
      s.vertices.emplace_back(0.5 * n * length);
    }
    else
    {
      throw std::runtime_error("Line: Unknown anchor '" + anchor + "'. Allowed: begin, center.");
    }
  }
  else if (use_vertices)
  {
    for (const auto* v = vtx_elem; v; v = v->NextSiblingElement("Vertex"))
    {
      double x = parseRequiredDoubleExpression(v, "x");
      double y = parseRequiredDoubleExpression(v, "y");
      double z = 0.0;
      queryDoubleExpression(v, "z", &z);
      s.vertices.emplace_back(x, y, z);
    }
    if (s.vertices.size() != 2)
      throw std::runtime_error("Line: Must have exactly 2 Vertex tags.");

    const bool z0 = (s.vertices[0].z() == 0.0 && s.vertices[1].z() == 0.0);
    const bool zN = (s.vertices[0].z() != 0.0 || s.vertices[1].z() != 0.0);
    if (!z0 && !zN)
      throw std::runtime_error("Line: Vertices must either both have z=0 or both specify z values.");
  }
  else
  {
    throw std::runtime_error("Line: Must specify either (AxisDirection + Length) or two Vertex tags.");
  }

  // If we set s.axes for Axis+Length, validate unit length.
  if (!s.axes.empty())
    validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseShape(const tinyxml2::XMLElement* elem)
{
  geometry::ShapeType type;
  if (const char* type_str = elem->Attribute("type"))
    type = geometry::shapeTypeFromString(type_str);
  else
    throw std::runtime_error("Shape element missing 'type' attribute at line " + std::to_string(elem->GetLineNum()));

  switch (type)
  {
    case geometry::ShapeType::Rectangle:
      return parseRectangleShape(elem);
    case geometry::ShapeType::Circle:
      return parseCircleShape(elem);
    case geometry::ShapeType::Triangle:
      return parseTriangleShape(elem);
    case geometry::ShapeType::Polygon:
      return parsePolygonShape(elem);
    case geometry::ShapeType::Box:
      return parseBoxShape(elem);
    case geometry::ShapeType::Cylinder:
      return parseCylinderShape(elem);
    case geometry::ShapeType::Sphere:
      return parseSphereShape(elem);
    case geometry::ShapeType::Cone:
      return parseConeShape(elem);
    case geometry::ShapeType::SphericalSegment:
      return parseSphericalSegmentShape(elem);
    case geometry::ShapeType::Plane:
      return parsePlaneShape(elem);
    case geometry::ShapeType::Mesh:
      return parseMeshShape(elem);
    case geometry::ShapeType::Line:
      return parseLineShape(elem);
    default:
      throw std::runtime_error("Unknown shape type in parseShapeElement at line " + std::to_string(elem->GetLineNum()));
  }
}

components::StackedShape parseStackedShape(const tinyxml2::XMLElement* stacked_elem)
{
  components::StackedShape stack;

  const tinyxml2::XMLElement* dir_elem = stacked_elem->FirstChildElement("AxisStackDirection");
  if (!dir_elem)
    throw std::runtime_error("<StackedShape> missing <AxisStackDirection> element at line " +
                             std::to_string(stacked_elem->GetLineNum()));
  stack.axis_stack_direction = parseUnitVector(dir_elem);
  stack.axis_stack_direction.normalize();

  const tinyxml2::XMLElement* ref_elem = stacked_elem->FirstChildElement("AxisStackReference");
  if (!ref_elem)
    throw std::runtime_error("<StackedShape> missing <AxisStackReference> element at line " +
                             std::to_string(stacked_elem->GetLineNum()));
  stack.axis_stack_reference = parseUnitVector(ref_elem);
  stack.axis_stack_reference.normalize();

  if (!geometry::areVectorsOrthonormal(stack.axis_stack_direction, stack.axis_stack_reference))
    throw std::runtime_error(std::string("Axes for StackedShape must be mutually orthonormal at line ") +
                             std::to_string(stacked_elem->GetLineNum()));

  Eigen::Vector3d stack_point = Eigen::Vector3d::Zero();  // Current stacking position
  Eigen::Vector3d stack_axis = Eigen::Vector3d::UnitZ();  // Default stacking axis (may be set per shape)

  for (const tinyxml2::XMLElement* shape_elem = stacked_elem->FirstChildElement("Shape"); shape_elem;
       shape_elem = shape_elem->NextSiblingElement("Shape"))
  {
    // Parse geometry with your existing function
    geometry::Shape shape = parseShape(shape_elem);

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
      const Eigen::Vector3d& axis_sym = getShapeSymmetryAxis(shape);
      const Eigen::Vector3d& axis_ref = getShapeReferenceAxis(shape);

      // Place base at current stack_point
      local_tf.translation() = stack_point;
      local_tf.linear() = geometry::buildIsometryFromZXAxes(Eigen::Vector3d::Zero(), axis_sym, axis_ref).linear();
      // Update stack point for next shape
      // stack_point = stack_point + axis * shapeHeight(shape);
      stack_point = axis_sym * shapeHeight(shape);
      stack_axis = axis_sym;
    }

    // Add shape to stack
    stack.shapes.push_back({ shape, local_tf });
  }

  return stack;
}

components::Button parseButton(const tinyxml2::XMLElement* btn_elem)
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
    queryDoubleExpression(detent_elem, "value", &btn.detent_spacing);
  }

  // Optional: <RestPositions>
  if (const tinyxml2::XMLElement* rest_elem = btn_elem->FirstChildElement("RestPositions"))
  {
    for (const tinyxml2::XMLElement* pos_elem = rest_elem->FirstChildElement("Position"); pos_elem;
         pos_elem = pos_elem->NextSiblingElement("Position"))
    {
      double val = 0.0;
      if (queryDoubleExpression(pos_elem, "value", &val))
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
      if (queryDoubleExpression(s_elem, "value", &val))
        btn.stiffness_profile.push_back(val);
    }
  }

  return btn;
}

components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem)
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
  if (!queryDoubleExpression(axis_elem, "x", &x) || !queryDoubleExpression(axis_elem, "y", &y) ||
      !queryDoubleExpression(axis_elem, "z", &z))
    throw std::runtime_error("VirtualButton <AxisPress> missing or invalid x/y/z attribute at line " +
                             std::to_string(axis_elem->GetLineNum()));
  vbtn.press_axis = Eigen::Vector3d(x, y, z);

  // validate unit vector
  if (!geometry::isUnitVector(vbtn.press_axis))
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

components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

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
  queryDoubleExpression(axis_elem, "x", &x);
  queryDoubleExpression(axis_elem, "y", &y);
  queryDoubleExpression(axis_elem, "z", &z);
  joint.axes.col(0) = Eigen::Vector3d(x, y, z);

  // validate unit vector
  if (!geometry::isUnitVector(joint.axes.col(0)))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  // States
  joint.position(0) = joint_elem->FirstChildElement("Position")->DoubleAttribute("value");
  // Velocity
  if (const auto* vel_elem = joint_elem->FirstChildElement("Velocity"))
    joint.velocity(0) = parseRequiredDoubleExpression(vel_elem, "value");

  // Effort
  if (const auto* eff_elem = joint_elem->FirstChildElement("Effort"))
    joint.effort(0) = parseRequiredDoubleExpression(eff_elem, "value");

  // Limits
  if (const auto* limit_elem = joint_elem->FirstChildElement("Limit"))
  {
    if (const auto* pos = limit_elem->FirstChildElement("Position"))
    {
      if (pos->Attribute("min"))
        joint.limit.min_position(0) = parseRequiredDoubleExpression(pos, "min");
      if (pos->Attribute("max"))
        joint.limit.max_position(0) = parseRequiredDoubleExpression(pos, "max");
    }
    if (const auto* vel = limit_elem->FirstChildElement("Velocity"))
    {
      if (vel->Attribute("min"))
        joint.limit.min_velocity(0) = parseRequiredDoubleExpression(vel, "min");
      if (vel->Attribute("max"))
        joint.limit.max_velocity(0) = parseRequiredDoubleExpression(vel, "max");
    }
    if (const auto* acc = limit_elem->FirstChildElement("Acceleration"))
    {
      if (acc->Attribute("min"))
        joint.limit.min_acceleration(0) = parseRequiredDoubleExpression(acc, "min");
      if (acc->Attribute("max"))
        joint.limit.max_acceleration(0) = parseRequiredDoubleExpression(acc, "max");
    }
    if (const auto* jerk = limit_elem->FirstChildElement("Jerk"))
    {
      if (jerk->Attribute("min"))
        joint.limit.min_jerk(0) = parseRequiredDoubleExpression(jerk, "min");
      if (jerk->Attribute("max"))
        joint.limit.max_jerk(0) = parseRequiredDoubleExpression(jerk, "max");
    }
    if (const auto* eff = limit_elem->FirstChildElement("Effort"))
    {
      if (eff->Attribute("min"))
        joint.limit.min_effort(0) = parseRequiredDoubleExpression(eff, "min");
      if (eff->Attribute("max"))
        joint.limit.max_effort(0) = parseRequiredDoubleExpression(eff, "max");
    }
  }

  // Dynamics
  if (const auto* dyn_elem = joint_elem->FirstChildElement("Dynamics"))
  {
    if (const auto* s = dyn_elem->FirstChildElement("Stiffness"))
      joint.dynamics.stiffness(0) = parseRequiredDoubleExpression(s, "value");
    if (const auto* d = dyn_elem->FirstChildElement("Damping"))
      joint.dynamics.damping(0) = parseRequiredDoubleExpression(d, "value");
    if (const auto* r = dyn_elem->FirstChildElement("RestPosition"))
      joint.dynamics.rest_position[0] =
          r->Attribute("value") ? std::optional<double>{ parseRequiredDoubleExpression(r, "value") } : std::nullopt;
    if (const auto* i = dyn_elem->FirstChildElement("Inertia"))
      joint.dynamics.inertia(0) = parseRequiredDoubleExpression(i, "value");
  }
  return joint;
}

components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

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
    queryDoubleExpression(axis_elem, "x", &x);
    queryDoubleExpression(axis_elem, "y", &y);
    queryDoubleExpression(axis_elem, "z", &z);
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
      if (!geometry::isUnitVector(joint.axes.col(0)))
        throw std::runtime_error(std::string("Axis for ") + type_str + " joint must be a unit vector at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;
    case JointType::SPHERICAL:
    case JointType::PLANAR:
      // Both require 3 axes
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
        throw std::runtime_error(std::string("Axes for ") + type_str + " joint must be mutually orthonormal at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;
    case JointType::FLOATING:
      // 6-DOF check both triplets
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::areVectorsOrthonormal(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
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
      v(i) = parseRequiredDoubleExpression(elem, "value");
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
      min_v(i) = parseRequiredDoubleExpression(elem, "min");
      max_v(i) = parseRequiredDoubleExpression(elem, "max");
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
      v(i) = parseRequiredDoubleExpression(elem, "value");
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
      joint.dynamics.rest_position[i] = parseRequiredDoubleExpression(r_elem, "value");
    }
  }

  return joint;
}

components::Joint parseJoint(const tinyxml2::XMLElement* joint_elem)
{
  // Heuristic: multi-DOF if <Axes> exists, single-DOF if <Axis> exists
  if (joint_elem->FirstChildElement("Axes"))
    return parseMultiDofJoint(joint_elem);
  if (joint_elem->FirstChildElement("Axis"))
    return parseSingleDofJoint(joint_elem);
  throw std::runtime_error("Joint element is missing <Axis> or <Axes>");
}

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* elem)
{
  using namespace components;
  ParallelGrasp grasp;

  // Grasp type
  const auto* grasp_type_elem = elem->FirstChildElement("GraspType");
  if (!grasp_type_elem || !grasp_type_elem->Attribute("value"))
    throw std::runtime_error("ParallelGrasp: <GraspType> tag with 'value' is required.");
  std::string grasp_type_val = grasp_type_elem->Attribute("value");
  if (grasp_type_val == "Internal")
    grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
  else if (grasp_type_val == "External")
    grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
  else
    throw std::runtime_error("ParallelGrasp: <GraspType> value must be 'Internal' or 'External'");

  // Gap size
  const auto* gap_elem = elem->FirstChildElement("GapSize");
  grasp.gap_size = gap_elem ? parseRequiredDoubleExpression(gap_elem, "value") : 0.0;
  grasp.gap_axis = parseUnitVector(elem->FirstChildElement("GapAxis"));

  // Rotational axis/symmetry
  grasp.rotation_axis = parseUnitVector(elem->FirstChildElement("RotationalAxis"));
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
  const tinyxml2::XMLElement* vs_elem = elem->FirstChildElement("VirtualSurface");
  if (vs_elem)
  {
    const tinyxml2::XMLElement* shape_elem = vs_elem->FirstChildElement("Shape");
    if (!shape_elem)
    {
      throw std::runtime_error("<VirtualSurface> missing <Shape> child at line " +
                               std::to_string(vs_elem->GetLineNum()));
    }

    grasp.canonical_surface = parseShape(shape_elem);
  }

  return grasp;
}

}  // namespace xml
}  // namespace sodf
