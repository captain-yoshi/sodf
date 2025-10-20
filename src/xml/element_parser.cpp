#include "sodf/xml/element_parser.h"
#include <sodf/xml/expression_parser.h>
#include <sodf/xml/utils.h>

namespace sodf {
namespace xml {

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos)
{
  pos.x() = evalNumberAttributeRequired(element, "x");
  pos.y() = evalNumberAttributeRequired(element, "y");
  pos.z() = evalNumberAttributeRequired(element, "z");
}

void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  double roll = evalNumberAttributeRequired(element, "roll");
  double pitch = evalNumberAttributeRequired(element, "pitch");
  double yaw = evalNumberAttributeRequired(element, "yaw");
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  q = rz * ry * rx;
}

void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q)
{
  q.x() = evalNumberAttributeRequired(element, "x");
  q.y() = evalNumberAttributeRequired(element, "y");
  q.z() = evalNumberAttributeRequired(element, "z");
  q.w() = evalNumberAttributeRequired(element, "w");
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
  (void)tryEvalTextAttribute(transform_elem, "parent", &frame.parent);

  frame.local = parseIsometry3D(transform_elem);
  frame.global = Eigen::Isometry3d::Identity();
  frame.dirty = true;

  return frame;
}

Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* element, double epsilon)
{
  Eigen::Vector3d vec;

  vec.x() = evalNumberAttributeRequired(element, "x");
  vec.y() = evalNumberAttributeRequired(element, "y");
  vec.z() = evalNumberAttributeRequired(element, "z");

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
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "width"));
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "height"));

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
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "radius"));

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
    s.vertices.emplace_back(evalNumberAttributeRequired(v, "x"), evalNumberAttributeRequired(v, "y"), 0.0);
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
    s.vertices.emplace_back(evalNumberAttributeRequired(v, "x"), evalNumberAttributeRequired(v, "y"), 0.0);
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
  const double width = evalNumberAttributeRequired(dims, "width");
  const double depth = evalNumberAttributeRequired(dims, "depth");
  const double height = evalNumberAttributeRequired(dims, "height");
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

geometry::Shape parseTriangularPrismShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::TriangularPrism;

  // Axes
  Eigen::Vector3d ax = parseUnitVector(reqChild(elem, "AxisWidth", "TriangularPrism"));
  Eigen::Vector3d ay = parseUnitVector(reqChild(elem, "AxisDepth", "TriangularPrism"));
  Eigen::Vector3d az = parseUnitVector(reqChild(elem, "AxisHeight", "TriangularPrism"));
  s.axes = { ax, ay, az };

  // Dimensions
  const auto* dims = reqChild(elem, "Dimensions", "TriangularPrism");
  const double bw = evalNumberAttributeRequired(dims, "width");       // base_width along AxisWidth
  const double bh = evalNumberAttributeRequired(dims, "depth");       // base_height along AxisDepth
  const double height = evalNumberAttributeRequired(dims, "height");  // extrusion along AxisHeight

  double u = 0.0;  // default: right-triangle (apex over A’s x)
  (void)tryEvalNumberAttribute(dims, "apex_offset", &u);

  if (bw <= 0.0 || bh <= 0.0 || height <= 0.0)
    throw std::runtime_error("TriangularPrism <Dimensions> require positive width, depth, and height at line " +
                             std::to_string(dims->GetLineNum()));

  s.dimensions = { bw, bh, height, u };

  // Validate orthonormal axes
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
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "radius"));
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "height"));

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseSphereShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Sphere;
  const auto* dims = reqChild(elem, "Dimensions", "Sphere");
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "radius"));
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
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "base_radius"));
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "top_radius"));
  s.dimensions.push_back(evalNumberAttributeRequired(dims, "height"));

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

  double r1 = has_r1 ? evalNumberAttributeRequired(dims, "base_radius") : 0.0;
  double r2 = has_r2 ? evalNumberAttributeRequired(dims, "top_radius") : 0.0;
  double h = has_h ? evalNumberAttributeRequired(dims, "height") : 0.0;

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
    s.dimensions.push_back(evalNumberAttributeRequired(dims, "width"));
    s.dimensions.push_back(evalNumberAttributeRequired(dims, "height"));
  }

  validateAxesOrthonormal(s, elem);
  return s;
}

geometry::Shape parseMeshShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Mesh;
  const auto* file = elem->FirstChildElement("Resource");
  if (!file)
    throw std::runtime_error("Mesh missing <Resource> at line " + std::to_string(elem->GetLineNum()));

  s.mesh_uri = evalTextAttributeRequired(file, "uri");

  // if relative, change to absolute
  s.mesh_uri = resolve_realtive_uri(file, doc, s.mesh_uri);

  if (const auto* scale = elem->FirstChildElement("Scale"))
  {
    s.scale.x() = evalNumberAttributeRequired(scale, "x");
    s.scale.y() = evalNumberAttributeRequired(scale, "y");
    s.scale.z() = evalNumberAttributeRequired(scale, "z");
  }
  // no axes to validate
  return s;
}

geometry::Shape parseLineShape(const tinyxml2::XMLElement* elem)
{
  geometry::Shape s;
  s.type = geometry::ShapeType::Line;

  std::string anchor;
  (void)tryEvalTextAttribute(elem, "anchor", &anchor);

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

    double length = evalNumberAttributeRequired(len_elem, "value");
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
      double x = evalNumberAttributeRequired(v, "x");
      double y = evalNumberAttributeRequired(v, "y");
      double z = 0.0;
      tryEvalNumberAttribute(v, "z", &z);
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

geometry::Shape parseShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
{
  const std::string type_str = evalTextAttributeRequired(elem, "type");
  geometry::ShapeType type = geometry::shapeTypeFromString(type_str.c_str());

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
    case geometry::ShapeType::TriangularPrism:
      return parseTriangularPrismShape(elem);
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
      return parseMeshShape(doc, elem);
    case geometry::ShapeType::Line:
      return parseLineShape(elem);
    default:
      throw std::runtime_error("Unknown shape type in parseShapeElement at line " + std::to_string(elem->GetLineNum()));
  }
}

components::StackedShape parseStackedShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* stacked_elem)
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
    geometry::Shape shape = parseShape(doc, shape_elem);

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

  Button btn;
  btn.type = buttonTypeFromString(evalTextAttributeRequired(btn_elem, "type").c_str());

  const tinyxml2::XMLElement* link_elem = btn_elem->FirstChildElement("Link");
  if (!link_elem)
    throw std::runtime_error("Button missing <Link> at line " + std::to_string(btn_elem->GetLineNum()));
  btn.link_id = evalTextAttributeRequired(link_elem, "ref");

  const tinyxml2::XMLElement* joint_elem = btn_elem->FirstChildElement("Joint");
  if (!joint_elem)
    throw std::runtime_error("Button missing <Joint> at line " + std::to_string(btn_elem->GetLineNum()));
  btn.joint_id = evalTextAttributeRequired(joint_elem, "ref");

  if (const auto* detent_elem = btn_elem->FirstChildElement("DetentSpacing"))
    tryEvalNumberAttribute(detent_elem, "value", &btn.detent_spacing);

  if (const auto* rest_elem = btn_elem->FirstChildElement("RestPositions"))
    for (const auto* pos_elem = rest_elem->FirstChildElement("Position"); pos_elem;
         pos_elem = pos_elem->NextSiblingElement("Position"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(pos_elem, "value", &v))
        btn.rest_positions.push_back(v);
    }

  if (const auto* stiff_elem = btn_elem->FirstChildElement("StiffnessProfile"))
    for (const auto* s_elem = stiff_elem->FirstChildElement("Stiffness"); s_elem;
         s_elem = s_elem->NextSiblingElement("Stiffness"))
    {
      double v = 0.0;
      if (tryEvalNumberAttribute(s_elem, "value", &v))
        btn.stiffness_profile.push_back(v);
    }

  return btn;
}

components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem)
{
  using namespace sodf::components;
  VirtualButton vbtn;

  const auto* shape_elem = vb_elem->FirstChildElement("Shape");
  if (!shape_elem)
    throw std::runtime_error("VirtualButton missing <Shape> at line " + std::to_string(vb_elem->GetLineNum()));
  vbtn.shape_id = evalTextAttributeRequired(shape_elem, "id");

  const auto* axis_elem = vb_elem->FirstChildElement("AxisPress");
  if (!axis_elem)
    throw std::runtime_error("VirtualButton missing <AxisPress> at line " + std::to_string(vb_elem->GetLineNum()));
  const double x = evalNumberAttributeRequired(axis_elem, "x");
  const double y = evalNumberAttributeRequired(axis_elem, "y");
  const double z = evalNumberAttributeRequired(axis_elem, "z");
  vbtn.press_axis = Eigen::Vector3d(x, y, z);
  if (!geometry::isUnitVector(vbtn.press_axis))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  if (const auto* label_elem = vb_elem->FirstChildElement("Label"))
    vbtn.label = evalTextAttribute(label_elem, "text", "");

  if (const auto* img_elem = vb_elem->FirstChildElement("Image"))
    vbtn.image_uri = evalTextAttribute(img_elem, "uri", "");

  return vbtn;
}

components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = joint_elem->FirstChildElement("Type");
  const auto* act_e = joint_elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
    throw std::runtime_error("Joint missing <Type> or <Actuation>");

  joint.type = jointTypeFromString(evalTextAttributeRequired(type_e, "value").c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value").c_str());
  joint.initialize_for_type();

  const auto* axis_elem = joint_elem->FirstChildElement("Axis");
  if (!axis_elem)
    throw std::runtime_error("Missing <Axis>");
  joint.axes.col(0) =
      Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x"), evalNumberAttributeRequired(axis_elem, "y"),
                      evalNumberAttributeRequired(axis_elem, "z"));
  if (!geometry::isUnitVector(joint.axes.col(0)))
    throw std::runtime_error("Axis element must be a unit vector " + std::to_string(axis_elem->GetLineNum()));

  const auto* pos_e = joint_elem->FirstChildElement("Position");
  if (!pos_e)
    throw std::runtime_error("Single-DOF joint missing <Position>");
  joint.position(0) = evalNumberAttributeRequired(pos_e, "value");

  if (const auto* vel_e = joint_elem->FirstChildElement("Velocity"))
    joint.velocity(0) = evalNumberAttributeRequired(vel_e, "value");
  if (const auto* eff_e = joint_elem->FirstChildElement("Effort"))
    joint.effort(0) = evalNumberAttributeRequired(eff_e, "value");

  if (const auto* limit_e = joint_elem->FirstChildElement("Limit"))
  {
    auto opt = [&](const tinyxml2::XMLElement* e, const char* attr, double& out) {
      return e && tryEvalNumberAttribute(e, attr, &out);
    };

    if (const auto* pos = limit_e->FirstChildElement("Position"))
    {
      (void)opt(pos, "min", joint.limit.min_position(0));
      (void)opt(pos, "max", joint.limit.max_position(0));
    }
    if (const auto* vel = limit_e->FirstChildElement("Velocity"))
    {
      (void)opt(vel, "min", joint.limit.min_velocity(0));
      (void)opt(vel, "max", joint.limit.max_velocity(0));
    }
    if (const auto* acc = limit_e->FirstChildElement("Acceleration"))
    {
      (void)opt(acc, "min", joint.limit.min_acceleration(0));
      (void)opt(acc, "max", joint.limit.max_acceleration(0));
    }
    if (const auto* jerk = limit_e->FirstChildElement("Jerk"))
    {
      (void)opt(jerk, "min", joint.limit.min_jerk(0));
      (void)opt(jerk, "max", joint.limit.max_jerk(0));
    }
    if (const auto* eff = limit_e->FirstChildElement("Effort"))
    {
      (void)opt(eff, "min", joint.limit.min_effort(0));
      (void)opt(eff, "max", joint.limit.max_effort(0));
    }
  }

  if (const auto* dyn_e = joint_elem->FirstChildElement("Dynamics"))
  {
    if (const auto* s = dyn_e->FirstChildElement("Stiffness"))
      joint.dynamics.stiffness(0) = evalNumberAttributeRequired(s, "value");
    if (const auto* d = dyn_e->FirstChildElement("Damping"))
      joint.dynamics.damping(0) = evalNumberAttributeRequired(d, "value");
    if (const auto* r = dyn_e->FirstChildElement("RestPosition"))
      joint.dynamics.rest_position[0] =
          r->Attribute("value") ? std::optional<double>{ evalNumberAttributeRequired(r, "value") } : std::nullopt;
    if (const auto* i = dyn_e->FirstChildElement("Inertia"))
      joint.dynamics.inertia(0) = evalNumberAttributeRequired(i, "value");
  }
  return joint;
}

components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem)
{
  using namespace sodf::components;

  Joint joint;

  const auto* type_e = joint_elem->FirstChildElement("Type");
  const auto* act_e = joint_elem->FirstChildElement("Actuation");
  if (!type_e || !act_e)
    throw std::runtime_error("Joint missing <Type> or <Actuation>");

  joint.type = jointTypeFromString(evalTextAttributeRequired(type_e, "value").c_str());
  joint.actuation = jointActuationFromString(evalTextAttributeRequired(act_e, "value").c_str());
  joint.initialize_for_type();  // <-- initialize first
  const int dof = joint.dof();  //     then query DOF

  const auto* axes_elem = joint_elem->FirstChildElement("Axes");
  if (!axes_elem)
    throw std::runtime_error("Multi-DOF joint missing <Axes> at line " + std::to_string(joint_elem->GetLineNum()));

  int axis_i = 0;
  for (const auto* axis_elem = axes_elem->FirstChildElement("Axis"); axis_elem && axis_i < dof;
       axis_elem = axis_elem->NextSiblingElement("Axis"), ++axis_i)
  {
    joint.axes.col(axis_i) =
        Eigen::Vector3d(evalNumberAttributeRequired(axis_elem, "x"), evalNumberAttributeRequired(axis_elem, "y"),
                        evalNumberAttributeRequired(axis_elem, "z"));
  }
  if (axis_i != dof)
    throw std::runtime_error("There must be " + std::to_string(dof) + " <Axis> tags for a " +
                             evalTextAttributeRequired(type_e, "value") + " joint at line " +
                             std::to_string(axes_elem->GetLineNum()));

  // Validate axes
  switch (joint.type)
  {
    case JointType::FIXED:
      // no axes required
      break;

    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      if (!geometry::isUnitVector(joint.axes.col(0)))
        throw std::runtime_error(std::string("Axis for ") + evalTextAttributeRequired(type_e, "value") +
                                 " joint must be a unit vector at line " + std::to_string(axes_elem->GetLineNum()));
      break;

    case JointType::SPHERICAL:
    case JointType::PLANAR:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)))
        throw std::runtime_error(std::string("Axes for ") + evalTextAttributeRequired(type_e, "value") +
                                 " joint must be mutually orthonormal at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;

    case JointType::FLOATING:
      if (!geometry::areVectorsOrthonormal(joint.axes.col(0), joint.axes.col(1), joint.axes.col(2)) ||
          !geometry::areVectorsOrthonormal(joint.axes.col(3), joint.axes.col(4), joint.axes.col(5)))
        throw std::runtime_error("Axes for FLOATING joint must be two mutually orthonormal triplets at line " +
                                 std::to_string(axes_elem->GetLineNum()));
      break;

    default:
      throw std::runtime_error("Unknown JointType for axes validation at line " +
                               std::to_string(axes_elem->GetLineNum()));
  }

  // Fill state vectors
  auto fill_vector = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value");
  };
  fill_vector(joint.position, joint_elem->FirstChildElement("Positions"), "Position");
  fill_vector(joint.velocity, joint_elem->FirstChildElement("Velocities"), "Velocity");
  fill_vector(joint.effort, joint_elem->FirstChildElement("Efforts"), "Effort");

  // Limits
  auto fill_limits = [](Eigen::VectorXd& min_v, Eigen::VectorXd& max_v, const tinyxml2::XMLElement* limits_elem,
                        const char* tag) {
    int i = 0;
    for (const auto* e = limits_elem ? limits_elem->FirstChildElement(tag) : nullptr; e && i < min_v.size();
         e = e->NextSiblingElement(tag), ++i)
    {
      min_v(i) = evalNumberAttributeRequired(e, "min");
      max_v(i) = evalNumberAttributeRequired(e, "max");
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

  // Dynamics
  auto fill_dyn = [](Eigen::VectorXd& v, const tinyxml2::XMLElement* parent, const char* tag) {
    int i = 0;
    for (const auto* e = parent ? parent->FirstChildElement(tag) : nullptr; e && i < v.size();
         e = e->NextSiblingElement(tag), ++i)
      v(i) = evalNumberAttributeRequired(e, "value");
  };
  const auto* dyn_elem = joint_elem->FirstChildElement("Dynamics");
  if (dyn_elem)
  {
    fill_dyn(joint.dynamics.stiffness, dyn_elem, "Stiffness");
    fill_dyn(joint.dynamics.damping, dyn_elem, "Damping");
    fill_dyn(joint.dynamics.inertia, dyn_elem, "Inertia");

    int i = 0;
    for (const auto* r = dyn_elem->FirstChildElement("RestPosition"); r && i < joint.dynamics.rest_position.size();
         r = r->NextSiblingElement("RestPosition"), ++i)
      joint.dynamics.rest_position[i] = evalNumberAttributeRequired(r, "value");
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

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem)
{
  using namespace components;
  ParallelGrasp grasp;

  const auto* gt = elem->FirstChildElement("GraspType");
  if (!gt)
    throw std::runtime_error("ParallelGrasp: <GraspType> is required.");
  const std::string gtv = evalTextAttributeRequired(gt, "value");
  if (gtv == "Internal")
    grasp.grasp_type = ParallelGrasp::GraspType::INTERNAL;
  else if (gtv == "External")
    grasp.grasp_type = ParallelGrasp::GraspType::EXTERNAL;
  else
    throw std::runtime_error("ParallelGrasp: <GraspType> value must be 'Internal' or 'External'");

  if (const auto* gap = elem->FirstChildElement("GapSize"))
    grasp.gap_size = evalNumberAttributeRequired(gap, "value");
  grasp.gap_axis = parseUnitVector(elem->FirstChildElement("GapAxis"));
  grasp.rotation_axis = parseUnitVector(elem->FirstChildElement("RotationalAxis"));

  if (const auto* sym = elem->FirstChildElement("RotationalSymmetry"))
    grasp.rotational_symmetry = evalUIntAttributeRequired(sym, "value");  // <- integer semantics
  else
    grasp.rotational_symmetry = 1;

  grasp.contact_shape_ids.clear();
  if (const auto* reals = elem->FirstChildElement("RealSurfaces"))
    for (const auto* surf = reals->FirstChildElement("Shape"); surf; surf = surf->NextSiblingElement("Shape"))
      grasp.contact_shape_ids.push_back(evalTextAttributeRequired(surf, "id"));

  if (const auto* vs = elem->FirstChildElement("VirtualSurface"))
  {
    const auto* shape = vs->FirstChildElement("Shape");
    if (!shape)
      throw std::runtime_error("<VirtualSurface> missing <Shape> at line " + std::to_string(vs->GetLineNum()));
    grasp.canonical_surface = parseShape(doc, shape);
  }
  return grasp;
}

}  // namespace xml
}  // namespace sodf
