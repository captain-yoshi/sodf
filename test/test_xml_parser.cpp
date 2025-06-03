#include <cmath>
#include <sodf/xml_parser.h>

#include <sodf/components/domain_shape.h>
#include <sodf/components/container.h>
#include <sodf/components/shape.h>
#include <sodf/components/grasp.h>
#include <sodf/components/object.h>

#include <gtest/gtest.h>

using namespace sodf;
using namespace sodf::components;
using namespace sodf::geometry;

TEST(XMLParser, ParseRectangleShape)
{
  std::string xml_txt = R"(
    <Shape type="Rectangle">
      <AxisNormal x="1.0" y="0.0" z="0.0"/>
      <AxisWidth x="0.0" y="1.0" z="0.0"/>
      <AxisHeight x="0.0" y="0.0" z="1.0"/>
      <Dimensions width="0.05" height="0.01"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Rectangle);
  EXPECT_DOUBLE_EQ(shape.dimensions[0], 0.05);
  EXPECT_DOUBLE_EQ(shape.dimensions[1], 0.01);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisNormal
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisWidth
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisHeight
}

TEST(XMLParser, ParseCircleShape)
{
  std::string xml_txt = R"(
    <Shape type="Circle">
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
      <AxisMajor x="1.0" y="0.0" z="0.0"/>
      <Dimensions radius="0.025"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Circle);
  EXPECT_DOUBLE_EQ(shape.dimensions[0], 0.025);
  ASSERT_EQ(shape.axes.size(), 2);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisMajor
}

TEST(XMLParser, ParseTriangleShape)
{
  std::string xml_txt = R"(
    <Shape type="Triangle">
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
      <AxisX x="1.0" y="0.0" z="0.0"/>
      <AxisY x="0.0" y="1.0" z="0.0"/>
      <Vertices>
        <Vertex x="0.0" y="0.0"/>
        <Vertex x="0.04" y="0.0"/>
        <Vertex x="0.02" y="0.03"/>
      </Vertices>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Triangle);
  ASSERT_EQ(shape.vertices.size(), 3);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisX
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisY
}

TEST(XMLParser, ParsePolygonShape)
{
  std::string xml_txt = R"(
    <Shape type="Polygon">
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
      <AxisX x="1.0" y="0.0" z="0.0"/>
      <AxisY x="0.0" y="1.0" z="0.0"/>
      <Vertices>
        <Vertex x="0.0" y="0.0"/>
        <Vertex x="0.04" y="0.0"/>
        <Vertex x="0.03" y="0.02"/>
        <Vertex x="0.01" y="0.03"/>
      </Vertices>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Polygon);
  ASSERT_EQ(shape.vertices.size(), 4);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisX
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisY
}

TEST(XMLParser, ParseBoxShape)
{
  std::string xml_txt = R"(
    <Shape type="Box">
      <AxisWidth x="1.0" y="0.0" z="0.0"/>
      <AxisDepth x="0.0" y="1.0" z="0.0"/>
      <AxisHeight x="0.0" y="0.0" z="1.0"/>
      <Dimensions width="0.05" height="0.01" depth="0.02"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Box);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisY
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisZ
}

TEST(XMLParser, ParseCylinderShape)
{
  std::string xml_txt = R"(
    <Shape type="Cylinder">
      <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
      <AxisReference x="1.0" y="0.0" z="0.0"/>
      <Dimensions radius="0.012" height="0.20"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Cylinder);
  ASSERT_EQ(shape.axes.size(), 2);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisSymmetry
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // ReferenceAxis
}

TEST(XMLParser, ParseSphereShape)
{
  std::string xml_txt = R"(
    <Shape type="Sphere">
      <Dimensions radius="0.015"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Sphere);
  // Spheres often have no axis, but you could assert shape.axes.size() == 0
}

TEST(XMLParser, ParseMeshShape)
{
  std::string xml_txt = R"(
    <Shape type="Mesh">
      <File path="package://some/path/mesh.stl"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Mesh);
}

TEST(XMLParser, ParsePlaneShape)
{
  std::string xml_txt = R"(
    <Shape type="Plane">
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
      <AxisX x="1.0" y="0.0" z="0.0"/>
      <AxisY x="0.0" y="1.0" z="0.0"/>
      <Dimensions width="0.1" height="0.05"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Plane);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisX
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisY
}

TEST(XMLParser, ParseConeShape)
{
  std::string xml_txt = R"(
    <Shape type="Cone">
      <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
      <AxisReference x="1.0" y="0.0" z="0.0"/>
      <Dimensions base_radius="0.01" top_radius="0.002" height="0.025"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Cone);
  ASSERT_EQ(shape.axes.size(), 2);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisSymmetry
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(1.0, 0.0, 0.0));  // ReferenceAxis
}

TEST(XMLParser, ParseLineShapeAnchor)
{
  std::string xml_txt = R"(
    <Shape type="Line" anchor="center">
      <AxisDirection x="0.0" y="1.0" z="0.0"/>
      <Length value="0.15"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 1);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisDirection
}

TEST(XMLParser, ParseLine2DShape)
{
  std::string xml_txt = R"(
    <Shape type="Line">
      <Vertex x="1.0" y="4.0" />
      <Vertex x="1.0" y="2.0" />
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 0);
}

TEST(XMLParser, ParseLine3DShape)
{
  std::string xml_txt = R"(
    <Shape type="Line">
      <Vertex x="1" y="2.0" z="3.0"/>
      <Vertex x="6.0" y="3.0" z="4.0"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = sodf::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 0);
}

TEST(XMLParser, FluidDomaineShapeComponent)
{
  std::string xml_txt = R"(
    <FluidDomainShape id="fluid/container">
      <StackedShapes>
        <Shape type="SphericalSegment" base_radius="0.0" top_radius="0.00142" height="0.00167"/>
        <Shape type="Cone" base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
        <Shape type="Cone" base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
        <Shape type="Cylinder" radius="0.00275" height="0.0040"/>
      </StackedShapes>
    </FluidDomainShape>)";

  auto db = ginseng::database{};
  auto eid = db.create_entity();

  // 1. Parse string as XML document
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);

  // 2. Get root element (here, it's "FluidDomainShape")
  const tinyxml2::XMLElement* shape_elem = doc.RootElement();
  ASSERT_TRUE(shape_elem);

  // Parse the XML (replace this with your actual parser)
  sodf::parseFluidDomainShapeComponent(shape_elem, db, eid);

  const auto* domain_component = db.get_component<sodf::components::DomainShapeComponent*>(eid);
  ASSERT_NE(domain_component, nullptr);  // component must exist

  // Check that the shape was registered
  auto shapes = sodf::find_in_flat_map(domain_component->domain_shape_map, "fluid/container");
  ASSERT_NE(shapes, nullptr);    // shape must exist
  ASSERT_EQ(shapes->size(), 4);  // 4 stacked shapes

  // Check first shape (SphericalSegment)
  {
    auto sph_seg = std::dynamic_pointer_cast<sodf::physics::FluidSphericalSegmentShape>(shapes->at(0));
    ASSERT_TRUE(bool(sph_seg));
    EXPECT_NEAR(sph_seg->base_radius_, 0.0, 1e-8);
    EXPECT_NEAR(sph_seg->top_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(sph_seg->getMaxFillHeight(), 0.00167, 1e-8);
  }

  // Check second shape (Cone)
  {
    auto cone1 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(shapes->at(1));
    ASSERT_TRUE(bool(cone1));
    EXPECT_NEAR(cone1->base_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(cone1->top_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(cone1->getMaxFillHeight(), 0.00765, 1e-8);
  }

  // Check third shape (Cone)
  {
    auto cone2 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(shapes->at(2));
    ASSERT_TRUE(bool(cone2));
    EXPECT_NEAR(cone2->base_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(cone2->top_radius_, 0.00275, 1e-8);
    EXPECT_NEAR(cone2->getMaxFillHeight(), 0.00478, 1e-8);
  }

  // Check fourth shape (Cylinder)
  {
    auto cyl = std::dynamic_pointer_cast<sodf::physics::FluidCylinderShape>(shapes->at(3));
    ASSERT_TRUE(bool(cyl));
    EXPECT_NEAR(cyl->radius_, 0.00275, 1e-8);
    EXPECT_NEAR(cyl->getMaxFillHeight(), 0.0040, 1e-8);
  }
}

TEST(XMLParser, ContainerComponent)
{
  std::string xml_txt = R"(
    <root>
      <FluidDomainShape id="fluid/container">
        <StackedShapes>
          <Shape type="SphericalSegment" base_radius="0.0" top_radius="0.00142" height="0.00167"/>
          <Shape type="Cone" base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
          <Shape type="Cone" base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
          <Shape type="Cylinder" radius="0.00275" height="0.0040"/>
        </StackedShapes>
      </FluidDomainShape>
      <Container id="container/A1">
        <Transform parent="root">
          <Position x="0.08436" y="0.16794261249" z="0.0"/>
          <Orientation roll="0.0" pitch="pi/2" yaw="0.0"/>
        </Transform>
        <Material type="polypropylene" />
        <Content type="water" volume="0.0" units="uL"/>
        <AxisBottom x="1.0" y="0.0" z="0.0"/>
        <DomainShapeRef id="fluid/container"/>
      </Container>
    </root>
  )";

  auto db = ginseng::database{};
  auto eid = db.create_entity();

  // 1. Parse string as XML document
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);

  // 2. Get root element (here, it's "Root")
  const tinyxml2::XMLElement* root = doc.RootElement();
  ASSERT_TRUE(root);

  // 3. Parse all FluidDomainShapes and Containers (order doesn't matter)
  for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    std::string tag = child->Name();
    if (tag == "FluidDomainShape")
    {
      sodf::parseFluidDomainShapeComponent(child, db, eid);
    }
    else if (tag == "Container")
    {
      sodf::parseContainerComponent(child, db, eid);
    }
  }

  // 4. Check FluidDomainShapeComponent as before (use your own asserts above)

  // 5. Now check the Container component
  const auto* container_comp = db.get_component<sodf::components::ContainerComponent*>(eid);
  ASSERT_NE(container_comp, nullptr);

  auto container_it = sodf::find_in_flat_map(container_comp->container_map, std::string("container/A1"));
  ASSERT_NE(container_it, nullptr);

  const auto& container = *container_it;
  EXPECT_EQ(container.content_type, "water");
  EXPECT_EQ(container.material_type, "polypropylene");
  EXPECT_NEAR(container.axis_bottom.x(), 1.0, 1e-8);
  EXPECT_NEAR(container.axis_bottom.y(), 0.0, 1e-8);
  EXPECT_NEAR(container.axis_bottom.z(), 0.0, 1e-8);
  EXPECT_EQ(container.domain_shape_ref, "fluid/container");

  // Check that the referenced domain shape exists
  const auto* domain_component = db.get_component<sodf::components::DomainShapeComponent*>(eid);
  ASSERT_NE(domain_component, nullptr);
  auto shapes = sodf::find_in_flat_map(domain_component->domain_shape_map, container.domain_shape_ref);
  ASSERT_NE(shapes, nullptr);
  ASSERT_EQ(shapes->size(), 4);
}

TEST(XMLParser, ParallelGraspDerivedFrom)
{
  // No shapes defined
  {
    std::string xml_txt = R"(
    <root>
      <Object id="test">

        <ParallelGrasp id="grasp/handle">
          <DerivedFromParallelShapes>
            <AxisSymetrie x="0.0" y="0.0" z="-1.0"/>
            <AxisNormal x="-1.0" y="0.0" z="0.0"/>
            <Shape id="area/front/handle"/>
            <Shape id="area/back/handle"/>
            <Shape id="area/left/handle"/>
            <Shape id="area/rigth/handle"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

     </Object>
   </root>
  )";

    XMLParser parser;
    auto db = ginseng::database{};
    ASSERT_ANY_THROW(parser.loadEntitiesFromText(xml_txt, db));
  }

  // No shapes defined
  {
    std::string xml_txt = R"(
    <root>
      <Object id="test">

        <!-- FRONT SURFACE (normal +X) -->
        <Shape id="area/front/handle" type="Rectangle">
          <Transform parent="root">
            <Position x="-0.01" y="-0.3" z="0.7"/> <!-- +0.01 along X -->
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
          <AxisNormal x="1.0" y="0.0" z="0.0"/>
          <AxisWidth  x="0.0" y="1.0" z="0.0"/>
          <AxisHeight x="0.0" y="0.0" z="1.0"/>
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- BACK SURFACE (normal -X) -->
        <Shape id="area/back/handle" type="Rectangle">
          <Transform parent="root">
            <Position x="0.01" y="-0.3" z="0.7"/> <!-- -0.01 along X -->
            <Orientation roll="0.0" pitch="0.0" yaw="pi"/>
          </Transform>
          <AxisNormal x="1.0" y="0.0" z="0.0"/>
          <AxisWidth  x="0.0" y="1.0" z="0.0"/>
          <AxisHeight x="0.0" y="0.0" z="1.0"/>
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- LEFT SURFACE (normal -Y) -->
        <Shape id="area/left/handle" type="Rectangle">
          <Transform parent="root">
            <Position x="0.0" y="-0.29" z="0.7"/> <!-- -0.01 along Y -->
            <Orientation roll="0.0" pitch="0.0" yaw="-pi/2"/>
          </Transform>
          <AxisNormal x="1.0" y="0.0" z="0.0"/>
          <AxisWidth  x="0.0" y="1.0" z="0.0"/>
          <AxisHeight x="0.0" y="0.0" z="1.0"/>
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- RIGHT SURFACE (normal +Y) -->
        <Shape id="area/right/handle" type="Rectangle">
          <Transform parent="root">
            <Position x="0.0" y="-0.31" z="0.7"/> <!-- +0.01 along Y -->
            <Orientation roll="0.0" pitch="0.0" yaw="+pi/2"/>
          </Transform>
          <AxisNormal x="1.0" y="0.0" z="0.0"/>
          <AxisWidth  x="0.0" y="1.0" z="0.0"/>
          <AxisHeight x="0.0" y="0.0" z="1.0"/>
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <ParallelGrasp id="grasp/handle">
          <DerivedFromParallelShapes>
            <AxisRotational x="0.0" y="0.0" z="-1.0"/>
            <AxisNormal x="-1.0" y="0.0" z="0.0"/>
            <Shape id="area/front/handle"/>
            <Shape id="area/back/handle"/>
            <Shape id="area/left/handle"/>
            <Shape id="area/right/handle"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    XMLParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    ASSERT_TRUE(db.has_component<components::ParallelGraspComponent>(eid));
    auto& component = db.get_component<components::ParallelGraspComponent>(eid);

    const auto* grasp_ptr = find_in_flat_map(component.grasp_map, "grasp/handle");
    ASSERT_TRUE(grasp_ptr);
    const auto& grasp = *grasp_ptr;

    ASSERT_EQ(0.02, grasp.gap_size);
    ASSERT_EQ(ParallelGraspApproach::INTERNAL, grasp.approach);
    ASSERT_EQ(4, grasp.rotational_symmetry);
    ASSERT_EQ("area/front/handle", grasp.real_surfaces[0]);
    ASSERT_EQ("area/back/handle", grasp.real_surfaces[1]);
    ASSERT_TRUE(grasp.axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp.virtual_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.01, vshape.dimensions.at(0));
    ASSERT_EQ(0.05, vshape.dimensions.at(1));
    ASSERT_TRUE(vshape.vertices.empty());
    ASSERT_EQ(3, vshape.axes.size());
    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(0, -1, 0)));
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 0, -1)));
    ASSERT_TRUE(vshape.axes[2].isApprox(Eigen::Vector3d(-1, 0, 0)));
  }

  {
    std::string xml_txt = R"(
    <root>
      <Object id="test">

        <Shape id="box" type="Box">
          <Transform parent="root">
            <Position x="0.0" y="-0.3" z="0.1"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
          <AxisWidth x="1.0" y="0.0" z="0.0"/>
          <AxisDepth x="0.0" y="1.0" z="0.0"/>
          <AxisHeight x="0.0" y="0.0" z="1.0"/>
          <Dimensions width="0.02" depth="0.02" height="0.05"/>
        </Shape>

        <ParallelGrasp id="grasp">
          <DerivedFromParallelShapes>
            <Approach value="Internal"/>
            <AxisRotational x="0.0" y="0.0" z="-1.0"/>
            <AxisNormal x="1.0" y="0.0" z="0.0"/>
            <Shape id="box"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    XMLParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    ASSERT_TRUE(db.has_component<components::ParallelGraspComponent>(eid));
    auto& component = db.get_component<components::ParallelGraspComponent>(eid);

    const auto* grasp_ptr = find_in_flat_map(component.grasp_map, "grasp");
    ASSERT_TRUE(grasp_ptr);
    const auto& grasp = *grasp_ptr;

    ASSERT_EQ(0.02, grasp.gap_size);
    ASSERT_EQ(ParallelGraspApproach::INTERNAL, grasp.approach);
    ASSERT_EQ(4, grasp.rotational_symmetry);
    ASSERT_EQ("box", grasp.real_surfaces[0]);
    ASSERT_EQ("", grasp.real_surfaces[1]);
    ASSERT_TRUE(grasp.axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp.virtual_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.02, vshape.dimensions.at(0));
    ASSERT_EQ(0.05, vshape.dimensions.at(1));
    ASSERT_EQ(3, vshape.axes.size());
    // ASSERT_TRUE(vshape.vertices.empty());
    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 1, 0)));
    ASSERT_TRUE(vshape.axes[2].isApprox(Eigen::Vector3d(0, 0, 1)));
  }

  {
    std::string xml_txt = R"(
    <root>
      <Object id="test">

        <Shape id="cylinder" type="Cylinder">
          <Transform parent="root">
            <Position x="0.0" y="0.0" z="1.0"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>

          <Dimensions radius="0.012" height="0.020"/>
        </Shape>

        <ParallelGrasp id="grasp">
          <DerivedFromParallelShapes>
            <Approach value="Internal"/>
            <AxisRotational x="0.0" y="0.0" z="1.0"/>
            <AxisNormal x="1.0" y="0.0" z="0.0"/>
            <Shape id="cylinder"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    XMLParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    ASSERT_TRUE(db.has_component<components::ParallelGraspComponent>(eid));
    auto& component = db.get_component<components::ParallelGraspComponent>(eid);

    const auto* grasp_ptr = find_in_flat_map(component.grasp_map, "grasp");
    ASSERT_TRUE(grasp_ptr);
    const auto& grasp = *grasp_ptr;

    ASSERT_EQ(0.024, grasp.gap_size);
    ASSERT_EQ(ParallelGraspApproach::INTERNAL, grasp.approach);
    ASSERT_EQ(0, grasp.rotational_symmetry);
    ASSERT_EQ("cylinder", grasp.real_surfaces[0]);
    ASSERT_EQ("", grasp.real_surfaces[1]);
    ASSERT_TRUE(grasp.axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp.virtual_surface;
    ASSERT_EQ(ShapeType::Line, vshape.type);
    ASSERT_EQ(1, vshape.dimensions.size());
    ASSERT_EQ(0.02, vshape.dimensions.at(0));
    ASSERT_EQ(1, vshape.axes.size());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));
    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_EQ(2, vshape.vertices.size());
    ASSERT_TRUE(vshape.vertices[0].isApprox(Eigen::Vector3d(-0.01, 0, 0)));
    ASSERT_TRUE(vshape.vertices[1].isApprox(Eigen::Vector3d(0.01, 0, 0)));
  }
}
