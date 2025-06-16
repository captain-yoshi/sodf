#include <cmath>
#include <sodf/xml/entity_parser.h>
#include <sodf/xml/element_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>

#include <sodf/components/container.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/grasp.h>
#include <sodf/components/object.h>
#include <sodf/components/shape.h>
#include <sodf/components/transform.h>

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

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

  Shape shape = xml::parseShape(elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 0);
}

TEST(XMLParser, FluidDomaineShapeComponent)
{
  std::string xml_txt = R"(

    <root>


      <StackedShape id="stacked_shape">
        <Shape type="SphericalSegment">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
        </Shape>

        <Shape type="Cone">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
        </Shape>

        <Shape type="Cone">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
        </Shape>

        <Shape type="Cylinder">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions radius="0.00275" height="0.0040"/>
        </Shape>
      </StackedShape>


      <FluidDomainShape id="fluid/container">
        <StackedShape id="stacked_shape"/>
      </FluidDomainShape>)

    </root>)";

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
    if (tag == "StackedShape")
    {
      xml::parseStackedShapeComponent(child, db, eid);
    }
    else if (tag == "FluidDomainShape")
    {
      xml::parseFluidDomainShapeComponent(child, db, eid);
    }
  }

  auto geom_shape = sodf::getComponentElement<components::StackedShapeComponent>(db, eid, "stacked_shape");
  ASSERT_NE(geom_shape, nullptr);    // shape must exist
  ASSERT_EQ(geom_shape->size(), 4);  // 4 stacked shapes

  auto fluid_domain_shape = sodf::getComponentElement<components::DomainShapeComponent>(db, eid, "fluid/container");
  ASSERT_NE(fluid_domain_shape, nullptr);            // shape must exist
  ASSERT_EQ(fluid_domain_shape->domains.size(), 4);  // 4 stacked shapes
  ASSERT_EQ(fluid_domain_shape->stacked_shape_id, "stacked_shape");

  auto& shapes = fluid_domain_shape->domains;

  // Check first shape (SphericalSegment)
  {
    auto sph_seg = std::dynamic_pointer_cast<sodf::physics::FluidSphericalSegmentShape>(shapes.at(0));
    ASSERT_TRUE(bool(sph_seg));
    EXPECT_NEAR(sph_seg->base_radius_, 0.0, 1e-8);
    EXPECT_NEAR(sph_seg->top_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(sph_seg->getMaxFillHeight(), 0.00167, 1e-8);
  }

  // Check second shape (Cone)
  {
    auto cone1 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(shapes.at(1));
    ASSERT_TRUE(bool(cone1));
    EXPECT_NEAR(cone1->base_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(cone1->top_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(cone1->getMaxFillHeight(), 0.00765, 1e-8);
  }

  // Check third shape (Cone)
  {
    auto cone2 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(shapes.at(2));
    ASSERT_TRUE(bool(cone2));
    EXPECT_NEAR(cone2->base_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(cone2->top_radius_, 0.00275, 1e-8);
    EXPECT_NEAR(cone2->getMaxFillHeight(), 0.00478, 1e-8);
  }

  // Check fourth shape (Cylinder)
  {
    auto cyl = std::dynamic_pointer_cast<sodf::physics::FluidCylinderShape>(shapes.at(3));
    ASSERT_TRUE(bool(cyl));
    EXPECT_NEAR(cyl->radius_, 0.00275, 1e-8);
    EXPECT_NEAR(cyl->getMaxFillHeight(), 0.0040, 1e-8);
  }
}

TEST(XMLParser, ContainerComponent)
{
  std::string xml_txt = R"(
    <root>

      <StackedShape id="stacked_shape">
        <Shape type="SphericalSegment">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
        </Shape>

        <Shape type="Cone">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
        </Shape>

        <Shape type="Cone">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
        </Shape>

        <Shape type="Cylinder">
          <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
          <AxisReference x="1.0" y="0.0" z="0.0"/>
          <Dimensions radius="0.00275" height="0.0040"/>
        </Shape>
      </StackedShape>

      <FluidDomainShape id="fluid/container">
        <StackedShape id="stacked_shape"/>
      </FluidDomainShape>)

      <Container id="container/A1">
        <Transform parent="root">
          <Position x="0.08436" y="0.16794261249" z="0.0"/>
          <Orientation roll="0.0" pitch="pi/2" yaw="0.0"/>
        </Transform>
        <Content type="water" volume="0.0" units="uL"/>
        <AxisBottom x="1.0" y="0.0" z="0.0"/>
        <AxisReference x="0.0" y="1.0" z="0.0"/>
        <DomainShapeRef id="fluid/container"/>
        <Material id=""/>
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
    if (tag == "StackedShape")
    {
      xml::parseStackedShapeComponent(child, db, eid);
    }
    else if (tag == "FluidDomainShape")
    {
      xml::parseFluidDomainShapeComponent(child, db, eid);
    }
    else if (tag == "Container")
    {
      xml::parseContainerComponent(child, db, eid);
    }
  }

  auto container = sodf::getComponentElement<components::ContainerComponent>(db, eid, "container/A1");
  ASSERT_NE(container, nullptr);  // shape must exist
  EXPECT_EQ(container->content_type, "water");
  EXPECT_NEAR(container->axis_bottom.x(), 1.0, 1e-8);
  EXPECT_NEAR(container->axis_bottom.y(), 0.0, 1e-8);
  EXPECT_NEAR(container->axis_bottom.z(), 0.0, 1e-8);
  EXPECT_EQ(container->domain_shape_id, "fluid/container");

  auto domain_shape = sodf::getComponentElement<components::DomainShapeComponent>(db, eid, container->domain_shape_id);
  ASSERT_NE(domain_shape, nullptr);
  ASSERT_EQ(domain_shape->domains.size(), 4);
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

    xml::EntityParser parser;
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
            <AxisRotationalFirstShape x="0.0" y="0.0" z="-1.0"/>
            <AxisNormalFirstShape x="-1.0" y="0.0" z="0.0"/>
            <AxisRotationalGrasp x="1.0" y="0.0" z="0.0"/>
            <Shape id="area/front/handle"/>
            <Shape id="area/back/handle"/>
            <Shape id="area/left/handle"/>
            <Shape id="area/right/handle"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    xml::EntityParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    auto grasp = sodf::getComponentElement<components::ParallelGraspComponent>(db, eid, "grasp/handle");
    auto tf_node = sodf::getComponentElement<components::TransformComponent>(db, eid, "grasp/handle");
    ASSERT_NE(grasp, nullptr);
    ASSERT_NE(tf_node, nullptr);

    ASSERT_EQ(0.02, grasp->gap_size);
    ASSERT_EQ(ParallelGrasp::ApproachType::INTERNAL, grasp->approach);
    ASSERT_EQ(4, grasp->rotational_symmetry);
    ASSERT_EQ("area/front/handle", grasp->contact_shape_ids[0]);
    ASSERT_EQ("area/back/handle", grasp->contact_shape_ids[1]);

    // ASSERT_TRUE((tf_ptr->local.linear() * grasp.axis_of_rotation).isApprox(Eigen::Vector3d(1, 0, 0)));
    ASSERT_TRUE(grasp->axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp->canonical_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.01, vshape.dimensions.at(0));
    ASSERT_EQ(0.05, vshape.dimensions.at(1));
    ASSERT_TRUE(vshape.vertices.empty());
    ASSERT_EQ(3, vshape.axes.size());
    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(0, 0, -1)));
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, -1, 0)));
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
            <AxisRotationalFirstShape x="0.0" y="0.0" z="-1.0"/>
            <AxisNormalFirstShape x="1.0" y="0.0" z="0.0"/>
            <AxisRotationalGrasp x="1.0" y="0.0" z="0.0"/>
            <Shape id="box"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    xml::EntityParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    auto grasp = sodf::getComponentElement<components::ParallelGraspComponent>(db, eid, "grasp");
    auto tf_node = sodf::getComponentElement<components::TransformComponent>(db, eid, "grasp");
    ASSERT_NE(grasp, nullptr);
    ASSERT_NE(tf_node, nullptr);

    ASSERT_EQ(0.02, grasp->gap_size);
    ASSERT_EQ(ParallelGrasp::ApproachType::INTERNAL, grasp->approach);
    ASSERT_EQ(4, grasp->rotational_symmetry);
    ASSERT_EQ(1, grasp->contact_shape_ids.size());
    ASSERT_EQ("box", grasp->contact_shape_ids[0]);
    ASSERT_TRUE(grasp->axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp->canonical_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.02, vshape.dimensions.at(0));
    ASSERT_EQ(0.05, vshape.dimensions.at(1));
    ASSERT_EQ(3, vshape.axes.size());
    // ASSERT_TRUE(vshape.vertices.empty());
    std::cout << grasp << std::endl;

    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(0, 0, 1)));
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 1, 0)));
    ASSERT_TRUE(vshape.axes[2].isApprox(Eigen::Vector3d(-1, 0, 0)));
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
            <AxisRotationalFirstShape x="0.0" y="0.0" z="1.0"/>
            <AxisNormalFirstShape x="1.0" y="0.0" z="0.0"/>
            <AxisRotationalGrasp x="1.0" y="0.0" z="0.0"/>
            <Shape id="cylinder"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </root>
    )";

    xml::EntityParser parser;
    auto db = ginseng::database{};
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ginseng::database::ent_id> id_map;
    db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    auto grasp = sodf::getComponentElement<components::ParallelGraspComponent>(db, eid, "grasp");
    auto tf_node = sodf::getComponentElement<components::TransformComponent>(db, eid, "grasp");
    ASSERT_NE(grasp, nullptr);
    ASSERT_NE(tf_node, nullptr);

    ASSERT_EQ(0.024, grasp->gap_size);
    ASSERT_EQ(ParallelGrasp::ApproachType::INTERNAL, grasp->approach);
    ASSERT_EQ(0, grasp->rotational_symmetry);
    ASSERT_EQ(1, grasp->contact_shape_ids.size());
    ASSERT_EQ("cylinder", grasp->contact_shape_ids[0]);
    ASSERT_TRUE(grasp->axis_of_rotation.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp->canonical_surface;
    ASSERT_EQ(ShapeType::Line, vshape.type);
    ASSERT_EQ(1, vshape.dimensions.size());
    ASSERT_EQ(0.02, vshape.dimensions.at(0));

    std::cout << grasp << std::endl;

    ASSERT_EQ(2, vshape.axes.size());
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 0, 1)));
    ASSERT_TRUE(vshape.mesh_path.empty());
    ASSERT_EQ(2, vshape.vertices.size());
    ASSERT_TRUE(vshape.vertices[0].isApprox(Eigen::Vector3d(-0.01, 0, 0)));
    ASSERT_TRUE(vshape.vertices[1].isApprox(Eigen::Vector3d(0.01, 0, 0)));
  }
}

TEST(XMLParser, ExpandForLoop)
{
  {
    // Simulate TinyXML2 element creation
    tinyxml2::XMLDocument doc;
    auto* forElem = doc.NewElement("ForLoop");
    forElem->SetAttribute("row_name", "A:C:1");
    forElem->SetAttribute("row", "1:3:1");
    forElem->SetAttribute("col", "0:2:1");
    forElem->SetAttribute("zipped", "[row_name,row]");

    // Use ForLoop expansion to print all variable combinations
    std::ostringstream oss;
    xml::expandForLoop(forElem, [&](const std::unordered_map<std::string, std::string>& ctx) {
      oss << "row_name=" << ctx.at("row_name") << ", row=" << ctx.at("row") << ", col=" << ctx.at("col") << "\n";
    });

    std::string expected = "row_name=A, row=1, col=0\n"
                           "row_name=A, row=1, col=1\n"
                           "row_name=A, row=1, col=2\n"
                           "row_name=B, row=2, col=0\n"
                           "row_name=B, row=2, col=1\n"
                           "row_name=B, row=2, col=2\n"
                           "row_name=C, row=3, col=0\n"
                           "row_name=C, row=3, col=1\n"
                           "row_name=C, row=3, col=2\n";
    ASSERT_EQ(oss.str(), expected);
  }

  {
    // Simulate TinyXML2 element creation
    tinyxml2::XMLDocument doc;
    auto* forElem = doc.NewElement("ForLoop");
    forElem->SetAttribute("row_name", "a:e:2");
    forElem->SetAttribute("row", "1:3:1");
    forElem->SetAttribute("col", "0:1:1");
    // No zipped attribute!

    std::ostringstream oss;
    xml::expandForLoop(forElem, [&](const std::unordered_map<std::string, std::string>& ctx) {
      oss << "row_name=" << ctx.at("row_name") << ", row=" << ctx.at("row") << ", col=" << ctx.at("col") << "\n";
    });

    // No zipped: Cartesian product (col varies fastest)
    std::string expected = "row_name=a, row=1, col=0\n"
                           "row_name=a, row=1, col=1\n"
                           "row_name=a, row=2, col=0\n"
                           "row_name=a, row=2, col=1\n"
                           "row_name=a, row=3, col=0\n"
                           "row_name=a, row=3, col=1\n"
                           "row_name=c, row=1, col=0\n"
                           "row_name=c, row=1, col=1\n"
                           "row_name=c, row=2, col=0\n"
                           "row_name=c, row=2, col=1\n"
                           "row_name=c, row=3, col=0\n"
                           "row_name=c, row=3, col=1\n"
                           "row_name=e, row=1, col=0\n"
                           "row_name=e, row=1, col=1\n"
                           "row_name=e, row=2, col=0\n"
                           "row_name=e, row=2, col=1\n"
                           "row_name=e, row=3, col=0\n"
                           "row_name=e, row=3, col=1\n";
    ASSERT_EQ(oss.str(), expected);
  }

  {
    tinyxml2::XMLDocument doc;
    auto* forElem = doc.NewElement("ForLoop");
    forElem->SetAttribute("row_name", "a:e:2:aa:ba:2:A:E:2:AA:BA:2");

    std::ostringstream oss;
    xml::expandForLoop(forElem, [&](const std::unordered_map<std::string, std::string>& ctx) {
      oss << "row_name=" << ctx.at("row_name") << "\n";
    });

    std::string expected =
        // a:e:2
        "row_name=a\n"
        "row_name=c\n"
        "row_name=e\n"
        // aa:ba:2
        "row_name=aa\n"
        "row_name=ac\n"
        "row_name=ae\n"
        "row_name=ag\n"
        "row_name=ai\n"
        "row_name=ak\n"
        "row_name=am\n"
        "row_name=ao\n"
        "row_name=aq\n"
        "row_name=as\n"
        "row_name=au\n"
        "row_name=aw\n"
        "row_name=ay\n"
        "row_name=ba\n"
        // A:E:2
        "row_name=A\n"
        "row_name=C\n"
        "row_name=E\n"
        // AA:BA:2
        "row_name=AA\n"
        "row_name=AC\n"
        "row_name=AE\n"
        "row_name=AG\n"
        "row_name=AI\n"
        "row_name=AK\n"
        "row_name=AM\n"
        "row_name=AO\n"
        "row_name=AQ\n"
        "row_name=AS\n"
        "row_name=AU\n"
        "row_name=AW\n"
        "row_name=AY\n"
        "row_name=BA\n";
    ASSERT_EQ(oss.str(), expected);
  }
}

TEST(XMLParser, ComponentsForLoop)
{
  // Example: Create a 2x3 (for simplicity in test), but you can do A:H and 1:12 for 96 wells
  std::string xml_txt = R"(
    <root>
      <Object id="plate">

        <StackedShape id="stacked_shape">
          <Shape type="SphericalSegment">
            <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
            <AxisReference x="1.0" y="0.0" z="0.0"/>
            <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
          </Shape>

          <Shape type="Cone">
            <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
            <AxisReference x="1.0" y="0.0" z="0.0"/>
            <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
          </Shape>

          <Shape type="Cone">
            <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
            <AxisReference x="1.0" y="0.0" z="0.0"/>
            <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
          </Shape>

          <Shape type="Cylinder">
            <AxisSymmetry x="0.0" y="0.0" z="1.0"/>
            <AxisReference x="1.0" y="0.0" z="0.0"/>
            <Dimensions radius="0.00275" height="0.0040"/>
          </Shape>
        </StackedShape>

        <FluidDomainShape id="fluid/container">
          <StackedShape id="stacked_shape"/>
        </FluidDomainShape>


        <ForLoop row_name="A:B:1" row="1:2:1" col="1:3:1" zipped="[row_name,row]">
          <Container id="container/{row_name}{col}">
            <Transform parent="root">
              <Position x="{col}.1" y="{row}.2" z="0.0008"/>
              <Orientation roll="0.0" pitch="1.57079632679" yaw="0.0"/>
            </Transform>
            <Material type="polypropylene"/>
            <Content type="empty" volume="0.0" units="uL"/>
            <AxisBottom x="1.0" y="0.0" z="0.0"/>
            <AxisReference x="0.0" y="1.0" z="0.0"/>
            <DomainShapeRef id="fluid/well_250ul"/>
            <LiquidLevelJointRef id="container/{row_name}{col}/liquid_level"/>
          </Container>
        </ForLoop>
      </Object>
    </root>
    )";

  xml::EntityParser parser;
  auto db = ginseng::database{};
  parser.loadEntitiesFromText(xml_txt, db);

  // Find the object entity
  std::unordered_map<std::string, ginseng::database::ent_id> id_map;
  db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
    id_map.insert({ object.id, id });
  });

  ASSERT_TRUE(!id_map.empty());
  ASSERT_EQ(1u, id_map.size());

  const auto& eid = id_map.begin()->second;

  std::vector<std::string> expected_ids = { "container/A1", "container/A2", "container/A3",
                                            "container/B1", "container/B2", "container/B3" };

  for (const auto& cid : expected_ids)
  {
    auto* cont = sodf::getComponentElement<components::ContainerComponent>(db, eid, cid);
    ASSERT_NE(cont, nullptr) << "Missing container: " << cid;
    ASSERT_EQ(cont->volume, 0.0);
    ASSERT_EQ(cont->domain_shape_id, "fluid/well_250ul");
    ASSERT_EQ(cont->liquid_level_joint_id, cid + "/liquid_level");
  }
}
