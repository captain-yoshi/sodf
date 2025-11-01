#include <cmath>
#include <filesystem>
#include <fstream>

#include <sodf/xml/entity_parser.h>
#include <sodf/xml/element_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>

#include <sodf/components/container.h>
#include <sodf/components/insertion.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/grasp.h>
#include <sodf/components/object.h>
#include <sodf/components/shape.h>
#include <sodf/components/transform.h>

#include <sodf/physics/fluid_domain_shape.h>

#include <gtest/gtest.h>

using namespace sodf;
using namespace sodf::components;
using namespace sodf::geometry;
namespace fs = std::filesystem;

static std::string writeTempFile(const std::string& filename, const std::string& contents)
{
  fs::path dir = fs::temp_directory_path() / "sodf_min_xml_tests";
  fs::create_directories(dir);
  fs::path p = dir / filename;
  std::ofstream(p.string()) << contents;
  return p.string();
}

static std::vector<std::string> collectObjectIds(ecs::Database& db)
{
  std::vector<std::string> ids;
  db.each([&](ecs::Database::entity_type id, ObjectComponent& obj) { ids.push_back(obj.id); });
  return ids;
}

TEST(XMLParser, IncludeMissingFile)
{
  const std::string scene_xml = R"(
    <Root>
      <Include uri="__does_not_exist__.xml"/>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_include_missing.xml", scene_xml);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, ImportMissingPath)
{
  const std::string scene_xml = R"(
    <Root>
      <Import/>
    </Root>)";
  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromText(scene_xml, db, /*base_dir=*/""));
}

TEST(XMLParser, DuplicateObjectIdAcrossIncludes)
{
  const std::string libA = R"(<Root><Object id="base"/></Root>)";
  const std::string libB = R"(<Root><Object id="base"/></Root>)";
  writeTempFile("dup_A.xml", libA);
  writeTempFile("dup_B.xml", libB);

  const std::string scene = R"(
    <Root>
      <Include uri="dup_A.xml"/>
      <Include uri="dup_B.xml"/>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_dup.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, OverlaySlotCollision)
{
  const std::string lib = R"(<?xml version="1.0"?>
    <Root>
      <Object id="device"/>
      <Overlay id="v1" slot="hmi" target="device"/>
      <Overlay id="v2" slot="hmi" target="device"/>
    </Root>)";
  writeTempFile("overlay_collision.xml", lib);

  const std::string scene = R"(<?xml version="1.0"?>
    <Root>
      <Include uri="overlay_collision.xml"/>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_overlay_collision.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, OverlayExactDuplicate)
{
  const std::string objDoc = R"(
    <Root>
      <Object id="device_dup"/>
    </Root>)";
  const std::string ovA = R"(
    <Root>
      <Overlay id="fw" slot="hmi" target="device_dup"/>
    </Root>)";
  const std::string ovB = R"(
    <Root>
      <Overlay id="fw" slot="hmi" target="device_dup"/>
    </Root>)";

  writeTempFile("device_dup.xml", objDoc);
  writeTempFile("ovA.xml", ovA);
  writeTempFile("ovB.xml", ovB);

  const std::string scene = R"(
    <Root>
      <Include uri="device_dup.xml"/>
      <Include uri="ovA.xml"/>
      <Include uri="ovB.xml"/>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_overlay_dup.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, CloneSourceNotFound)
{
  const std::string scene = R"(
<Root>
  <Object id="x" model="missing_model"/>
</Root>)";
  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromText(scene, db, /*base_dir=*/""));
}

TEST(XMLParser, ImportWithNS_UnqualifiedModelRef)
{
  const std::string lib = R"(
    <Root>
      <Object id="base"/>
    </Root>)";
  writeTempFile("lib_ns.xml", lib);

  const std::string scene = R"(
    <Root>
      <Import uri="lib_ns.xml" ns="foo"/>
      <Object id="inst" model="base"/> <!-- should be model="foo:base" -->
    </Root>)";
  const std::string scene_path = writeTempFile("scene_import_ns_bad.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, InvalidChildInClone)
{
  const std::string lib = R"(
    <Root>
      <Object id="base"></Object>
    </Root>)";
  writeTempFile("lib_bad_child.xml", lib);

  const std::string scene = R"(
    <Root>
      <Include uri="lib_bad_child.xml"/>
      <Object id="inst" model="base">
        <Origin id="extra"/> <!-- illegal under model="..." -->
      </Object>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_bad_child.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, OverlayRemoveMissingComponent)
{
  const std::string lib = R"(
    <Root>
      <Object id="base_remove"/>
      <Overlay id="ov1" slot="hmi" target="base_remove">
        <Remove>
          <Link id="L1"/>
        </Remove>
      </Overlay>
    </Root>)";
  writeTempFile("lib_remove_missing.xml", lib);

  const std::string scene = R"(
    <Root>
      <Include uri="lib_remove_missing.xml"/>
      <Object id="inst" model="base_remove">
        <Overlay slot="hmi" id="ov1"/>
      </Object>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_remove_missing.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, ContractModelUnsatisfied)
{
  const std::string lib = R"(
    <Root>
      <Object id="device_contract">
        <Overlay slot="hmi"/> <!-- required by default -->
      </Object>
    </Root>)";
  writeTempFile("lib_contract.xml", lib);

  const std::string scene = R"(
    <Root>
      <Include uri="lib_contract.xml"/>
      <Object id="inst" model="device_contract"/>
    </Root>)";
  const std::string scene_path = writeTempFile("scene_contract_bad.xml", scene);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser, IncludePublishesModelForClone)
{
  // A simple model file with a plain model (no overlays/requirements)
  const std::string models_xml = R"(
    <Root>
       <Object id="base"/>
    </Root>)";

  // Scene that includes the model file, then clones it
  const std::string scene_xml = R"(
    <Root>
      <Include uri="models_include.xml"/>
      <Object id="inst" model="base"/>
    </Root>)";

  const std::string models_path = writeTempFile("models_include.xml", models_xml);
  const std::string scene_path = writeTempFile("scene_include.xml", scene_xml);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "inst");
}

TEST(XMLParser, ImportPublishesModelForClone)
{
  // Imported file defines a plain model (no overlays)
  const std::string models_xml = R"(
    <Root>
      <Object id="base_import"/>
    </Root>)";

  // Scene imports the file, then clones the imported model
  const std::string scene_xml = R"(
    <Root>
      <Import uri="models_import.xml"/>
      <Object id="inst2" model="base_import"/>
    </Root>)";

  const std::string models_path = writeTempFile("models_import.xml", models_xml);
  const std::string scene_path = writeTempFile("scene_import.xml", scene_xml);

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "inst2");
}

TEST(XMLParser, OverlayRequiredEnforcedAndSatisfied)
{
  // A contract model: declares a required overlay slot (no id => required by default)
  const std::string model_xml = R"(
    <Root>
      <Object id="needs_hmi">
        <Overlay slot="hmi"/>
      </Object>

      <!-- Provide an overlay library entry for the slot -->
      <Overlay id="fw1" slot="hmi" target="needs_hmi">
        <!-- could add components here; not necessary for this minimal test -->
        <Add block="./">
          <Product/>
        </Add>
      </Overlay>
    </Root>)";
  const std::string model_path = writeTempFile("overlay_contract.xml", model_xml);

  // A) Scene that clones but does NOT satisfy the required slot -> MUST throw
  {
    const std::string scene_bad_xml = R"(
    <Root>
      <Include uri="overlay_contract.xml"/>
      <Object id="bad_inst" model="needs_hmi"/>
    </Root>)";
    const std::string scene_bad_path = writeTempFile("scene_overlay_bad.xml", scene_bad_xml);

    xml::EntityParser parser;
    ecs::Database db;
    ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_bad_path, db));
  }

  // B) Scene that supplies overlay id -> OK
  {
    const std::string scene_ok_xml = R"(
    <Root>
      <Include uri="overlay_contract.xml"/>
      <Object id="ok_inst" model="needs_hmi">
        <Overlay slot="hmi" id="fw1"/>
      </Object>
    </Root>)";
    const std::string scene_ok_path = writeTempFile("scene_overlay_ok.xml", scene_ok_xml);

    xml::EntityParser parser;
    ecs::Database db;
    ASSERT_TRUE(parser.loadEntitiesFromFile(scene_ok_path, db));

    auto ids = collectObjectIds(db);
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_EQ(ids[0], "ok_inst");
  }

  // C) Scene that disables the required slot -> OK
  {
    const std::string scene_disable_xml = R"(
    <Root>
      <Include uri="overlay_contract.xml"/>
      <Object id="disabled_inst" model="needs_hmi">
        <Overlay slot="hmi" disable="true"/>
      </Object>
    </Root>)";
    const std::string scene_disable_path = writeTempFile("scene_overlay_disable.xml", scene_disable_xml);

    xml::EntityParser parser;
    ecs::Database db;
    ASSERT_TRUE(parser.loadEntitiesFromFile(scene_disable_path, db));

    auto ids = collectObjectIds(db);
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_EQ(ids[0], "disabled_inst");
  }
}

TEST(XMLParser, ParseRectangleShape)
{
  std::string xml_txt = R"(
    <Shape type="Rectangle" origin="AABBCenter">
      <AxisHeight x="1.0" y="0.0" z="0.0"/>
      <AxisWidth  x="0.0" y="1.0" z="0.0"/>
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
      <Dimensions width="0.05" height="0.01"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Rectangle);
  EXPECT_DOUBLE_EQ(shape.dimensions[0], 0.05);
  EXPECT_DOUBLE_EQ(shape.dimensions[1], 0.01);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisHeight
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisWidth
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
}

TEST(XMLParser, ParseCircleShape)
{
  std::string xml_txt = R"(
    <Shape type="Circle" origin="AABBCenter">
      <AxisReferenceX x="1.0" y="0.0" z="0.0"/>
      <AxisNormal     x="0.0" y="0.0" z="1.0"/>
      <Dimensions radius="0.025"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Circle);
  EXPECT_DOUBLE_EQ(shape.dimensions[0], 0.025);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisReferenceX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisReferenceY
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
}

TEST(XMLParser, ParseTriangleShape)
{
  std::string xml_txt = R"(
    <Shape type="Triangle" origin="Native">
      <AxisAltitude x="1.0" y="0.0" z="0.0"/>
      <AxisBase     x="0.0" y="1.0" z="0.0"/>
      <AxisNormal   x="0.0" y="0.0" z="1.0"/>
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

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Triangle);
  ASSERT_EQ(shape.vertices.size(), 3);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisAltitude (X)
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisBase     (Y)
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal   (Z)
}

TEST(XMLParser, ParsePolygonShape)
{
  std::string xml_txt = R"(
    <Shape type="Polygon" origin="Native">
      <AxisX      x="1.0" y="0.0" z="0.0"/>
      <AxisY      x="0.0" y="1.0" z="0.0"/>
      <AxisNormal x="0.0" y="0.0" z="1.0"/>
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

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Polygon);
  ASSERT_EQ(shape.vertices.size(), 4);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisY
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisNormal
}

TEST(XMLParser, ParseBoxShape)
{
  std::string xml_txt = R"(
    <Shape type="Box" origin="AABBCenter">
      <AxisWidth x="0.0" y="1.0" z="0.0"/>
      <AxisDepth x="1.0" y="0.0" z="0.0"/>
      <AxisHeight x="0.0" y="0.0" z="1.0"/>
      <Dimensions width="0.05" height="0.01" depth="0.02"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Box);
  ASSERT_EQ(shape.axes.size(), 3);

  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisDepth (X=depth)
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisWidth (Y=width)
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisHeight (Z=height)
}

TEST(XMLParser, ParseCylinderShape)
{
  std::string xml_txt = R"(
    <Shape type="Cylinder" origin="AABBCenter">
      <AxisReferenceX x="1.0" y="0.0" z="0.0"/>
      <AxisSymmetry   x="0.0" y="0.0" z="1.0"/>
      <Dimensions radius="0.012" height="0.20"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Cylinder);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // AxisReferenceX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisSymmetry
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // AxisSymmetry
}

TEST(XMLParser, ParseSphereShape)
{
  std::string xml_txt = R"(
    <Shape type="Sphere" origin="AABBCenter">
      <Dimensions radius="0.015"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Sphere);
  ASSERT_EQ(shape.axes.size(), 0);  // No axes for spheres
}

TEST(XMLParser, ParseMeshShape)
{
  std::string xml_txt = R"(
    <Shape type="Mesh">
      <External uri="package://some/path/mesh.stl"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Mesh);
}

TEST(XMLParser, ParsePlaneShape)
{
  std::string xml_txt = R"(
    <Shape type="Plane" origin="AABBCenter">
      <AxisReferenceX x="1.0" y="0.0" z="0.0"/>
      <AxisReferenceY x="0.0" y="1.0" z="0.0"/>
      <AxisNormal     x="0.0" y="0.0" z="1.0"/>
      <Dimensions width="0.1" height="0.05"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Plane);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // RefX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // RefY
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // Normal
}

TEST(XMLParser, ParseConeShape)
{
  std::string xml_txt = R"(
    <Shape type="Cone" origin="AABBCenter">
      <AxisReferenceX x="1.0" y="0.0" z="0.0"/>
      <AxisSymmetry   x="0.0" y="0.0" z="1.0"/>
      <Dimensions base_radius="0.01" top_radius="0.002" height="0.025"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Cone);
  ASSERT_EQ(shape.axes.size(), 3);
  ASSERT_EQ(shape.axes.size(), 3);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(1.0, 0.0, 0.0));  // RefX
  EXPECT_EQ(shape.axes[1], Eigen::Vector3d(0.0, 1.0, 0.0));  // RefY
  EXPECT_EQ(shape.axes[2], Eigen::Vector3d(0.0, 0.0, 1.0));  // Symmetry
}

TEST(XMLParser, ParseLineShapeAnchor)
{
  std::string xml_txt = R"(
    <Shape type="Line" origin="AABBCenter">
      <AxisDirection x="0.0" y="1.0" z="0.0"/>
      <Dimensions length="0.15"/>
    </Shape>)";

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const auto* elem = doc.RootElement();
  ASSERT_TRUE(elem);

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 1);
  EXPECT_EQ(shape.axes[0], Eigen::Vector3d(0.0, 1.0, 0.0));  // AxisDirection
  EXPECT_NEAR(shape.dimensions[0], 0.15, 1e-12);
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

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 0);
  EXPECT_NEAR(shape.vertices[0].x(), 1.0, 1e-12);
  EXPECT_NEAR(shape.vertices[0].y(), 4.0, 1e-12);
  EXPECT_NEAR(shape.vertices[1].x(), 1.0, 1e-12);
  EXPECT_NEAR(shape.vertices[1].y(), 2.0, 1e-12);
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

  Shape shape = xml::parseShape(&doc, elem);

  EXPECT_EQ(shape.type, ShapeType::Line);
  ASSERT_EQ(shape.axes.size(), 0);
  EXPECT_NEAR(shape.vertices[0].x(), 1.0, 1e-12);
  EXPECT_NEAR(shape.vertices[0].y(), 2.0, 1e-12);
  EXPECT_NEAR(shape.vertices[0].z(), 3.0, 1e-12);
  EXPECT_NEAR(shape.vertices[1].x(), 6.0, 1e-12);
  EXPECT_NEAR(shape.vertices[1].y(), 3.0, 1e-12);
  EXPECT_NEAR(shape.vertices[1].z(), 4.0, 1e-12);
}

TEST(XMLParser, DomainShapeComponent_Fluid_Well200uL)
{
  std::string xml_txt = R"XML(
    <Root>
      <StackedShape id="stacked_shape/well_200ul">
        <Shape type="SphericalSegment" origin="BaseCenter">
          <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
        </Shape>
        <Shape type="Cone" origin="BaseCenter">
          <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
          <Transform>
            <Position x="0.0" y="0.0" z="${../../../Shape[0]/Dimensions.height}"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
        <Shape type="Cone" origin="BaseCenter">
          <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
          <Transform>
            <Position x="0.00000" y="0.00000"
                      z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height})"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
        <Shape type="Cylinder" origin="BaseCenter">
          <Dimensions radius="0.00275" height="0.0040"/>
          <Transform>
            <Position x="0.00000" y="0.00000"
                      z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height} + ${../../../Shape[2]/Dimensions.height})"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
      </StackedShape>

      <DomainShape id="domain_shape/well_200uL" type="Fluid">
        <StackedShapeRef id="stacked_shape/well_200ul"/>
      </DomainShape>
    </Root>
  )XML";

  ecs::Database db;
  auto eid = db.create();

  // 1) Parse string as XML document
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);

  // 2) Get root element
  const tinyxml2::XMLElement* root = doc.RootElement();
  ASSERT_TRUE(root);

  // 3) Parse StackedShape and DomainShape (order agnostic)
  for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "StackedShape")
    {
      xml::parseStackedShapeComponent(&doc, child, db, eid);
    }
    else if (tag == "DomainShape")
    {
      xml::parseDomainShapeComponent(&doc, child, db, eid);
    }
  }

  // 4) Verify stacked shape parsed
  auto* geom_shape = db.get_element<components::StackedShapeComponent>(eid, "stacked_shape/well_200ul");
  ASSERT_NE(geom_shape, nullptr);
  ASSERT_EQ(geom_shape->shapes.size(), 4u);

  // 5) Verify domain shape parsed & linked to stacked shape
  auto* domain_comp = db.get_element<components::DomainShapeComponent>(eid, "domain_shape/well_200ul");
  ASSERT_NE(domain_comp, nullptr);
  ASSERT_EQ(domain_comp->stacked_shape_id, "stacked_shape/well_200ul");
  ASSERT_EQ(domain_comp->segments().size(), 4u);

  const auto& segs = domain_comp->segments();

  // First: SphericalSegment
  {
    auto sph = std::dynamic_pointer_cast<sodf::physics::FluidSphericalSegmentShape>(segs.at(0));
    ASSERT_TRUE(bool(sph));
    EXPECT_NEAR(sph->base_radius_, 0.0, 1e-8);
    EXPECT_NEAR(sph->top_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(sph->getMaxFillHeight(), 0.00167, 1e-8);
  }

  // Second: Cone
  {
    auto c1 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(segs.at(1));
    ASSERT_TRUE(bool(c1));
    EXPECT_NEAR(c1->base_radius_, 0.00142, 1e-8);
    EXPECT_NEAR(c1->top_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(c1->getMaxFillHeight(), 0.00765, 1e-8);
  }

  // Third: Cone
  {
    auto c2 = std::dynamic_pointer_cast<sodf::physics::FluidConeShape>(segs.at(2));
    ASSERT_TRUE(bool(c2));
    EXPECT_NEAR(c2->base_radius_, 0.00256, 1e-8);
    EXPECT_NEAR(c2->top_radius_, 0.00275, 1e-8);
    EXPECT_NEAR(c2->getMaxFillHeight(), 0.00478, 1e-8);
  }

  // Fourth: Cylinder
  {
    auto cy = std::dynamic_pointer_cast<sodf::physics::FluidCylinderShape>(segs.at(3));
    ASSERT_TRUE(bool(cy));
    EXPECT_NEAR(cy->radius_, 0.00275, 1e-8);
    EXPECT_NEAR(cy->getMaxFillHeight(), 0.0040, 1e-8);
  }
}

TEST(XMLParser, ContainerComponent)
{
  // Use a tagged raw string so )" in attribute expressions doesn't break the literal.
  std::string xml_txt = R"XML(
    <Root>
      <!-- Geometry source -->
      <StackedShape id="stacked_shape/well_200ul">
        <Shape type="SphericalSegment" origin="BaseCenter">
          <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
        </Shape>
        <Shape type="Cone" origin="BaseCenter">
          <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
          <Transform>
            <Position x="0.0" y="0.0" z="${../../../Shape[0]/Dimensions.height}"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
        <Shape type="Cone" origin="BaseCenter">
          <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
          <Transform>
            <Position x="0.00000" y="0.00000"
                      z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height})"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
        <Shape type="Cylinder" origin="BaseCenter">
          <Dimensions radius="0.00275" height="0.0040"/>
          <Transform>
            <Position x="0.00000" y="0.00000"
                      z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height} + ${../../../Shape[2]/Dimensions.height})"/>
            <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
          </Transform>
        </Shape>
      </StackedShape>

      <!-- Domain built from that stack -->
      <DomainShape id="domain_shape/well_200ul" type="Fluid">
        <StackedShapeRef id="stacked_shape/well_200ul"/>
      </DomainShape>

      <!-- Insertion for this container -->
      <Insertion id="insertion/container/A1">
        <Transform parent="container/A1">
          <Position x="-0.01810" y="0.0" z="0.0"/>
          <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
        </Transform>
        <AxisInsertion x="1.0" y="0.0" z="0.0"/>
        <AxisReference x="0.0" y="1.0" z="0.0"/>
        <RotationalSymmetry value="2"/>
        <ApproachOffset value="0.0"/>
        <MaxDepth value="0.01810"/>
      </Insertion>

      <!-- Container using the domain + insertion -->
      <Container id="container/A1">
        <Transform parent="root">
          <Position x="0.0315" y="0.0495" z="0.0008"/>
          <Orientation roll="0.0" pitch="pi/2" yaw="0.0"/>
        </Transform>

        <!-- semantic content (kept simple for the test) -->
        <Content type="water" volume="100.0" units="uL"/>

        <!-- link to insertion -->
        <InsertionRef id="insertion/container/A1"/>

        <!-- payload + where it lives for this instance -->
        <PayloadDomain>
          <DomainShapeRef id="domain_shape/well_200ul">
            <Transform id="container/A1/shape" parent="container/A1">
              <Position x="0.0" y="0.0" z="0.0"/>
              <Orientation roll="0.0" pitch="-pi/2" yaw="0.0"/>
            </Transform>
          </DomainShapeRef>

          <!-- live fluid surface frame (for visualization/joint hookup) -->
          <LiquidLevelFrame id="container/A1/liquid_level"/>
        </PayloadDomain>

        <Material id=""/>
      </Container>
    </Root>
  )XML";

  ecs::Database db;
  auto eid = db.create();

  // Parse
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(xml_txt.c_str()), tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* root = doc.RootElement();
  ASSERT_TRUE(root);

  // Order-agnostic parse
  for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "StackedShape")
      xml::parseStackedShapeComponent(&doc, child, db, eid);
    else if (tag == "DomainShape")
      xml::parseDomainShapeComponent(&doc, child, db, eid);
    else if (tag == "Insertion")
      xml::parseInsertionComponent(&doc, child, db, eid);
    else if (tag == "Container")
      xml::parseContainerComponent(&doc, child, db, eid);
  }

  // Geometry source present
  auto* stacked = db.get_element<components::StackedShapeComponent>(eid, "stacked_shape/well_200ul");
  ASSERT_NE(stacked, nullptr);
  ASSERT_EQ(stacked->shapes.size(), 4u);

  // DomainShape present and linked to the stack; segments built (analytic)
  auto* domain = db.get_element<components::DomainShapeComponent>(eid, "domain_shape/well_200ul");
  ASSERT_NE(domain, nullptr);
  EXPECT_EQ(domain->stacked_shape_id, "stacked_shape/well_200ul");
  ASSERT_EQ(domain->segments().size(), 4u);

  // Insertion present
  auto* ins = db.get_element<components::InsertionComponent>(eid, "insertion/container/A1");
  ASSERT_NE(ins, nullptr);
  // basic axis sanity
  EXPECT_NEAR(ins->axis_insertion.x(), 1.0, 1e-12);
  EXPECT_NEAR(ins->axis_insertion.y(), 0.0, 1e-12);
  EXPECT_NEAR(ins->axis_insertion.z(), 0.0, 1e-12);
  EXPECT_NEAR(ins->max_depth, 0.01810, 1e-6);

  // Container present with updated fields
  auto* cont = db.get_element<components::ContainerComponent>(eid, "container/A1");
  ASSERT_NE(cont, nullptr);

  // Content & semantics
  EXPECT_EQ(cont->content_type, std::string("water"));

  // Insertion link
  EXPECT_EQ(cont->insertion_id, std::string("insertion/container/A1"));

  // Payload domain linkage
  EXPECT_EQ(cont->payload.domain_shape_id, std::string("domain_shape/well_200ul"));
  EXPECT_EQ(cont->payload.frame_id, std::string("container/A1/shape"));
  // Optional: if your parser converts uL→m^3 into payload.volume, this checks 100 uL = 1e-7 m^3.
  // If you haven't wired it yet, comment this out or relax it.
  // EXPECT_NEAR(CONT->payload.volume, 100.0e-9, 1e-12);

  // Fluid runtime linkage
  // Depending on your parser, joint id may be set equal to the frame id or left empty.
  // We assert the frame id (it’s explicitly authored).
  EXPECT_EQ(cont->fluid.liquid_level_frame_id, std::string("container/A1/liquid_level"));
  // If your parser also sets a joint id, enable the line below accordingly:
  // EXPECT_EQ(C.fluid.liquid_level_joint_id, std::string("container/A1/liquid_level"));

  // And sanity-check the referenced DomainShape again
  auto* dom_again = db.get_element<components::DomainShapeComponent>(eid, cont->payload.domain_shape_id);
  ASSERT_NE(dom_again, nullptr);
  ASSERT_EQ(dom_again->segments().size(), 4u);
}

TEST(XMLParser, ParallelGraspDerivedFrom)
{
  // No shapes defined
  {
    std::string xml_txt = R"(
    <Root>
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
   </Root>
  )";

    xml::EntityParser parser;
    ecs::Database db;
    ASSERT_ANY_THROW(parser.loadEntitiesFromText(xml_txt, db));
  }

  // No shapes defined
  {
    std::string xml_txt = R"(
    <Root>
      <Object id="test">

        <!-- FRONT (N = +X), centered at (x=-0.01, y=-0.30, z=0.70) -->
        <Shape id="area/front/handle" type="Rectangle" origin="AABBCenter">
          <Transform parent="root">
            <Position x="-0.01" y="-0.30" z="0.70"/>
            <Orientation roll="0" pitch="0" yaw="0"/>
          </Transform>
          <AxisHeight x="0" y="0" z="-1"/>
          <AxisWidth  x="0" y="1" z="0"/>
          <AxisNormal x="1" y="0" z="0"/>   <!-- (0,0,-1) × (0,1,0) = (1,0,0) -->
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- BACK (N = -X), centered at (x=+0.01, y=-0.30, z=0.70) -->
        <Shape id="area/back/handle" type="Rectangle" origin="AABBCenter">
          <Transform parent="root">
            <Position x="0.01" y="-0.30" z="0.70"/>
            <Orientation roll="0" pitch="0" yaw="0"/>
          </Transform>
          <AxisHeight x="0" y="0" z="1"/>    <!-- flip H -->
          <AxisWidth  x="0" y="1" z="0"/>
          <AxisNormal x="-1" y="0" z="0"/>   <!-- (0,0, 1) × (0,1,0) = (-1,0,0) -->
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- LEFT (N = -Y), centered at (x=0.00, y=-0.29, z=0.70) -->
        <Shape id="area/left/handle" type="Rectangle" origin="AABBCenter">
          <Transform parent="root">
            <Position x="0.00" y="-0.29" z="0.70"/>
            <Orientation roll="0" pitch="0" yaw="0"/>
          </Transform>
          <AxisHeight x="0" y="0" z="-1"/>
          <AxisWidth  x="1" y="0" z="0"/>    <!-- choose W = +X -->
          <AxisNormal x="0" y="-1" z="0"/>   <!-- (0,0,-1) × (1,0,0) = (0,-1,0) -->
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <!-- RIGHT (N = +Y), centered at (x=0.00, y=-0.31, z=0.70) -->
        <Shape id="area/right/handle" type="Rectangle" origin="AABBCenter">
          <Transform parent="root">
            <Position x="0.00" y="-0.31" z="0.70"/>
            <Orientation roll="0" pitch="0" yaw="0"/>
          </Transform>
          <AxisHeight x="0" y="0" z="-1"/>
          <AxisWidth  x="-1" y="0" z="0"/>   <!-- flip W -->
          <AxisNormal x="0" y="1" z="0"/>    <!-- (0,0,-1) × (-1,0,0) = (0,1,0) -->
          <Dimensions width="0.01" height="0.05"/>
        </Shape>

        <ParallelGrasp id="grasp/handle">
          <DerivedFromParallelShapes>
            <!-- Use the FRONT face as the ‘first’ shape -->
            <AxisRotationalFirstShape x="0" y="0" z="-1"/>  <!-- its Height -->
            <AxisNormalFirstShape    x="1" y="0" z="0"/>    <!-- its Normal -->
            <AxisRotationalGrasp     x="1" y="0" z="0"/>    <!-- as you had -->
            <Shape id="area/front/handle"/>
            <Shape id="area/back/handle"/>
            <Shape id="area/left/handle"/>
            <Shape id="area/right/handle"/>
          </DerivedFromParallelShapes>
        </ParallelGrasp>

      </Object>
    </Root>
    )";

    xml::EntityParser parser;
    ecs::Database db;
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ecs::Database::entity_type> id_map;
    db.each([&id_map](ecs::Database::entity_type id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });

    ASSERT_TRUE(!id_map.empty());

    auto& sid = id_map.begin()->first;
    auto& eid = id_map.begin()->second;

    auto grasp = db.get_element<components::ParallelGraspComponent>(eid, "grasp/handle");
    auto tf_node = db.get_element<components::TransformComponent>(eid, "grasp/handle");
    ASSERT_NE(grasp, nullptr);
    ASSERT_NE(tf_node, nullptr);

    ASSERT_EQ(0.02, grasp->gap_size);
    ASSERT_EQ(ParallelGrasp::GraspType::INTERNAL, grasp->grasp_type);
    ASSERT_EQ(4, grasp->rotational_symmetry);
    ASSERT_EQ("area/front/handle", grasp->contact_shape_ids[0]);
    ASSERT_EQ("area/back/handle", grasp->contact_shape_ids[1]);

    // ASSERT_TRUE((tf_ptr->local.linear() * grasp.axis_of_rotation).isApprox(Eigen::Vector3d(1, 0, 0)));
    ASSERT_TRUE(grasp->rotation_axis.isApprox(Eigen::Vector3d(1, 0, 0)));

    auto& vshape = grasp->canonical_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.01, vshape.dimensions.at(0));
    ASSERT_EQ(0.05, vshape.dimensions.at(1));
    ASSERT_TRUE(vshape.vertices.empty());
    ASSERT_EQ(3, vshape.axes.size());
    ASSERT_FALSE(sodf::geometry::hasExternalMesh(vshape));
    ASSERT_FALSE(sodf::geometry::hasInlineMesh(vshape));
    ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));  // Height
    ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 1, 0)));  // Width
    ASSERT_TRUE(vshape.axes[2].isApprox(Eigen::Vector3d(0, 0, 1)));  // Normal
  }

  {
    std::string xml_txt = R"(
  <Root>
    <Object id="test">

      <Shape id="box" type="Box" origin="AABBCenter">
        <Transform parent="root">
          <Position x="0.0" y="-0.3" z="0.1"/>
          <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
        </Transform>
        <!-- Box modeling axes can be anything orthonormal; they do NOT define vshape.axes -->
        <AxisHeight x="1.0" y="0.0" z="0.0"/>
        <AxisDepth  x="0.0" y="1.0" z="0.0"/>
        <AxisWidth  x="0.0" y="0.0" z="1.0"/>
        <Dimensions width="0.02" depth="0.02" height="0.05"/>
      </Shape>

      <ParallelGrasp id="grasp">
        <DerivedFromParallelShapes>
          <GraspType value="Internal"/>
          <!-- Grasp frame definition -->
          <AxisRotationalFirstShape x="0.0" y="0.0" z="-1.0"/>  <!-- x -->
          <AxisNormalFirstShape    x="1.0" y="0.0" z="0.0"/>   <!-- z -->
          <AxisRotationalGrasp     x="1.0" y="0.0" z="0.0"/>   <!-- for metadata -->
          <Shape id="box"/>
        </DerivedFromParallelShapes>
      </ParallelGrasp>

    </Object>
  </Root>
  )";

    xml::EntityParser parser;
    ecs::Database db;
    parser.loadEntitiesFromText(xml_txt, db);

    std::unordered_map<std::string, ecs::Database::entity_type> id_map;
    db.each([&id_map](ecs::Database::entity_type id, components::ObjectComponent& object) {
      id_map.insert({ object.id, id });
    });
    ASSERT_FALSE(id_map.empty());
    auto eid = id_map.begin()->second;

    auto grasp = db.get_element<components::ParallelGraspComponent>(eid, "grasp");
    auto tf_node = db.get_element<components::TransformComponent>(eid, "grasp");
    ASSERT_NE(grasp, nullptr);
    ASSERT_NE(tf_node, nullptr);

    // Core grasp properties
    EXPECT_EQ(0.02, grasp->gap_size);
    EXPECT_EQ(ParallelGrasp::GraspType::INTERNAL, grasp->grasp_type);
    EXPECT_EQ(4u, grasp->rotational_symmetry);
    ASSERT_EQ(1u, grasp->contact_shape_ids.size());
    EXPECT_EQ("box", grasp->contact_shape_ids[0]);
    EXPECT_TRUE(grasp->rotation_axis.isApprox(Eigen::Vector3d(1, 0, 0)));  // stored in world

    // Verify the grasp frame itself (world)
    const Eigen::Matrix3d& R = tf_node->local.linear();
    const Eigen::Vector3d xg = R.col(0);  // should follow AxisRotationalFirstShape
    const Eigen::Vector3d zg = R.col(2);  // should follow AxisNormalFirstShape
    const Eigen::Vector3d yg = R.col(1);  // z × x
    EXPECT_TRUE(xg.isApprox(Eigen::Vector3d(0, 0, -1)));
    EXPECT_TRUE(zg.isApprox(Eigen::Vector3d(1, 0, 0)));
    EXPECT_TRUE(yg.isApprox(Eigen::Vector3d(0, 1, 0)));
    // Right-handed check
    EXPECT_GT(xg.dot(yg.cross(zg)), 0.0);

    // Canonical surface (in grasp frame!)
    auto& vshape = grasp->canonical_surface;
    ASSERT_EQ(ShapeType::Rectangle, vshape.type);
    ASSERT_EQ(0.02, vshape.dimensions.at(0));  // width
    ASSERT_EQ(0.05, vshape.dimensions.at(1));  // height
    ASSERT_EQ(3u, vshape.axes.size());
    EXPECT_FALSE(sodf::geometry::hasExternalMesh(vshape));
    EXPECT_FALSE(sodf::geometry::hasInlineMesh(vshape));

    // Axes are **in grasp coordinates** and must be the grasp basis in canonical rectangle order [Height, Width, Normal]
    EXPECT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));  // Height = +Xᵍ
    EXPECT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 1, 0)));  // Width  = +Yᵍ
    EXPECT_TRUE(vshape.axes[2].isApprox(Eigen::Vector3d(0, 0, 1)));  // Normal = +Zᵍ
  }

  {
    // std::string xml_txt = R"(
    // <Root>
    //   <Object id="test">

    //     <Shape id="cylinder" type="Cylinder" origin="AABBCenter">
    //       <Transform parent="root">
    //         <Position x="0.0" y="0.0" z="1.0"/>
    //         <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
    //       </Transform>
    //       <AxisReferenceX x="1.0" y="0.0" z="0.0"/>
    //       <AxisSymmetry   x="0.0" y="0.0" z="1.0"/>
    //       <Dimensions radius="0.012" height="0.020"/>
    //     </Shape>

    //     <ParallelGrasp id="grasp">
    //       <DerivedFromParallelShapes>
    //         <GraspType value="Internal"/>
    //         <AxisRotationalFirstShape x="0.0" y="0.0" z="1.0"/>
    //         <AxisNormalFirstShape x="1.0" y="0.0" z="0.0"/>
    //         <AxisRotationalGrasp x="1.0" y="0.0" z="0.0"/>
    //         <Shape id="cylinder"/>
    //       </DerivedFromParallelShapes>
    //     </ParallelGrasp>

    //   </Object>
    // </Root>
    // )";

    // xml::EntityParser parser;
    // ecs::Database db;
    // parser.loadEntitiesFromText(xml_txt, db);

    // std::unordered_map<std::string, ecs::Database::entity_type> id_map;
    // db.each([&id_map](ecs::Database::entity_type id, components::ObjectComponent& object) {
    //   id_map.insert({ object.id, id });
    // });

    // ASSERT_TRUE(!id_map.empty());

    // auto& sid = id_map.begin()->first;
    // auto& eid = id_map.begin()->second;

    // auto grasp = db.get_element<components::ParallelGraspComponent>(eid, "grasp");
    // auto tf_node = db.get_element<components::TransformComponent>(eid, "grasp");
    // ASSERT_NE(grasp, nullptr);
    // ASSERT_NE(tf_node, nullptr);

    // ASSERT_EQ(0.024, grasp->gap_size);
    // ASSERT_EQ(ParallelGrasp::GraspType::INTERNAL, grasp->grasp_type);
    // ASSERT_EQ(0, grasp->rotational_symmetry);
    // ASSERT_EQ(1, grasp->contact_shape_ids.size());
    // ASSERT_EQ("cylinder", grasp->contact_shape_ids[0]);
    // ASSERT_TRUE(grasp->rotation_axis.isApprox(Eigen::Vector3d(1, 0, 0)));

    // auto& vshape = grasp->canonical_surface;
    // ASSERT_EQ(ShapeType::Line, vshape.type);
    // ASSERT_EQ(1, vshape.dimensions.size());
    // ASSERT_EQ(0.02, vshape.dimensions.at(0));

    // ASSERT_EQ(2, vshape.axes.size());
    // ASSERT_TRUE(vshape.axes[0].isApprox(Eigen::Vector3d(1, 0, 0)));
    // ASSERT_TRUE(vshape.axes[1].isApprox(Eigen::Vector3d(0, 0, 1)));
    // ASSERT_FALSE(sodf::geometry::hasExternalMesh(vshape));
    // ASSERT_FALSE(sodf::geometry::hasInlineMesh(vshape));
    // ASSERT_EQ(2, vshape.vertices.size());
    // ASSERT_TRUE(vshape.vertices[0].isApprox(Eigen::Vector3d(-0.01, 0, 0)));
    // ASSERT_TRUE(vshape.vertices[1].isApprox(Eigen::Vector3d(0.01, 0, 0)));
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
  // Use a tagged raw string so attribute math like ")"
  // doesn't terminate the literal accidentally.
  std::string xml_txt = R"XML(
    <Root>
      <Object id="plate">

        <!-- Geometry source for wells -->
        <StackedShape id="stacked_shape/well_250ul">
          <Shape type="SphericalSegment" origin="BaseCenter">
            <Dimensions base_radius="0.0" top_radius="0.00142" height="0.00167"/>
          </Shape>
          <Shape type="Cone" origin="BaseCenter">
            <Dimensions base_radius="0.00142" top_radius="0.00256" height="0.00765"/>
            <Transform>
              <Position x="0.0" y="0.0" z="${../../../Shape[0]/Dimensions.height}"/>
              <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
            </Transform>
          </Shape>
          <Shape type="Cone" origin="BaseCenter">
            <Dimensions base_radius="0.00256" top_radius="0.00275" height="0.00478"/>
            <Transform>
              <Position x="0.00000" y="0.00000"
                        z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height})"/>
              <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
            </Transform>
          </Shape>
          <Shape type="Cylinder" origin="BaseCenter">
            <Dimensions radius="0.00275" height="0.0040"/>
            <Transform>
              <Position x="0.00000" y="0.00000"
                        z="(${../../../Shape[0]/Dimensions.height} + ${../../../Shape[1]/Dimensions.height} + ${../../../Shape[2]/Dimensions.height})"/>
              <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
            </Transform>
          </Shape>
        </StackedShape>

        <!-- Domain built from that stack -->
        <DomainShape id="domain_shape/well_250ul" type="Fluid">
          <StackedShapeRef id="stacked_shape/well_250ul"/>
        </DomainShape>

        <!-- 2x3 loop: A..B rows, 1..3 cols -->
        <ForLoop row_name="A:B:1" row="1:2:1" col="1:3:1" zipped="[row_name,row]">

          <Container id="container/{row_name}{col}">
            <Transform parent="root">
              <Position x="{col}.1" y="{row}.2" z="0.0008"/>
              <Orientation roll="0.0" pitch="1.57079632679" yaw="0.0"/>
            </Transform>

            <Material id="polypropylene"/>
            <Content type="empty" volume="0.0" units="uL"/>

            <PayloadDomain>
              <DomainShapeRef id="domain_shape/well_250ul">
                <!-- Per-instance frame where the payload lives -->
                <Transform id="container/{row_name}{col}/shape" parent="container/{row_name}{col}">
                  <Position x="0.0" y="0.0" z="0.0"/>
                  <Orientation roll="0.0" pitch="-pi/2" yaw="0.0"/>
                </Transform>
              </DomainShapeRef>

              <!-- Live fluid surface frame -->
              <LiquidLevelFrame id="container/{row_name}{col}/liquid_level"/>
            </PayloadDomain>
          </Container>

        </ForLoop>
      </Object>
    </Root>
  )XML";

  // Parse the scene into an entity
  xml::EntityParser parser;
  ecs::Database db;
  parser.loadEntitiesFromText(xml_txt, db);

  // Find the single object entity we just created
  std::unordered_map<std::string, ecs::Database::entity_type> id_map;
  db.each([&id_map](ecs::Database::entity_type id, components::ObjectComponent& object) {
    id_map.insert({ object.id, id });
  });

  ASSERT_FALSE(id_map.empty());
  ASSERT_EQ(1u, id_map.size());
  const auto& eid = id_map.begin()->second;

  // Container ids expected from the 2×3 ForLoop
  std::vector<std::string> expected_ids = { "container/A1", "container/A2", "container/A3",
                                            "container/B1", "container/B2", "container/B3" };

  // DomainShape exists and is backed by the stacked shape
  {
    auto* domain = db.get_element<components::DomainShapeComponent>(eid, "domain_shape/well_250ul");
    ASSERT_NE(domain, nullptr);
    EXPECT_EQ(domain->stacked_shape_id, "stacked_shape/well_250ul");
    ASSERT_EQ(domain->segments().size(), 4u);
  }

  // Validate each container under the new Container schema
  for (const auto& cid : expected_ids)
  {
    auto* cont = db.get_element<components::ContainerComponent>(eid, cid);
    ASSERT_NE(cont, nullptr) << "Missing ContainerComponent entry for " << cid;

    // Content semantics
    EXPECT_EQ(cont->content_type, std::string("empty"));

    // Payload domain linkage
    EXPECT_EQ(cont->payload.domain_shape_id, std::string("domain_shape/well_250ul"));
    EXPECT_EQ(cont->payload.frame_id, cid + std::string("/shape"));
    EXPECT_DOUBLE_EQ(cont->payload.volume, 0.0);  // 0 uL → 0 m^3 either way

    // Fluid runtime frame
    EXPECT_EQ(cont->fluid.liquid_level_frame_id, cid + std::string("/liquid_level"));

    // If your parser also synthesizes a joint id, you can assert it here:
    // EXPECT_EQ(C.fluid.liquid_level_joint_id, cid + std::string("/liquid_level"));
  }
}
