#include <cmath>
#include <filesystem>
#include <fstream>

#include <sodf/xml/entity_parser.h>
#include <sodf/xml/element_parser.h>
#include <sodf/xml/component_parser.h>
#include <sodf/xml/for_loop_parser.h>

#include <sodf/components/object.h>
#include <sodf/components/transform.h>
#include <sodf/components/shape.h>
#include <sodf/components/container.h>

#include <gtest/gtest.h>

using namespace sodf;
using namespace sodf::components;
namespace fs = std::filesystem;

static std::string writeTempFile(const std::string& filename, const std::string& contents)
{
  fs::path dir = fs::temp_directory_path() / "sodf_patch_ops_tests";
  fs::create_directories(dir);
  fs::path p = dir / filename;
  std::ofstream(p.string()) << contents;
  return p.string();
}

static std::vector<std::string> collectObjectIds(ecs::Database& db)
{
  std::vector<std::string> ids;
  db.each([&](ecs::Database::entity_type, const components::ObjectComponent& obj) { ids.push_back(obj.id); });
  return ids;
}

// Compose a scene doc by injecting object bodies
static std::string makeScene(const std::string& objects_xml)
{
  return std::string(R"(<?xml version="1.0" encoding="utf-8"?>
<Root sodf_version="0.0.7">
  <Include uri="bio-rad-t100-thermal-cycler.xml" ns="allo"/>
  <Include uri="bio-rad-t100-thermal-cycler.hmi-overlay.xml"/>
)") + objects_xml +
         "\n</Root>\n";
}

// Minimal model + overlay library used across tests
static const char* kModelXml = R"(<?xml version="1.0" encoding="utf-8"?>
<Root sodf_version="0.0.7">
  <Object id="bio-rad:t100-thermal-cycler">
    <!-- contract slot to force overlay -->
    <Overlay slot="hmi"/>
    <Origin id="root">
      <Transform parent="world">
        <Position x="0" y="0" z="0"/>
        <Orientation roll="0" pitch="0" yaw="0"/>
      </Transform>
    </Origin>
    <Origin id="aux">
      <Transform parent="root">
        <Position x="1" y="1" z="1"/>
        <Orientation roll="0" pitch="0" yaw="0"/>
      </Transform>
    </Origin>
    <Link id="link/base">
      <Visual>
        <Shape type="Mesh">
          <External uri="sodf://bio-rad/t100-thermal-cycler/visual/base_161k.dae"/>
          <Scale x="1.0" y="1.0" z="1.0"/>
        </Shape>
      </Visual>
      <Collision>
        <Shape type="Mesh">
          <External uri="sodf://bio-rad/t100-thermal-cycler/collision/base_161k.stl"/>
          <Scale x="1.0" y="1.0" z="1.0"/>
        </Shape>
      </Collision>
      <Inertial>
        <Mass value="0.250"/>
        <CenterOfMass x="0.0" y="0.0" z="0.0"/>
        <Tensor ixx="0.001" ixy="0.000" ixz="0.000" iyy="0.001" iyz="0.000" izz="0.001"/>
      </Inertial>
    </Link>                            
    <FSM id="fsm/touchscreen">
      <StartState name="home"/>
      <States>
        <State name="home"/>
        <State name="incubate"/>
        <State name="saved_protocols"/>
      </States>
      <Actions>
        <Action name="incubate"/>
        <Action name="saved_protocols"/>
      </Actions>
      <Transitions>
        <!-- home -->
        <Transition from="home" action="incubate" to="incubate"/>
        <Transition from="home" action="saved_protocols" to="saved_protocols"/>
      </Transitions>
    </FSM>

  </Object>
</Root>)";

static const char* kOverlayXml = R"(<?xml version="1.0" encoding="utf-8"?>
<Root sodf_version="0.0.7">
  <Overlay id="hmi-fw-2.4.3" slot="hmi" target="bio-rad:t100-thermal-cycler">
    <!-- Direct components introduced by overlay -->
    <Shape id="shape/ui_button" type="Rectangle" origin="AABBCenter">
      <AxisNormal x="1.0" y="0.0" z="0.0"/>
      <AxisWidth x="0.0" y="1.0" z="0.0"/>
      <AxisHeight x="0.0" y="0.0" z="-1.0"/>
      <Dimensions width="0.05" height="0.01"/>
    </Shape>

    <VirtualButton id="btn/incubate">
      <Shape id="shape/ui_button"/>
      <Label text="incubate"/>
      <Image uri=""/>
      <Transform parent="touchscreen">
        <Position x="0.0" y="-0.01425" z="-0.0026"/>
      </Transform>
      <AxisPress x="1.0" y="0.0" z="0.0"/>
      <ActionMap fsm="fsm/touchscreen">
        <Map trigger="press" action="incubate"/>
      </ActionMap>
    </VirtualButton>
    <!-- Patch ops INSIDE overlay (must be applied in-order) -->
    <Add block="/Origin[@id='root']/Transform">
      <Orientation roll="0" pitch="0" yaw="0"/>
    </Add>
    <Upsert attr="/Origin[@id='root']" name="overlay_attr" value="ok"/>
  </Overlay>
</Root>)";

TEST(XMLParser_PatchOps, OverlayAllowsPatchOps_InOrder_NoThrow)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_overlay" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>
    <!-- also do a scene-level inline remove to ensure order -->
    <Remove block="FSM[@id=fsm/touchscreen]"/>
  </Object>)";

  const std::string scene_path = writeTempFile("scene_overlay_ops.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "pcr_overlay");
}

TEST(XMLParser_PatchOps, Update_Block_Tag_Attr_Variants_NoThrow)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_update" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>

    <Update block="/Origin[@id=root]">
      <Origin id="root">
        <Transform parent="table">
          <Position x="0.0" y="0.0" z="3.0"/>
          <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
        </Transform>
      </Origin>
    </Update>

    <Update tag="/Origin[@id=root]" name="Origin"/>
    <Update attr="/Origin[@id=root].id" value="root2"/>

    <Update block="/Origin[@id=root]/Transform/Position">
      <Position x="2.0" y="2.0"/>
    </Update>
  </Object>)";

  const std::string scene_path = writeTempFile("scene_update_variants.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "pcr_update");
}

TEST(XMLParser_PatchOps, AddAttr_Duplicate_Throws)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_adddup" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>
    <Add attr="/Origin[@id=root]" name="parent" value="6.0"/>
    <Add attr="/Origin[@id=root]" name="parent" value="6.0"/> <!-- duplicate -->
  </Object>)";

  const std::string scene_path = writeTempFile("scene_add_attr_dup.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_ANY_THROW(parser.loadEntitiesFromFile(scene_path, db));
}

TEST(XMLParser_PatchOps, UpsertAttr_Replaces_NoThrow)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_upsert" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>
    <Add    attr="/Origin[@id=root]" name="parent" value="6.0"/>
    <Upsert attr="/Origin[@id=root]" name="parent" value="7.5"/>
  </Object>)";

  const std::string scene_path = writeTempFile("scene_upsert_attr.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "pcr_upsert");
}

TEST(XMLParser_PatchOps, AddBlock_AnchorBefore_NoThrow)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_addblock" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>
    <Add block="/Origin[@id=root]" anchor="before">
      <Origin id="pre_root"/>
      <Position x="" y="" z=""/>
    </Add>
  </Object>)";

  const std::string scene_path = writeTempFile("scene_add_block_before.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "pcr_addblock");
}

TEST(XMLParser_PatchOps, FullMatrix_Add_Update_Upsert_Remove_NoThrow)
{
  const std::string models_path = writeTempFile("bio-rad-t100-thermal-cycler.xml", kModelXml);
  const std::string overlay_path = writeTempFile("bio-rad-t100-thermal-cycler.hmi-overlay.xml", kOverlayXml);

  const std::string objects = R"(
  <Object id="pcr_full" model="allo:bio-rad:t100-thermal-cycler">
    <Overlay id="hmi-fw-2.4.3" slot="hmi"/>

    <!-- UPDATEs -->
    <Update block="/Origin[@id=root]">
      <Origin id="root">
        <Transform parent="table">
          <Position x="0.0" y="0.0" z="0.0"/>
          <Orientation roll="0.0" pitch="0.0" yaw="0.0"/>
        </Transform>
      </Origin>
    </Update>
    <Update tag="/Origin[@id=root]" name="Origin"/>
    <Update attr="/Origin[@id=root].id" value="root2"/>
    <Update block="/Origin[@id=root]/Transform/Position">
      <Position x="2.0" y="2.0" z="2.0"/>
    </Update>

    <!-- ADDs / UPSERTs -->
    <Add attr="/Origin[@id=root]" name="alpha" value="1"/>
    <Upsert attr="/Origin[@id=root]" name="alpha" value="2"/>
    <Add block="/Origin[@id=root]" anchor="after">
      <Origin id="after_root"/>
    </Add>

    <!-- REMOVE selector zoo -->
    <Remove block="Origin[0]"/>
  </Object>)";

  const std::string scene_path = writeTempFile("scene_full_matrix.xml", makeScene(objects));

  xml::EntityParser parser;
  ecs::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(scene_path, db));

  auto ids = collectObjectIds(db);
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids[0], "pcr_full");
}
