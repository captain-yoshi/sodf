
#include <sodf/ecs.h>
#include <sodf/xml/entity_parser.h>

#include <sodf/components/button.h>
#include <sodf/components/joint.h>
#include <sodf/components/link.h>
#include <sodf/components/finite_state_machine.h>
#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/transform.h>

#include <sodf/systems/fsm.h>
#include <sodf/systems/scene_graph.h>

#include <gtest/gtest.h>

using namespace sodf;

TEST(ECS, ParsingSingleObject)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/bio-rad-t100-thermal-cycler.no-overlay.xml";

  xml::EntityParser parser;
  auto db = ginseng::database{};
  parser.loadEntitiesFromFile(filename, db);

  std::unordered_map<std::string, ginseng::database::ent_id> id_map;
  db.visit([&id_map](ginseng::database::ent_id id, components::ObjectComponent& object) {
    id_map.insert({ object.id, id });
  });

  ASSERT_TRUE(!id_map.empty());

  auto& sid = id_map.begin()->first;
  auto& eid = id_map.begin()->second;

  ASSERT_TRUE(db.has_component<components::ObjectComponent>(eid));
  ASSERT_TRUE(db.has_component<components::TransformComponent>(eid));
  ASSERT_TRUE(db.has_component<components::LinkComponent>(eid));
  ASSERT_TRUE(db.has_component<components::JointComponent>(eid));
  ASSERT_TRUE(db.has_component<components::TouchscreenComponent>(eid));
  ASSERT_TRUE(db.has_component<components::FSMComponent>(eid));
  ASSERT_TRUE(db.has_component<components::ActionMapComponent>(eid));
  ASSERT_TRUE(db.has_component<components::VirtualButtonComponent>(eid));

  // Object component validation
  auto& object = db.get_component<components::ObjectComponent>(eid);
  EXPECT_EQ(object.id, "bio-rad:t100-thermal-cycler");
  EXPECT_EQ(object.name, "T100 Thermal Cycler");
  EXPECT_EQ(object.model, "T100");
  EXPECT_EQ(object.serial_number, "");
  EXPECT_EQ(object.vendor, "Bio-Rad");

  // Transform component validation
  auto& transform = db.get_component<components::TransformComponent>(eid);
  EXPECT_EQ(transform.elements.size(), 69);

  // Link component validation
  auto& link = db.get_component<components::LinkComponent>(eid);
  EXPECT_EQ(link.elements.size(), 3);

  // Joint component validation
  auto& joint = db.get_component<components::JointComponent>(eid);
  EXPECT_EQ(joint.elements.size(), 2);

  // Button component validation
  auto& button = db.get_component<components::VirtualButtonComponent>(eid);
  EXPECT_EQ(button.elements.size(), 51);

  // system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok" });
  // system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok", "zero" });
}

TEST(ECS, ParseSceneA)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/scene-a.xml";

  xml::EntityParser parser;
  auto db = ginseng::database{};
  parser.loadEntitiesFromFile(filename, db);

  std::vector<std::pair<std::string, ginseng::database::ent_id>> ids;
  db.visit(
      [&ids](ginseng::database::ent_id id, components::ObjectComponent& object) { ids.emplace_back(object.id, id); });

  ASSERT_TRUE(!ids.empty());

  // pcr1
  // clone: thermocycler clone
  // remove:
  //   link/base
  //   btn/incubate
  //   fsm/touchscreen
  // update: root

  {
    auto it = std::find_if(ids.begin(), ids.end(), [](const auto& pair) { return pair.first == "pcr1"; });

    ASSERT_NE(it, ids.end()) << "Entity with id='pcr1' not found";

    auto& sid = it->first;
    auto& eid = it->second;

    ASSERT_TRUE(db.has_component<components::ObjectComponent>(eid));
    ASSERT_TRUE(db.has_component<components::TransformComponent>(eid));
    ASSERT_TRUE(db.has_component<components::LinkComponent>(eid));
    ASSERT_TRUE(db.has_component<components::JointComponent>(eid));
    ASSERT_TRUE(db.has_component<components::TouchscreenComponent>(eid));
    ASSERT_FALSE(db.has_component<components::FSMComponent>(eid));
    ASSERT_TRUE(db.has_component<components::ActionMapComponent>(eid));
    ASSERT_TRUE(db.has_component<components::OriginComponent>(eid));
    ASSERT_TRUE(db.has_component<components::VirtualButtonComponent>(eid));

    // Object component validation
    auto& object = db.get_component<components::ObjectComponent>(eid);
    EXPECT_EQ(object.id, "pcr1");
    EXPECT_EQ(object.name, "T100 Thermal Cycler");
    EXPECT_EQ(object.model, "T100");
    EXPECT_EQ(object.serial_number, "");
    EXPECT_EQ(object.vendor, "Bio-Rad");

    // Transform component validation
    auto& transform = db.get_component<components::TransformComponent>(eid);
    EXPECT_EQ(transform.elements.size(), 64);

    // Link component validation
    auto& link = db.get_component<components::LinkComponent>(eid);
    EXPECT_EQ(link.elements.size(), 2);

    // Joint component validation
    auto& joint = db.get_component<components::JointComponent>(eid);
    EXPECT_EQ(joint.elements.size(), 2);
  }

  auto obj_ent_map = sodf::systems::make_object_entity_map(db);
  sodf::systems::align_origin_transforms(db, obj_ent_map);
  sodf::systems::resolve_transform_parent_entities(db, obj_ent_map);
  sodf::systems::update_all_global_transforms(db);
}

TEST(ECS, ParseSceneB)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/scene-b.xml";

  xml::EntityParser parser;
  auto db = ginseng::database{};
  parser.loadEntitiesFromFile(filename, db);

  std::vector<std::pair<std::string, ginseng::database::ent_id>> ids;
  db.visit(
      [&ids](ginseng::database::ent_id id, components::ObjectComponent& object) { ids.emplace_back(object.id, id); });

  ASSERT_TRUE(!ids.empty());

  // pcr1
  // clone: thermocycler clone
  // remove:
  //   link/base
  //   btn/incubate
  //   fsm/touchscreen
  // update: root

  {
    auto& sid = ids[0].first;
    auto& eid = ids[0].second;

    ASSERT_TRUE(db.has_component<components::ObjectComponent>(eid));
    ASSERT_TRUE(db.has_component<components::TransformComponent>(eid));
    ASSERT_TRUE(db.has_component<components::LinkComponent>(eid));
    ASSERT_TRUE(db.has_component<components::JointComponent>(eid));
    ASSERT_TRUE(db.has_component<components::TouchscreenComponent>(eid));
    ASSERT_FALSE(db.has_component<components::FSMComponent>(eid));
    ASSERT_TRUE(db.has_component<components::ActionMapComponent>(eid));
    ASSERT_TRUE(db.has_component<components::OriginComponent>(eid));

    // Object component validation
    auto& object = db.get_component<components::ObjectComponent>(eid);
    EXPECT_EQ(object.id, "pcr3");
    EXPECT_EQ(object.name, "T100 Thermal Cycler");
    EXPECT_EQ(object.model, "T100");
    EXPECT_EQ(object.serial_number, "");
    EXPECT_EQ(object.vendor, "Bio-Rad");

    // Transform component validation
    auto& transform = db.get_component<components::TransformComponent>(eid);
    EXPECT_EQ(transform.elements.size(), 64);

    // Link component validation
    auto& link = db.get_component<components::LinkComponent>(eid);
    EXPECT_EQ(link.elements.size(), 2);

    // Joint component validation
    auto& joint = db.get_component<components::JointComponent>(eid);
    EXPECT_EQ(joint.elements.size(), 2);
  }
}
