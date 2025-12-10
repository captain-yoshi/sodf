
#include <sodf/database/database.h>
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

TEST(DATABASE, ParsingSingleObject)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/bio-rad-t100-thermal-cycler.no-overlay.xml";

  xml::EntityParser parser;
  database::Database db;
  parser.loadEntitiesFromFile(filename, db);

  std::unordered_map<std::string, database::Database::entity_type> id_map;
  db.each([&id_map](database::Database::entity_type id, components::ObjectComponent& object) {
    id_map.insert({ object.id, id });
  });

  ASSERT_TRUE(!id_map.empty());

  auto& sid = id_map.begin()->first;
  auto& eid = id_map.begin()->second;

  ASSERT_TRUE(db.has<components::ObjectComponent>(eid));
  ASSERT_TRUE(db.has<components::TransformComponent>(eid));
  ASSERT_TRUE(db.has<components::LinkComponent>(eid));
  ASSERT_TRUE(db.has<components::JointComponent>(eid));
  ASSERT_TRUE(db.has<components::TouchscreenComponent>(eid));
  ASSERT_TRUE(db.has<components::FSMComponent>(eid));
  ASSERT_TRUE(db.has<components::ActionMapComponent>(eid));
  ASSERT_TRUE(db.has<components::VirtualButtonComponent>(eid));

  // Object component validation
  auto* object = db.get<components::ObjectComponent>(eid);
  EXPECT_EQ(object->id, "bio-rad:t100-thermal-cycler");
  EXPECT_EQ(object->name, "T100 Thermal Cycler");
  EXPECT_EQ(object->model, "T100");
  EXPECT_EQ(object->serial_number, "");
  EXPECT_EQ(object->vendor, "Bio-Rad");

  // Transform component validation
  auto* transform = db.get<components::TransformComponent>(eid);
  EXPECT_EQ(transform->elements.size(), 263);

  // Link component validation
  auto* link = db.get<components::LinkComponent>(eid);
  EXPECT_EQ(link->elements.size(), 3);

  // Joint component validation
  auto* joint = db.get<components::JointComponent>(eid);
  EXPECT_EQ(joint->elements.size(), 2);

  // Button component validation
  auto* button = db.get<components::VirtualButtonComponent>(eid);
  EXPECT_EQ(button->elements.size(), 51);

  // system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok" });
  // system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok", "zero" });
}

TEST(DATABASE, ParseSceneA)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/scene-a.xml";

  xml::EntityParser parser;
  database::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(filename, db));

  // Collect (object id -> entity) using the façade type
  std::vector<std::pair<std::string, database::Database::entity_type>> ids;
  db.each([&](database::Database::entity_type id, const components::ObjectComponent& object) {
    ids.emplace_back(object.id, id);
  });

  ASSERT_FALSE(ids.empty());

  {
    auto it = std::find_if(ids.begin(), ids.end(), [](const auto& pair) { return pair.first == "pcr1"; });
    ASSERT_NE(it, ids.end()) << "Entity with id='pcr1' not found";

    const auto& sid = it->first;
    (void)sid;
    const auto eid = it->second;

    // Presence checks via façade
    ASSERT_TRUE(db.has<components::ObjectComponent>(eid));
    ASSERT_TRUE(db.has<components::TransformComponent>(eid));
    ASSERT_TRUE(db.has<components::LinkComponent>(eid));
    ASSERT_TRUE(db.has<components::JointComponent>(eid));
    ASSERT_TRUE(db.has<components::TouchscreenComponent>(eid));
    ASSERT_FALSE(db.has<components::FSMComponent>(eid));
    ASSERT_TRUE(db.has<components::ActionMapComponent>(eid));
    ASSERT_TRUE(db.has<components::OriginComponent>(eid));
    ASSERT_TRUE(db.has<components::VirtualButtonComponent>(eid));

    // Object component validation
    {
      auto* object = db.get<components::ObjectComponent>(eid);
      ASSERT_NE(object, nullptr);
      EXPECT_EQ(object->id, "pcr1");
      EXPECT_EQ(object->name, "T100 Thermal Cycler");
      EXPECT_EQ(object->model, "T100");
      EXPECT_EQ(object->serial_number, "");
      EXPECT_EQ(object->vendor, "Bio-Rad");
    }

    // Transform component validation
    {
      auto* transform = db.get<components::TransformComponent>(eid);
      ASSERT_NE(transform, nullptr);
      EXPECT_EQ(transform->elements.size(), 257u);
    }

    // Link component validation
    {
      auto* link = db.get<components::LinkComponent>(eid);
      ASSERT_NE(link, nullptr);
      EXPECT_EQ(link->elements.size(), 2u);
    }

    // Joint component validation
    {
      auto* joint = db.get<components::JointComponent>(eid);
      ASSERT_NE(joint, nullptr);
      EXPECT_EQ(joint->elements.size(), 2u);
    }
  }
}

TEST(Database, ParseSceneB)
{
  std::string filename = std::string(SODF_TEST_FOLDER) + "/xml/scene-b.xml";

  xml::EntityParser parser;
  database::Database db;
  ASSERT_TRUE(parser.loadEntitiesFromFile(filename, db));

  // Collect ids
  std::vector<std::pair<std::string, database::Database::entity_type>> ids;
  db.each([&](database::Database::entity_type id, const components::ObjectComponent& object) {
    ids.emplace_back(object.id, id);
  });

  ASSERT_FALSE(ids.empty());

  {
    const auto& sid = ids[0].first;
    (void)sid;
    const auto eid = ids[0].second;

    ASSERT_TRUE(db.has<components::ObjectComponent>(eid));
    ASSERT_TRUE(db.has<components::TransformComponent>(eid));
    ASSERT_TRUE(db.has<components::LinkComponent>(eid));
    ASSERT_TRUE(db.has<components::JointComponent>(eid));
    ASSERT_TRUE(db.has<components::TouchscreenComponent>(eid));
    ASSERT_FALSE(db.has<components::FSMComponent>(eid));
    ASSERT_TRUE(db.has<components::ActionMapComponent>(eid));
    ASSERT_TRUE(db.has<components::OriginComponent>(eid));

    // Object
    {
      auto* object = db.get<components::ObjectComponent>(eid);
      ASSERT_NE(object, nullptr);
      EXPECT_EQ(object->id, "pcr3");
      EXPECT_EQ(object->name, "T100 Thermal Cycler");
      EXPECT_EQ(object->model, "T100");
      EXPECT_EQ(object->serial_number, "");
      EXPECT_EQ(object->vendor, "Bio-Rad");
    }

    // Transform
    {
      auto* transform = db.get<components::TransformComponent>(eid);
      ASSERT_NE(transform, nullptr);
      EXPECT_EQ(transform->elements.size(), 257u);
    }

    // Link
    {
      auto* link = db.get<components::LinkComponent>(eid);
      ASSERT_NE(link, nullptr);
      EXPECT_EQ(link->elements.size(), 2u);
    }

    // Joint
    {
      auto* joint = db.get<components::JointComponent>(eid);
      ASSERT_NE(joint, nullptr);
      EXPECT_EQ(joint->elements.size(), 2u);
    }
  }
}
