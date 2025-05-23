
#include <sodf/ecs.h>
#include <sodf/xml_parser.h>

#include <sodf/components/object.h>
#include <sodf/components/transform.h>
#include <sodf/components/joint.h>
#include <sodf/components/link.h>
#include <sodf/components/touchscreen.h>
#include <sodf/components/finite_state_machine.h>

#include <sodf/systems/fsm.h>

#include <gtest/gtest.h>

using namespace sodf;

TEST(ECS, ParsingSingleObject)
{
  auto db = ginseng::database{};

  XMLParser parser;
  std::string test_dir = SODF_TEST_FOLDER;
  std::string base_dir = test_dir + "/../database/biorad";
  std::string filename = "thermocycler_t100.xml";  // wrt. the base dir
  // path += "/../database/biorad/thermocycler_t100.xml";
  parser.loadEntities(filename, base_dir, db);

  // parser.parseComponents(db);

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

  // Object component validation
  auto& object = db.get_component<components::ObjectComponent>(eid);
  EXPECT_EQ(object.id, "biorad_thermal_cycler_t100");
  EXPECT_EQ(object.name, "Thermal Cycler");
  EXPECT_EQ(object.model, "T100");
  EXPECT_EQ(object.serial_number, "SN-ABC123456");
  EXPECT_EQ(object.vendor, "BioRad");

  // Transform component validation
  auto& transform = db.get_component<components::TransformComponent>(eid);
  EXPECT_EQ(transform.transform_map.size(), 60);

  // Link component validation
  auto& link = db.get_component<components::LinkComponent>(eid);
  EXPECT_EQ(link.link_map.size(), 3);

  // Joint component validation
  auto& joint = db.get_component<components::JointComponent>(eid);
  EXPECT_EQ(joint.joint_map.size(), 2);

  // db.visit([](components::FSMComponent& fsm_comp) {
  //   for (auto& [fsm_id, fsm] : fsm_comp.fsm_map)
  //   {
  //     const std::string action = "incubate";  // example

  //     std::cout << "FSM current state: " << fsm.state_labels.to_string(fsm.current_state) << "\n";
  //     std::cout << "Has 'incubate'? " << fsm.action_labels.has_label("incubate") << "\n";

  //     if (!fsm.action_labels.has_label(action))
  //       continue;

  //     int action_id = fsm.action_labels.to_id(action);
  //     int current = fsm.current_state;

  //     if (current < fsm.transitions.size() && action_id < fsm.transitions[current].size())
  //     {
  //       int next = fsm.transitions[current][action_id];
  //       if (next >= 0)
  //       {
  //         std::cout << "FSM " << fsm_id << ": " << current << " → " << next << "\n";
  //         fsm.current_state = next;
  //       }
  //     }
  //   }
  // });

  // system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok" });
  system::simulate_action_sequence_on_all(db, "fsm/touchscreen", { "incubate", "bloc_temperature", "ok", "zero" });
}

// TEST(ECS, Entity)
// {
// auto db = ginseng::database{};

// // entity
// auto pcr = db.create_entity();

// // component: id
// db.add_component(pcr, components::ID{ "pcr" });

// // component: constraint
// db.add_component(pcr, components::Constraint{ "root", "root" });

// // component: joint
// {
//   components::JointCollection joints;

//   joints.joint_map.emplace_back("joint1", components::Joint{ KDL::Joint(KDL::Joint::RotX), "root", 0.0 });

//   db.add_component(pcr, joints);
// }

// // component: relative transforms
// {
//   std::vector<geometry::Transform> transforms;

//   transforms.push_back(geometry::Transform{ "root", "container/A1" });

//   db.add_component(pcr, components::RelativeTransforms{ transforms });
// }

// // component: container
// {
//   auto containers = components::ContainerCollection{};
//   std::vector<geometry::BaseVolumePtr> shape;
//   shape.emplace_back(std::make_shared<geometry::SphericalCapVolume>(0.00142, 0.00167));
//   shape.emplace_back(std::make_shared<geometry::TruncatedConeVolume>(0.00142, 0.00256, 0.00765));
//   shape.emplace_back(std::make_shared<geometry::TruncatedConeVolume>(0.00256, 0.00275, 0.00478));
//   shape.emplace_back(std::make_shared<geometry::CylinderVolume>(0.00275, 0.0040));

//   auto container = components::Container{ 0.0, shape };
//   containers.container_map.emplace_back("container/A1", container);

//   db.add_component(pcr, containers);
// }

// // system: scene graph
// systems::SceneGraph scene_graph(db);

// scene_graph.displayInObjectRoot("pcr", "container/A1");

// // test diffs
// auto db1 = ginseng::database{};
// auto pcr2 = db1.create_entity();

// db1.add_component(pcr2, pcr);

// auto& jc_component = db.get_component<components::JointCollection>(pcr);

// auto new_component = jc_component;
// new_component.joint_map[0].second.joint_position = M_PI;

// db1.add_component(pcr2, new_component);

// ASSERT_EQ(jc_component.joint_map[0].second.joint_position, 0.0);

// db.add_component(pcr, new_component);

// ASSERT_EQ(jc_component.joint_map[0].second.joint_position, M_PI);
// }

TEST(ECS, ParseScene1)
{
  auto db = ginseng::database{};

  XMLParser parser;
  std::string test_dir = SODF_TEST_FOLDER;
  std::string base_dir = test_dir + "/../database";
  std::string filename = "scene.xml";  // wrt. the base dir
  // path += "/../database/biorad/thermocycler_t100.xml";
  parser.loadEntities(filename, base_dir, db);

  // parser.parseComponents(db);

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
    ASSERT_TRUE(db.has_component<components::AlignGeometricPairComponent>(eid));

    // Object component validation
    auto& object = db.get_component<components::ObjectComponent>(eid);
    EXPECT_EQ(object.id, "pcr1");
    EXPECT_EQ(object.name, "Thermal Cycler");
    EXPECT_EQ(object.model, "T100");
    EXPECT_EQ(object.serial_number, "SN-ABC123456");
    EXPECT_EQ(object.vendor, "BioRad");

    // Transform component validation
    auto& transform = db.get_component<components::TransformComponent>(eid);
    EXPECT_EQ(transform.transform_map.size(), 58);

    // Link component validation
    auto& link = db.get_component<components::LinkComponent>(eid);
    EXPECT_EQ(link.link_map.size(), 2);

    // Joint component validation
    auto& joint = db.get_component<components::JointComponent>(eid);
    EXPECT_EQ(joint.joint_map.size(), 2);
  }
}
TEST(ECS, ParseScene2)
{
  auto db = ginseng::database{};

  XMLParser parser;
  std::string test_dir = SODF_TEST_FOLDER;
  std::string base_dir = test_dir + "/../database";
  std::string filename = "scene2.xml";  // wrt. the base dir
  // path += "/../database/biorad/thermocycler_t100.xml";
  parser.loadEntities(filename, base_dir, db);

  // parser.parseComponents(db);

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
    ASSERT_TRUE(db.has_component<components::AlignGeometricPairComponent>(eid));

    // Object component validation
    auto& object = db.get_component<components::ObjectComponent>(eid);
    EXPECT_EQ(object.id, "pcr2");
    EXPECT_EQ(object.name, "Thermal Cycler");
    EXPECT_EQ(object.model, "T100");
    EXPECT_EQ(object.serial_number, "SN-ABC123456");
    EXPECT_EQ(object.vendor, "BioRad");

    // Transform component validation
    auto& transform = db.get_component<components::TransformComponent>(eid);
    EXPECT_EQ(transform.transform_map.size(), 58);

    // Link component validation
    auto& link = db.get_component<components::LinkComponent>(eid);
    EXPECT_EQ(link.link_map.size(), 2);

    // Joint component validation
    auto& joint = db.get_component<components::JointComponent>(eid);
    EXPECT_EQ(joint.joint_map.size(), 2);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
