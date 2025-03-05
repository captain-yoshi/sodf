#include <sodf/components/container.h>
#include <sodf/components/constraint.h>
#include <sodf/components/id.h>
#include <sodf/components/joint.h>
#include <sodf/geometry/volume.h>

#include <sodf/systems/scene_graph.h>

#include <ginseng.hpp>

#include <gtest/gtest.h>

using namespace sodf;

TEST(ECS, Entity)
{
  auto db = ginseng::database{};

  // entity
  auto pcr = db.create_entity();

  // component: id
  db.add_component(pcr, components::ID{ "pcr" });

  // component: constraint
  db.add_component(pcr, components::Constraint{ "root", "root" });

  // component: joint
  {
    components::JointCollection joints;

    joints.joint_map.emplace_back("joint1", components::Joint{ KDL::Joint(KDL::Joint::RotX), "root", 0.0 });

    db.add_component(pcr, joints);
  }

  // component: relative transforms
  {
    std::vector<geometry::Transform> transforms;

    transforms.push_back(geometry::Transform{ "root", "container/A1" });

    db.add_component(pcr, components::RelativeTransforms{ transforms });
  }

  // component: container
  {
    auto containers = components::ContainerCollection{};
    std::vector<geometry::BaseVolumePtr> shape;
    shape.emplace_back(std::make_shared<geometry::SphericalCapVolume>(0.00142, 0.00167));
    shape.emplace_back(std::make_shared<geometry::TruncatedConeVolume>(0.00142, 0.00256, 0.00765));
    shape.emplace_back(std::make_shared<geometry::TruncatedConeVolume>(0.00256, 0.00275, 0.00478));
    shape.emplace_back(std::make_shared<geometry::CylinderVolume>(0.00275, 0.0040));

    auto container = components::Container{ 0.0, shape };
    containers.container_map.emplace_back("container/A1", container);

    db.add_component(pcr, containers);
  }

  // system: scene graph
  systems::SceneGraph scene_graph(db);

  scene_graph.displayInObjectRoot("pcr", "container/A1");

  // test diffs
  auto db1 = ginseng::database{};
  auto pcr2 = db1.create_entity();

  db1.add_component(pcr2, pcr);

  auto& jc_component = db.get_component<components::JointCollection>(pcr);

  auto new_component = jc_component;
  new_component.joint_map[0].second.joint_position = M_PI;

  db1.add_component(pcr2, new_component);

  ASSERT_EQ(jc_component.joint_map[0].second.joint_position, 0.0);

  db.add_component(pcr, new_component);

  ASSERT_EQ(jc_component.joint_map[0].second.joint_position, M_PI);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
