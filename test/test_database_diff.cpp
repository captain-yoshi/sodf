#include <sodf/database/database.h>
#include <sodf/components/transform.h>
#include <sodf/components/object.h>
#include <sodf/systems/scene_graph.h>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

using namespace sodf;

namespace {

static geometry::TransformNode makeNode(const std::string& parent, const Eigen::Vector3d& t, bool is_static = true,
                                        bool dirty = false)
{
  geometry::TransformNode n;
  n.parent = parent;
  n.local = Eigen::Isometry3d::Identity();
  n.local.translation() = t;
  n.global = Eigen::Isometry3d::Identity();
  n.rest_local = n.local;
  n.is_static = is_static;
  n.dirty = dirty;
  return n;
}

}  // namespace

TEST(DatabaseDiff, ReadThroughAndOverrideTransformElement)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff diff(base);

  {
    const auto* a = diff.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }

  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a_override = *a_base;
    a_override.local.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

    diff.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override);

    const auto* a = diff.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

    const auto* a_check = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_check, nullptr);
    EXPECT_TRUE(a_check->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
}

TEST(DatabaseDiff, RemoveTransformElement)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameB", makeNode("root", Eigen::Vector3d(5.0, 0.0, 0.0)));

  database::DatabaseDiff diff(base);

  ASSERT_NE(diff.get_element<components::TransformComponent>(e, "frameB"), nullptr);

  diff.remove<components::TransformComponent>(e, std::string("frameB"));

  EXPECT_EQ(diff.get_element<components::TransformComponent>(e, "frameB"), nullptr);
  EXPECT_NE(base.get_element<components::TransformComponent>(e, "frameB"), nullptr);
}

TEST(DatabaseDiff, AddNewTransformElementOnlyInDiff)
{
  database::Database base;

  auto e = base.create();
  base.add<components::TransformComponent>(e);

  database::DatabaseDiff diff(base);

  EXPECT_EQ(base.get_element<components::TransformComponent>(e, "virtual"), nullptr);

  diff.add_or_replace<components::TransformComponent>(e, std::string("virtual"),
                                                      makeNode("root", Eigen::Vector3d(0.1, 0.2, 0.3)));

  {
    const auto* v = diff.get_element<components::TransformComponent>(e, "virtual");
    ASSERT_NE(v, nullptr);
    EXPECT_TRUE(v->local.translation().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  }

  EXPECT_EQ(base.get_element<components::TransformComponent>(e, "virtual"), nullptr);
}

TEST(DatabaseDiff, RootReadThroughStillWorks)
{
  database::Database base;

  auto e = base.create();
  base.add<components::TransformComponent>(e);

  database::DatabaseDiff diff(base);

  const auto* root = diff.get_element<components::TransformComponent>(e, "root");
  ASSERT_NE(root, nullptr);
  EXPECT_TRUE(root->local.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(DatabaseDiff, ClearResetsToBaseView)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameC", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff diff(base);

  auto overrideC = makeNode("root", Eigen::Vector3d(9.0, 9.0, 9.0));
  diff.add_or_replace<components::TransformComponent>(e, std::string("frameC"), overrideC);

  ASSERT_FALSE(diff.empty());

  {
    const auto* c = diff.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c, nullptr);
    EXPECT_TRUE(c->local.translation().isApprox(Eigen::Vector3d(9.0, 9.0, 9.0)));
  }

  diff.clear();
  EXPECT_TRUE(diff.empty());

  {
    const auto* c = diff.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c, nullptr);
    EXPECT_TRUE(c->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
}

// -----------------------------------------------------------------------------
// NEW: diff layered over another diff
// -----------------------------------------------------------------------------

TEST(DatabaseDiff, LayeredDiffOverridesParentDiff)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  // Override in d1
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a1 = *a_base;
    a1.local.translation() = Eigen::Vector3d(1.0, 1.0, 1.0);
    d1.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a1);
  }

  database::DatabaseDiff d2(d1);

  // Override again in d2
  {
    const auto* a_d1 = d1.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_d1, nullptr);

    auto a2 = *a_d1;
    a2.local.translation() = Eigen::Vector3d(2.0, 2.0, 2.0);
    d2.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a2);
  }

  // Check each layer
  {
    const auto* a0 = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a0, nullptr);
    EXPECT_TRUE(a0->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
  {
    const auto* a1 = d1.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a1, nullptr);
    EXPECT_TRUE(a1->local.translation().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
  }
  {
    const auto* a2 = d2.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a2, nullptr);
    EXPECT_TRUE(a2->local.translation().isApprox(Eigen::Vector3d(2.0, 2.0, 2.0)));
  }
}

TEST(DatabaseDiff, LayeredDiffRemoveBeatsParentAdd)
{
  database::Database base;

  auto e = base.create();
  base.add<components::TransformComponent>(e);

  database::DatabaseDiff d1(base);
  d1.add_or_replace<components::TransformComponent>(e, std::string("virtual"),
                                                    makeNode("root", Eigen::Vector3d(0.5, 0.0, 0.0)));

  database::DatabaseDiff d2(d1);
  d2.remove<components::TransformComponent>(e, std::string("virtual"));

  EXPECT_NE(d1.get_element<components::TransformComponent>(e, "virtual"), nullptr);
  EXPECT_EQ(d2.get_element<components::TransformComponent>(e, "virtual"), nullptr);
}

TEST(SceneGraphDiff, RootGlobalReflectsDiffOverride)
{
  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  {
    const auto* root_base = base.get_element<components::TransformComponent>(e, "root");
    ASSERT_NE(root_base, nullptr);

    auto root_override = *root_base;
    root_override.local = Eigen::Isometry3d::Identity();
    root_override.local.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    root_override.rest_local = root_override.local;
    root_override.dirty = true;

    d1.add_or_replace<components::TransformComponent>(e, std::string("root"), root_override);
  }

  database::DatabaseDiff d2(d1);

  auto map = systems::make_object_entity_map(d2);

  systems::SceneGraphCache cache(d2);

  systems::update_entity_global_transforms(cache, e, map);
  Eigen::Isometry3d W = systems::get_root_global_transform(cache, map, "obj");

  EXPECT_TRUE(W.translation().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(SceneGraphDiff, ChildGlobalReflectsDiffOverride)
{
  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj_child_override";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  // Override frameA in d1
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a_override = *a_base;
    a_override.local = Eigen::Isometry3d::Identity();
    a_override.local.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
    a_override.rest_local = a_override.local;
    a_override.dirty = true;

    d1.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override);
  }

  database::DatabaseDiff d2(d1);

  auto map = systems::make_object_entity_map(d2);
  systems::SceneGraphCache cache(d2);

  systems::update_entity_global_transforms(cache, e, map);

  Eigen::Isometry3d A = systems::get_global_transform(cache, e, "frameA");
  EXPECT_TRUE(A.translation().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
}

TEST(SceneGraphDiff, ParentOverridePropagatesToChild)
{
  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj_parent_propagates";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));
  tc.elements.emplace_back("frameB", makeNode("frameA", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  // Override frameA in d1 (parent)
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a_override = *a_base;
    a_override.local = Eigen::Isometry3d::Identity();
    a_override.local.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    a_override.rest_local = a_override.local;
    a_override.dirty = true;

    d1.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override);
  }

  database::DatabaseDiff d2(d1);

  auto map = systems::make_object_entity_map(d2);
  systems::SceneGraphCache cache(d2);

  systems::update_entity_global_transforms(cache, e, map);

  Eigen::Isometry3d B = systems::get_global_transform(cache, e, "frameB");

  // frameB local is zero; should inherit parent offset
  EXPECT_TRUE(B.translation().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(SceneGraphDiff, LayeredDiffChildOverrideWins)
{
  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj_layered_child";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  // Override in d1
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a1 = *a_base;
    a1.local = Eigen::Isometry3d::Identity();
    a1.local.translation() = Eigen::Vector3d(1.0, 1.0, 1.0);
    a1.rest_local = a1.local;
    a1.dirty = true;

    d1.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a1);
  }

  database::DatabaseDiff d2(d1);

  // Override again in d2
  {
    const auto* a_d1 = d1.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_d1, nullptr);

    auto a2 = *a_d1;
    a2.local = Eigen::Isometry3d::Identity();
    a2.local.translation() = Eigen::Vector3d(2.0, 2.0, 2.0);
    a2.rest_local = a2.local;
    a2.dirty = true;

    d2.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a2);
  }

  auto map = systems::make_object_entity_map(d2);
  systems::SceneGraphCache cache(d2);

  systems::update_entity_global_transforms(cache, e, map);

  Eigen::Isometry3d A = systems::get_global_transform(cache, e, "frameA");
  EXPECT_TRUE(A.translation().isApprox(Eigen::Vector3d(2.0, 2.0, 2.0)));
}

TEST(SceneGraphDiff, RemoveFrameDoesNotCrashSceneGraphPath)
{
  // Current contract:
  // - Removal is reflected in the DatabaseDiff public API.
  // - SceneGraphCache may still materialize base-ordered lists.
  // - Scene graph prefers per-element diff lookups at compute time.
  //
  // Because the internal "removed element" representation can differ
  // (nullptr vs tombstone-like behavior), we do not assert a specific
  // global translation here.
  //
  // We lock in the safety contract instead: no crash.

  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj_remove_frame";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.5, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  d1.remove<components::TransformComponent>(e, std::string("frameA"));

  // Public diff API reflects removal
  ASSERT_EQ(d1.get_element<components::TransformComponent>(e, "frameA"), nullptr);

  database::DatabaseDiff d2(d1);

  auto map = systems::make_object_entity_map(d2);
  systems::SceneGraphCache cache(d2);

  systems::update_entity_global_transforms(cache, e, map);

  EXPECT_NO_THROW({
    Eigen::Isometry3d A = systems::get_global_transform(cache, e, "frameA");
    (void)A;
  });
}

TEST(SceneGraphDiff, CacheInvalidatesOnRevisionChange)
{
  database::Database base;

  auto e = base.create();
  auto& obj = base.add<components::ObjectComponent>(e);
  obj.id = "obj_cache_revision";

  auto& tc = base.add<components::TransformComponent>(e);
  if (tc.elements.empty())
    tc.elements.emplace_back("root", geometry::TransformNode{});

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff d1(base);

  // First override in d1
  {
    const auto* root_base = base.get_element<components::TransformComponent>(e, "root");
    ASSERT_NE(root_base, nullptr);

    auto root1 = *root_base;
    root1.local = Eigen::Isometry3d::Identity();
    root1.local.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    root1.rest_local = root1.local;
    root1.dirty = true;

    d1.add_or_replace<components::TransformComponent>(e, std::string("root"), root1);
  }

  database::DatabaseDiff d2(d1);

  auto map = systems::make_object_entity_map(d2);
  systems::SceneGraphCache cache(d2);

  // Compute once
  systems::update_entity_global_transforms(cache, e, map);
  Eigen::Isometry3d W1 = systems::get_root_global_transform(cache, map, "obj_cache_revision");
  EXPECT_TRUE(W1.translation().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));

  // Second override directly on d2 (should bump revision)
  {
    const auto* root_d2 = d2.get_element<components::TransformComponent>(e, "root");
    ASSERT_NE(root_d2, nullptr);

    auto root2 = *root_d2;
    root2.local = Eigen::Isometry3d::Identity();
    root2.local.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
    root2.rest_local = root2.local;
    root2.dirty = true;

    d2.add_or_replace<components::TransformComponent>(e, std::string("root"), root2);
  }

  // Recompute using the same cache instance
  systems::update_entity_global_transforms(cache, e, map);
  Eigen::Isometry3d W2 = systems::get_root_global_transform(cache, map, "obj_cache_revision");
  EXPECT_TRUE(W2.translation().isApprox(Eigen::Vector3d(3.0, 0.0, 0.0)));
}
