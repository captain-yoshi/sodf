#include <sodf/database/database.h>
#include <sodf/components/transform.h>

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
  n.global = Eigen::Isometry3d::Identity();  // not computed here
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

  // TransformComponent constructor seeds "root".
  // Add one authored frame.
  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  // Create diff overlay
  database::DatabaseDiff diff(base);

  // Read-through: should see base data
  {
    const auto* a = diff.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }

  // Override in diff
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a_override = *a_base;
    a_override.local.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

    diff.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override);

    const auto* a = diff.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

    // Base remains unchanged
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

  // Sanity base visible
  ASSERT_NE(diff.get_element<components::TransformComponent>(e, "frameB"), nullptr);

  // Remove in diff
  diff.remove<components::TransformComponent>(e, std::string("frameB"));

  // Diff should hide it
  EXPECT_EQ(diff.get_element<components::TransformComponent>(e, "frameB"), nullptr);

  // Base still has it
  EXPECT_NE(base.get_element<components::TransformComponent>(e, "frameB"), nullptr);
}

TEST(DatabaseDiff, AddNewTransformElementOnlyInDiff)
{
  database::Database base;

  auto e = base.create();
  base.add<components::TransformComponent>(e);

  database::DatabaseDiff diff(base);

  // Base doesn't have "virtual"
  EXPECT_EQ(base.get_element<components::TransformComponent>(e, "virtual"), nullptr);

  // Add only in patch
  diff.add_or_replace<components::TransformComponent>(e, std::string("virtual"),
                                                      makeNode("root", Eigen::Vector3d(0.1, 0.2, 0.3)));

  // Diff sees it
  {
    const auto* v = diff.get_element<components::TransformComponent>(e, "virtual");
    ASSERT_NE(v, nullptr);
    EXPECT_TRUE(v->local.translation().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  }

  // Base still doesn't
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

  // Override
  auto overrideC = makeNode("root", Eigen::Vector3d(9.0, 9.0, 9.0));
  diff.add_or_replace<components::TransformComponent>(e, std::string("frameC"), overrideC);

  ASSERT_FALSE(diff.empty());

  {
    const auto* c = diff.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c, nullptr);
    EXPECT_TRUE(c->local.translation().isApprox(Eigen::Vector3d(9.0, 9.0, 9.0)));
  }

  // Clear patch
  diff.clear();
  EXPECT_TRUE(diff.empty());

  // Back to base
  {
    const auto* c = diff.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c, nullptr);
    EXPECT_TRUE(c->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
}

TEST(DatabaseDiff, LayeredDiffReadThroughAndOverride)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);

  tc.elements.emplace_back("frameA", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  // First diff over base
  database::DatabaseDiff diff1(base);

  // Override in diff1 to (1,2,3)
  {
    const auto* a_base = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_base, nullptr);

    auto a_override = *a_base;
    a_override.local.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

    diff1.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override);
  }

  // Second diff over diff1
  database::DatabaseDiff diff2(diff1);

  // Should read-through diff1's override
  {
    const auto* a = diff2.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  }

  // Override again in diff2 to (7,8,9)
  {
    const auto* a_d1 = diff1.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a_d1, nullptr);

    auto a_override2 = *a_d1;
    a_override2.local.translation() = Eigen::Vector3d(7.0, 8.0, 9.0);

    diff2.add_or_replace<components::TransformComponent>(e, std::string("frameA"), a_override2);
  }

  // diff2 sees latest override
  {
    const auto* a = diff2.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(7.0, 8.0, 9.0)));
  }

  // diff1 remains unchanged
  {
    const auto* a = diff1.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  }

  // base remains unchanged
  {
    const auto* a = base.get_element<components::TransformComponent>(e, "frameA");
    ASSERT_NE(a, nullptr);
    EXPECT_TRUE(a->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
}

TEST(DatabaseDiff, LayeredDiffRemoveHidesLowerOverride)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameB", makeNode("root", Eigen::Vector3d(5.0, 0.0, 0.0)));

  database::DatabaseDiff diff1(base);

  // Override frameB in diff1
  {
    const auto* b_base = base.get_element<components::TransformComponent>(e, "frameB");
    ASSERT_NE(b_base, nullptr);

    auto b_override = *b_base;
    b_override.local.translation() = Eigen::Vector3d(6.0, 0.0, 0.0);

    diff1.add_or_replace<components::TransformComponent>(e, std::string("frameB"), b_override);
  }

  // Create diff2 over diff1 and remove frameB there
  database::DatabaseDiff diff2(diff1);
  diff2.remove<components::TransformComponent>(e, std::string("frameB"));

  // diff2 should hide it entirely
  EXPECT_EQ(diff2.get_element<components::TransformComponent>(e, "frameB"), nullptr);

  // diff1 still has its override
  {
    const auto* b = diff1.get_element<components::TransformComponent>(e, "frameB");
    ASSERT_NE(b, nullptr);
    EXPECT_TRUE(b->local.translation().isApprox(Eigen::Vector3d(6.0, 0.0, 0.0)));
  }

  // base still has original
  {
    const auto* b = base.get_element<components::TransformComponent>(e, "frameB");
    ASSERT_NE(b, nullptr);
    EXPECT_TRUE(b->local.translation().isApprox(Eigen::Vector3d(5.0, 0.0, 0.0)));
  }
}

TEST(DatabaseDiff, LayeredDiffAddNewElementAtLowerThenOverrideAtUpper)
{
  database::Database base;

  auto e = base.create();
  base.add<components::TransformComponent>(e);

  database::DatabaseDiff diff1(base);

  // Add new element only in diff1
  diff1.add_or_replace<components::TransformComponent>(e, std::string("virtual"),
                                                       makeNode("root", Eigen::Vector3d(0.1, 0.2, 0.3)));

  // diff2 layered over diff1
  database::DatabaseDiff diff2(diff1);

  // Read-through sees diff1 addition
  {
    const auto* v = diff2.get_element<components::TransformComponent>(e, "virtual");
    ASSERT_NE(v, nullptr);
    EXPECT_TRUE(v->local.translation().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  }

  // Override "virtual" in diff2
  diff2.add_or_replace<components::TransformComponent>(e, std::string("virtual"),
                                                       makeNode("root", Eigen::Vector3d(9.1, 9.2, 9.3)));

  // diff2 sees its override
  {
    const auto* v = diff2.get_element<components::TransformComponent>(e, "virtual");
    ASSERT_NE(v, nullptr);
    EXPECT_TRUE(v->local.translation().isApprox(Eigen::Vector3d(9.1, 9.2, 9.3)));
  }

  // diff1 still sees its original addition
  {
    const auto* v = diff1.get_element<components::TransformComponent>(e, "virtual");
    ASSERT_NE(v, nullptr);
    EXPECT_TRUE(v->local.translation().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  }

  // base doesn't see it
  EXPECT_EQ(base.get_element<components::TransformComponent>(e, "virtual"), nullptr);
}

TEST(DatabaseDiff, LayeredDiffClearUpperRevealsLowerState)
{
  database::Database base;

  auto e = base.create();
  auto& tc = base.add<components::TransformComponent>(e);
  tc.elements.emplace_back("frameC", makeNode("root", Eigen::Vector3d(0.0, 0.0, 0.0)));

  database::DatabaseDiff diff1(base);
  diff1.add_or_replace<components::TransformComponent>(e, std::string("frameC"),
                                                       makeNode("root", Eigen::Vector3d(1.0, 1.0, 1.0)));

  database::DatabaseDiff diff2(diff1);
  diff2.add_or_replace<components::TransformComponent>(e, std::string("frameC"),
                                                       makeNode("root", Eigen::Vector3d(2.0, 2.0, 2.0)));

  // Sanity checks
  {
    const auto* c1 = diff1.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c1, nullptr);
    EXPECT_TRUE(c1->local.translation().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));

    const auto* c2 = diff2.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c2, nullptr);
    EXPECT_TRUE(c2->local.translation().isApprox(Eigen::Vector3d(2.0, 2.0, 2.0)));
  }

  // Clear only upper diff
  diff2.clear();

  // Should fall back to diff1's view
  {
    const auto* c2 = diff2.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c2, nullptr);
    EXPECT_TRUE(c2->local.translation().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
  }

  // Base still unchanged
  {
    const auto* c0 = base.get_element<components::TransformComponent>(e, "frameC");
    ASSERT_NE(c0, nullptr);
    EXPECT_TRUE(c0->local.translation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  }
}
