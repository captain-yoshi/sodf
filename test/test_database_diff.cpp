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
