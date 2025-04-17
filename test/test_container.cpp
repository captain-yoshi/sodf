#include <sodf/elements/container.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;
using namespace sodf::elements;

constexpr double VOLUME_EPSILON = 1e-09;

TEST(RectangularPrismVolume, getVolume)
{
  Transform tf(KDL::Frame(), "pcr/container/A1", "root");

  std::vector<BaseVolumePtr> shape;
  shape.emplace_back(std::make_shared<SphericalCapVolume>(0.00142, 0.00167));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00142, 0.00256, 0.00765));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00256, 0.00275, 0.00478));
  shape.emplace_back(std::make_shared<CylinderVolume>(0.00275, 0.0040));

  Container c(shape, tf);

  EXPECT_EQ(0.0, c.getCurrentVolume());

  // EXPECT_NEAR(2.5, c.getMaxVolume(), VOLUME_EPSILON);
  // EXPECT_NEAR(2.5, c.getHeightFromAddingVolume(0.000000200), VOLUME_EPSILON);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
