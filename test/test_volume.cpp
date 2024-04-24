#include <sodf/volume.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

constexpr double VOLUME_EPSILON = 4 * std::numeric_limits<double>::epsilon();

TEST(RectangularPrismVolume, getVolume)
{
  RectangularPrismVolume shape(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));

  EXPECT_NEAR(0.06, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.12, shape.getVolume(0.8), VOLUME_EPSILON);

  EXPECT_EQ(0.0, shape.getVolume(0.9));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
