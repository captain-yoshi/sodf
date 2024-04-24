#include <sodf/volume.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

constexpr double VOLUME_EPSILON = 1e-09;

TEST(RectangularPrismVolume, getVolume)
{
  RectangularPrismVolume shape(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));

  EXPECT_NEAR(0.06, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.12, shape.getVolume(0.8), VOLUME_EPSILON);

  EXPECT_EQ(0.0, shape.getVolume(0.9));
}

TEST(SphericalCapVolume, getVolume)
{
  SphericalCapVolume shape(5.0, 5.0);

  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));

  EXPECT_NEAR(106.356001293, shape.getVolume(2.5), VOLUME_EPSILON);
  EXPECT_NEAR(159.893976098, shape.getVolume(3.5), VOLUME_EPSILON);
  EXPECT_NEAR(261.799387799, shape.getVolume(5.0), VOLUME_EPSILON);

  EXPECT_EQ(0.0, shape.getVolume(5.0001));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
