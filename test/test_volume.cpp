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
  // Shape1 - Half-Sphere
  SphericalCapVolume shape1(5.0, 5.0);

  EXPECT_EQ(0.0, shape1.getVolume(-0.5));
  EXPECT_EQ(0.0, shape1.getVolume(0.0));
  EXPECT_EQ(0.0, shape1.getVolume(5.0001));

  EXPECT_NEAR(81.81230868723421, shape1.getVolume(2.5), VOLUME_EPSILON);
  EXPECT_NEAR(147.52395502482068, shape1.getVolume(3.5), VOLUME_EPSILON);
  EXPECT_NEAR(165.66992509164925, shape1.getVolume(3.75), VOLUME_EPSILON);
  EXPECT_NEAR(261.79938779914943, shape1.getVolume(5.0), VOLUME_EPSILON);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
