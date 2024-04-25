#include <sodf/volume.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

constexpr double VOLUME_EPSILON = 1e-09;

TEST(RectangularPrismVolume, getVolume)
{
  RectangularPrismVolume shape(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));
  EXPECT_EQ(0.0, shape.getVolume(0.9));

  EXPECT_NEAR(0.06, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.12, shape.getVolume(0.8), VOLUME_EPSILON);
}

TEST(RectangularPrismVolume, getHeight)
{
  RectangularPrismVolume shape(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape.getHeight(-0.5));
  EXPECT_EQ(0.0, shape.getHeight(0.0));
  EXPECT_EQ(0.0, shape.getHeight(0.121));

  EXPECT_NEAR(0.4, shape.getHeight(0.06), VOLUME_EPSILON);
  EXPECT_NEAR(0.8, shape.getHeight(0.12), VOLUME_EPSILON);
}

TEST(CylinderVolume, getVolume)
{
  CylinderVolume shape(0.5, 0.8);

  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));
  EXPECT_EQ(0.0, shape.getVolume(0.9));

  EXPECT_NEAR(0.31415926535897931, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.62831853071795862, shape.getVolume(0.8), VOLUME_EPSILON);
}

TEST(CylinderVolume, getHeight)
{
  CylinderVolume shape(0.5, 0.8);

  EXPECT_EQ(0.0, shape.getHeight(-0.5));
  EXPECT_EQ(0.0, shape.getHeight(0.0));
  EXPECT_EQ(0.0, shape.getHeight(0.628319));

  EXPECT_NEAR(0.4, shape.getHeight(0.31415926535897931), VOLUME_EPSILON);
  EXPECT_NEAR(0.8, shape.getHeight(0.6283185307179586), VOLUME_EPSILON);
}

TEST(TruncatedConeVolume, getVolume)
{
  TruncatedConeVolume shape1(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape1.getVolume(-0.5));
  EXPECT_EQ(0.0, shape1.getVolume(0.0));
  EXPECT_EQ(0.0, shape1.getVolume(0.9));

  EXPECT_NEAR(0.25551620249196988, shape1.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.4105014400690663, shape1.getVolume(0.8), VOLUME_EPSILON);

  TruncatedConeVolume shape2(0.3, 0.5, 0.8);
  EXPECT_NEAR(0.4105014400690663 - 0.25551620249196988, shape2.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.4105014400690663, shape2.getVolume(0.8), VOLUME_EPSILON);
}

TEST(TruncatedConeVolume, getHeight)
{
  TruncatedConeVolume shape1(0.5, 0.3, 0.8);

  EXPECT_EQ(0.0, shape1.getHeight(-0.5));
  EXPECT_EQ(0.0, shape1.getHeight(0.0));
  EXPECT_EQ(0.0, shape1.getHeight(0.411));

  EXPECT_NEAR(0.4, shape1.getHeight(0.25551620249196988), VOLUME_EPSILON);
  EXPECT_NEAR(0.8, shape1.getHeight(0.4105014400), VOLUME_EPSILON);

  TruncatedConeVolume shape2(0.3, 0.5, 0.8);
  EXPECT_NEAR(0.4, shape2.getHeight(0.4105014400 - 0.25551620249196988), VOLUME_EPSILON);
  EXPECT_NEAR(0.8, shape2.getHeight(0.4105014400), VOLUME_EPSILON);
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

TEST(SphericalCapVolume, getHeight)
{
  // Shape1 - Half-Sphere
  SphericalCapVolume shape1(5.0, 5.0);

  EXPECT_EQ(0.0, shape1.getHeight(-0.5));
  EXPECT_EQ(0.0, shape1.getHeight(0.0));
  EXPECT_EQ(0.0, shape1.getHeight(262));

  EXPECT_NEAR(2.5, shape1.getHeight(81.81230868723421), VOLUME_EPSILON);
  EXPECT_NEAR(3.5, shape1.getHeight(147.52395502482068), VOLUME_EPSILON);
  EXPECT_NEAR(3.75, shape1.getHeight(165.66992509164925), VOLUME_EPSILON);
  EXPECT_NEAR(5.0, shape1.getHeight(261.799387799149), VOLUME_EPSILON);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
