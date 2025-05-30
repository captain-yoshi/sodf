#include <cmath>
#include <sodf/fluid/fluid_domain_shape.h>

#include <gtest/gtest.h>

using namespace sodf::fluid;

constexpr double VOLUME_EPSILON = 1e-12;
constexpr double HEIGHT_EPSILON = 1e-09;

TEST(BoxShape, VolumeAndHeight)
{
  // BoxShape(width, length, height)
  BoxShape shape(0.5, 0.3, 0.8);

  // getVolume (by height)
  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));
  EXPECT_EQ(0.0, shape.getVolume(0.9));
  EXPECT_NEAR(0.06, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.12, shape.getVolume(0.8), VOLUME_EPSILON);

  // getHeight (by volume)
  EXPECT_EQ(0.0, shape.getHeight(-0.5));
  EXPECT_EQ(0.0, shape.getHeight(0.0));
  EXPECT_EQ(0.0, shape.getHeight(0.121));
  EXPECT_NEAR(0.4, shape.getHeight(0.06), HEIGHT_EPSILON);
  EXPECT_NEAR(0.8, shape.getHeight(0.12), HEIGHT_EPSILON);
}

TEST(CylinderShape, VolumeAndHeight)
{
  CylinderShape shape(0.5, 0.8);

  // getVolume (by height)
  EXPECT_EQ(0.0, shape.getVolume(-0.5));
  EXPECT_EQ(0.0, shape.getVolume(0.0));
  EXPECT_EQ(0.0, shape.getVolume(0.9));
  EXPECT_NEAR(0.31415926535897931, shape.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.62831853071795862, shape.getVolume(0.8), VOLUME_EPSILON);

  // getHeight (by volume)
  EXPECT_EQ(0.0, shape.getHeight(-0.5));
  EXPECT_EQ(0.0, shape.getHeight(0.0));
  EXPECT_EQ(0.0, shape.getHeight(0.628319));
  EXPECT_NEAR(0.4, shape.getHeight(0.31415926535897931), HEIGHT_EPSILON);
  EXPECT_NEAR(0.8, shape.getHeight(0.6283185307179586), HEIGHT_EPSILON);
}

TEST(ConeShape, VolumeAndHeight)
{
  // base_radius, top_radius, height
  ConeShape shape1(0.5, 0.3, 0.8);

  // getVolume (by height)
  EXPECT_EQ(0.0, shape1.getVolume(-0.5));
  EXPECT_EQ(0.0, shape1.getVolume(0.0));
  EXPECT_EQ(0.0, shape1.getVolume(0.9));
  EXPECT_NEAR(0.25551620249196988, shape1.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.4105014400690663, shape1.getVolume(0.8), VOLUME_EPSILON);

  // getHeight (by volume)
  EXPECT_EQ(0.0, shape1.getHeight(-0.5));
  EXPECT_EQ(0.0, shape1.getHeight(0.0));
  EXPECT_EQ(0.0, shape1.getHeight(0.411));
  EXPECT_NEAR(0.4, shape1.getHeight(0.25551620249196988), HEIGHT_EPSILON);
  EXPECT_NEAR(0.8, shape1.getHeight(0.4105014400), HEIGHT_EPSILON);

  // Inverted cone: top_radius > base_radius
  ConeShape shape2(0.3, 0.5, 0.8);
  EXPECT_NEAR(0.4105014400690663 - 0.25551620249196988, shape2.getVolume(0.4), VOLUME_EPSILON);
  EXPECT_NEAR(0.4105014400690663, shape2.getVolume(0.8), VOLUME_EPSILON);
  EXPECT_NEAR(0.4, shape2.getHeight(0.4105014400 - 0.25551620249196988), HEIGHT_EPSILON);
  EXPECT_NEAR(0.8, shape2.getHeight(0.4105014400), HEIGHT_EPSILON);
}

TEST(SphericalSegmentShape, VolumeAndHeight)
{
  // SphericalCap bowl upright
  {
    SphericalSegmentShape shape(0.0, 5.0, 5.0);  // base_radius, top_radius, height

    // getVolume (by height)
    EXPECT_EQ(0.0, shape.getVolume(-0.5));
    EXPECT_EQ(0.0, shape.getVolume(0.0));
    EXPECT_EQ(0.0, shape.getVolume(5.0001));

    EXPECT_NEAR(22.498384888989406, shape.getVolume(1.25), VOLUME_EPSILON);
    EXPECT_NEAR(31.808625617596654, shape.getVolume(1.5), VOLUME_EPSILON);
    EXPECT_NEAR(81.81230868723421, shape.getVolume(2.5), VOLUME_EPSILON);
    EXPECT_NEAR(147.52395502482068, shape.getVolume(3.5), VOLUME_EPSILON);
    EXPECT_NEAR(165.66992509164925, shape.getVolume(3.75), VOLUME_EPSILON);
    EXPECT_NEAR(261.79938779914943, shape.getVolume(5.0), VOLUME_EPSILON);

    // getHeight (by volume)
    EXPECT_EQ(0.0, shape.getHeight(-0.5));
    EXPECT_EQ(0.0, shape.getHeight(0.0));
    EXPECT_EQ(0.0, shape.getHeight(262));
    EXPECT_NEAR(1.25, shape.getHeight(22.498384888989406), HEIGHT_EPSILON);
    EXPECT_NEAR(1.5, shape.getHeight(31.808625617596654), HEIGHT_EPSILON);
    EXPECT_NEAR(2.5, shape.getHeight(81.81230868723421), HEIGHT_EPSILON);
    EXPECT_NEAR(3.5, shape.getHeight(147.52395502482068), HEIGHT_EPSILON);
    EXPECT_NEAR(3.75, shape.getHeight(165.66992509164925), HEIGHT_EPSILON);
    EXPECT_NEAR(5.0, shape.getHeight(261.799387799149), HEIGHT_EPSILON);
  }

  // SphericalCap bowl downright
  {
    SphericalSegmentShape shape(5.0, 0.0, 5.0);  // base_radius, top_radius, height

    // getVolume (by height)
    EXPECT_EQ(0.0, shape.getVolume(-0.5));
    EXPECT_EQ(0.0, shape.getVolume(0.0));
    EXPECT_EQ(0.0, shape.getVolume(5.0001));

    EXPECT_NEAR(shape.volume() - 165.66992509164925, shape.getVolume(1.25), VOLUME_EPSILON);
    EXPECT_NEAR(shape.volume() - 147.52395502482068, shape.getVolume(1.5), VOLUME_EPSILON);
    EXPECT_NEAR(shape.volume() - 81.81230868723421, shape.getVolume(2.5), VOLUME_EPSILON);
    EXPECT_NEAR(shape.volume() - 31.808625617596654, shape.getVolume(3.5), VOLUME_EPSILON);
    EXPECT_NEAR(shape.volume() - 22.498384888989406, shape.getVolume(3.75), VOLUME_EPSILON);
    EXPECT_NEAR(261.799387799149, shape.getVolume(5.0), VOLUME_EPSILON);

    // getHeight (by volume)
    EXPECT_EQ(0.0, shape.getHeight(-0.5));
    EXPECT_EQ(0.0, shape.getHeight(0.0));
    EXPECT_EQ(0.0, shape.getHeight(262));
    EXPECT_NEAR(1.25, shape.getHeight(shape.volume() - 165.66992509164925), HEIGHT_EPSILON);
    EXPECT_NEAR(1.5, shape.getHeight(shape.volume() - 147.52395502482068), HEIGHT_EPSILON);
    EXPECT_NEAR(2.5, shape.getHeight(shape.volume() - 81.81230868723421), HEIGHT_EPSILON);
    EXPECT_NEAR(3.5, shape.getHeight(shape.volume() - 31.808625617596654), HEIGHT_EPSILON);
    EXPECT_NEAR(3.75, shape.getHeight(shape.volume() - 22.498384888989406), HEIGHT_EPSILON);

    // Function is flat near tip, ill-conditioned
    EXPECT_NEAR(5.0, shape.getHeight(261.799387799149429), 1e-05);
  }

  // Spherical segment (belt around equator of a sphere of radius 5, between y=-3 and y=+3)
  {
    double sphere_radius = 5.0;
    double y1 = -3.0;  // bottom plane (distance from center)
    double y2 = 3.0;   // top plane
    double a1 = std::sqrt(sphere_radius * sphere_radius - y1 * y1);
    double a2 = std::sqrt(sphere_radius * sphere_radius - y2 * y2);
    double H = y2 - y1;

    SphericalSegmentShape shape(a1, a2, H);

    // Analytical volume for this segment
    double expected_volume = M_PI * H / 6.0 * (3 * a1 * a1 + 3 * a2 * a2 + H * H);

    // Test total segment volume
    EXPECT_NEAR(expected_volume, shape.volume(), VOLUME_EPSILON);

    // Test getVolume at partial heights (e.g., 0.5*H)
    double partial_height = 0.5 * H;
    double v_partial = shape.getVolume(partial_height);
    EXPECT_LT(v_partial, expected_volume);
    EXPECT_GT(v_partial, 0.0);

    // Test getHeight (invert)
    double h_from_volume = shape.getHeight(v_partial);
    EXPECT_NEAR(partial_height, h_from_volume, HEIGHT_EPSILON);
  }
}

TEST(FluidDomainShapeCollection, FluidVolumeAndHeight)
{
  // Build a stacked domain shape collection
  std::vector<DomainShapePtr> domains;
  domains.push_back(std::make_shared<SphericalSegmentShape>(0.0, 5.0, 5.0));
  domains.push_back(std::make_shared<ConeShape>(5.0, 7.0, 10.0));
  domains.push_back(std::make_shared<CylinderShape>(7.0, 15.0));
  domains.push_back(std::make_shared<BoxShape>(7.0, 7.0, 20.0));

  // Compute total stacked height and volume
  double total_height = 0.0;
  for (const auto& domain : domains)
    total_height += domain->height();

  double total_volume = 0.0;
  for (const auto& domain : domains)
    total_volume += domain->volume();

  // Check that the total volume by stacking equals the direct total
  EXPECT_NEAR(total_volume, getFluidVolume(domains, total_height, VOLUME_EPSILON), VOLUME_EPSILON);

  // Stepwise checks (partial fill scenarios)
  EXPECT_NEAR(domains[0]->volume() + domains[1]->getVolume(5.0), getFluidVolume(domains, 5.0 + 5.0, VOLUME_EPSILON),
              VOLUME_EPSILON);

  EXPECT_NEAR(domains[0]->volume() + domains[1]->volume() + domains[2]->getVolume(3.0),
              getFluidVolume(domains, 5.0 + 10.0 + 3.0, VOLUME_EPSILON), VOLUME_EPSILON);

  EXPECT_NEAR(domains[0]->volume() + domains[1]->volume() + domains[2]->volume() + domains[3]->getVolume(18.0),
              getFluidVolume(domains, 5.0 + 10.0 + 15.0 + 18.0, VOLUME_EPSILON), VOLUME_EPSILON);

  // Inverse: total stacked height by volume
  EXPECT_NEAR(total_height, getFluidHeight(domains, total_volume, HEIGHT_EPSILON), VOLUME_EPSILON);

  EXPECT_NEAR(5.0 + 5.0, getFluidHeight(domains, domains[0]->volume() + domains[1]->getVolume(5.0), HEIGHT_EPSILON),
              HEIGHT_EPSILON);

  EXPECT_NEAR(5.0 + 10.0 + 3.0,
              getFluidHeight(domains, domains[0]->volume() + domains[1]->volume() + domains[2]->getVolume(3.0),
                             HEIGHT_EPSILON),
              HEIGHT_EPSILON);

  EXPECT_NEAR(
      5.0 + 10.0 + 15.0 + 18.0,
      getFluidHeight(domains,
                     domains[0]->volume() + domains[1]->volume() + domains[2]->volume() + domains[3]->getVolume(18.0),
                     HEIGHT_EPSILON),
      HEIGHT_EPSILON);
}
