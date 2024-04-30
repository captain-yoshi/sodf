#include <sodf/object.h>
#include <sodf/elements/container.h>

#include <gtest/gtest.h>

using namespace sodf;
using namespace sodf::geometry;
using namespace sodf::elements;

constexpr double VOLUME_EPSILON = 1e-09;

TEST(Object, addElement)
{
  // Element -> Container

  std::vector<BaseVolumePtr> shape;
  shape.emplace_back(std::make_shared<SphericalCapVolume>(0.00142, 0.00167));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00142, 0.00256, 0.00765));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00256, 0.00275, 0.00478));
  shape.emplace_back(std::make_shared<CylinderVolume>(0.00275, 0.0040));

  {
    Transform tf(KDL::Frame(), "pcr/container/A1", "root");

    auto container = std::make_unique<Container>(shape, tf);
    auto container_ptr = container.get();

    auto volume = container->addVolume(0.000000200);

    // Object
    Object obj;

    EXPECT_EQ(false, obj.removeElement("a"));
    EXPECT_EQ(false, obj.addElement("a", nullptr));
    EXPECT_EQ(true, obj.addElement("a", std::move(container)));
    EXPECT_EQ(false, obj.addElement("a", std::move(container)));
    EXPECT_EQ(container_ptr, obj.getElement<Element>("a"));
    EXPECT_EQ(container_ptr, obj.getElement<Element>("a"));
    EXPECT_EQ(container_ptr, obj.getElement<Container>("a"));
    EXPECT_EQ(container_ptr, obj.getElement<Container>("a"));
    EXPECT_EQ(volume, obj.getElement<Container>("a")->getCurrentVolume());

    EXPECT_EQ(true, obj.removeElement("a"));
    EXPECT_EQ(false, obj.removeElement("a"));
    EXPECT_EQ(nullptr, obj.getElement<Element>("a"));
    EXPECT_EQ(nullptr, obj.getElement<Container>("a"));
  }

  {
    Transform tf2(KDL::Frame(), "pcr/container/A1", "unknown");

    auto container = std::make_unique<Container>(shape, tf2);

    // Object
    Object obj;
    EXPECT_EQ(false, obj.addElement(tf2.frameId(), std::move(container)));
    EXPECT_EQ(nullptr, obj.getElement<Element>(tf2.frameId()));
  }
}

TEST(Object, getTree)
{
  // Element -> Container
  std::vector<BaseVolumePtr> shape;
  shape.emplace_back(std::make_shared<SphericalCapVolume>(0.00142, 0.00167));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00142, 0.00256, 0.00765));
  shape.emplace_back(std::make_shared<TruncatedConeVolume>(0.00256, 0.00275, 0.00478));
  shape.emplace_back(std::make_shared<CylinderVolume>(0.00275, 0.0040));

  Transform tf(KDL::Frame(), "pcr/container/A1", "root");

  auto container = std::make_unique<Container>(shape, tf);
  auto container_ptr = container.get();

  auto volume = container->addVolume(0.000000200);

  // Object
  Object obj;

  EXPECT_EQ(0, obj.getTree().getNrOfSegments());
  EXPECT_EQ(true, obj.addElement(tf.frameId(), std::move(container)));
  EXPECT_EQ(1, obj.getTree().getNrOfSegments());

  // auto segment_it = obj.getTree().getSegment(tf.frameId());
  EXPECT_STREQ(tf.frameId().c_str(), obj.getTree().getSegment(tf.frameId())->second.segment.getName().c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
