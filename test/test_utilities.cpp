#include <sodf/geometry/utilities.h>
#include <sodf/conversion.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

TEST(Geometry, alignCenterFrames)
{
  Eigen::Isometry3d wTa = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d wTb = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d xTc = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d xTd = Eigen::Isometry3d::Identity();

  wTa.translate(Eigen::Vector3d(1, 1, 1));
  wTb.translate(Eigen::Vector3d(1.02, 1.02, 1));
  xTc.translate(Eigen::Vector3d(0, -0.0182843, 1));
  xTd.translate(Eigen::Vector3d(0, 0.01, 1));

  auto frame = alignCenterFrames(wTa, wTb, xTc, xTd, 1e-06);

  xTc.translation() = Eigen::Vector3d(0, 0, 0);
  xTc.rotate(sodf::RPY(0, 0, M_PI_4).linear());
  xTc.translate(Eigen::Vector3d(0, -0.0182843, 1));

  xTd.translation() = Eigen::Vector3d(0, 0, 0);
  xTd.rotate(sodf::RPY(0, 0, M_PI_4).linear());
  xTd.translate(Eigen::Vector3d(0, 0.01, 1));

  frame = alignCenterFrames(wTa, wTb, xTc, xTd, 1e-06);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
