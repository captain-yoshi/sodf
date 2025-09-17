#include <sodf/geometry/alignment.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace sodf {
namespace geometry {

Eigen::Isometry3d alignCenterFrames(const Eigen::Isometry3d& wTa, const Eigen::Isometry3d& wTb,
                                    const Eigen::Isometry3d& xTc, const Eigen::Isometry3d& xTd, double epsilon)
{
  constexpr double kTiny = 1e-12;

  // 1) Place C onto A in {W}
  Eigen::Isometry3d wTx = wTa * xTc.inverse();
  const Eigen::Isometry3d wTc = wTx * xTc;
  const Eigen::Isometry3d wTd = wTx * xTd;

  // 2) Validate segment lengths ||AB|| vs ||CD||
  const double length_ab = (wTb.translation() - wTa.translation()).norm();
  const double length_cd = (wTd.translation() - wTc.translation()).norm();

  if (length_ab < kTiny || length_cd < kTiny)
  {
    throw std::runtime_error("Degenerate case: |AB| or |CD| is below threshold");
  }

  const double length_error = length_ab - length_cd;
  if (std::abs(length_error) > epsilon)
  {
    throw std::runtime_error("||AB|| - ||CD|| = " + std::to_string(length_error) + " exceeds epsilon " +
                             std::to_string(epsilon));
  }

  // 3) Align vector CD to vector AB
  const Eigen::Isometry3d aTb = wTa.inverse() * wTb;
  const Eigen::Isometry3d cTd = wTc.inverse() * wTd;

  const Eigen::Vector3d v_ab_A = aTb.translation();
  const Eigen::Vector3d v_cd_A = cTd.translation();

  Eigen::Vector3d dir_ab_A = v_ab_A.normalized();
  Eigen::Vector3d dir_cd_A = v_cd_A.normalized();

  Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(dir_cd_A, dir_ab_A);
  q_align.normalize();

  Eigen::Isometry3d align_rotation = Eigen::Isometry3d::Identity();
  align_rotation.linear() = q_align.toRotationMatrix();

  // Apply rotation about A/C (they coincide in {W})
  wTx = wTa * align_rotation * xTc.inverse();

  // 4) Center: translate along the aligned AB by half the small length difference
  if (length_ab != length_cd)
  {
    const double offset = 0.5 * (length_ab - length_cd);

    // Rotate {A}'s +X to the AB direction to get a unit axis along AB in {A}
    Eigen::Quaterniond q_x_to_ab = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir_ab_A);
    q_x_to_ab.normalize();

    Eigen::Isometry3d rotate_axis = Eigen::Isometry3d::Identity();
    rotate_axis.linear() = q_x_to_ab.toRotationMatrix();

    // Move along that axis by "offset" in {W}
    Eigen::Isometry3d wTa_offset = wTa * rotate_axis;
    wTa_offset.translate(Eigen::Vector3d(offset, 0.0, 0.0));

    // Update {W}T{X} by the world-space translation delta
    wTx.pretranslate(wTa_offset.translation() - wTa.translation());
  }

  return wTx;
}

}  // namespace geometry
}  // namespace sodf
