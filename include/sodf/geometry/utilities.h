#pragma once

#include <Eigen/Geometry>
#include <iostream>
#include <exception>
#include <iomanip>

namespace sodf {
namespace geometry {

//
// c = sqrt(a^2+b^2 - 2*a*b*cos(gamma))
double computeCosineAngle(double a, double b, double c)
{
  return std::acos((std::pow(c, 2) - std::pow(a, 2) - std::pow(b, 2)) / (-2 * a * b));
}

// https://math.stackexchange.com/a/4034978
double computeAngle(const Eigen::Vector3d& v, const Eigen::Vector3d& w)
{
  return std::acos(v.normalized().transpose() * w.normalized());
}

// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
Eigen::Matrix3d computeRotationMatrixFromAxisAngle(double theta, const Eigen::Vector3d& axis)
{
  Eigen::Matrix3d rot;

  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  const double& sx = axis(0);
  const double& sy = axis(1);
  const double& sz = axis(2);

  const double sx2 = std::pow(sx, 2);
  const double sy2 = std::pow(sy, 2);
  const double sz2 = std::pow(sz, 2);

  rot(0, 0) = sx2 * (1 - cos_theta) + cos_theta;
  rot(0, 1) = sx * sy * (1 - cos_theta) - sz * sin_theta;
  rot(0, 2) = sx * sz * (1 - cos_theta) + sy * sin_theta;

  rot(1, 0) = sx * sy * (1 - cos_theta) + sz * sin_theta;
  rot(1, 1) = sy2 * (1 - cos_theta) + cos_theta;
  rot(1, 2) = sy * sz * (1 - cos_theta) - sx * sin_theta;

  rot(2, 0) = sx * sz * (1 - cos_theta) - sy * sin_theta;
  rot(2, 1) = sy * sz * (1 - cos_theta) + sx * sin_theta;
  rot(2, 2) = sz2 * (1 - cos_theta) + cos_theta;

  return rot;
}

// https://math.stackexchange.com/a/4034978
Eigen::Vector3d computeShortestAxisOfRotation(const Eigen::Vector3d& v, const Eigen::Vector3d& w)
{
  return (v.cross(w) / (v.cross(w)).norm()).normalized();
}

double computeDistance(const Eigen::Vector3d& v, const Eigen::Vector3d& w)
{
  return (v - w).norm();
}

double computeDistance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b)
{
  return computeDistance(a.translation(), b.translation());
}

/* @brief Align and center vector CD with vector AB
 *
 * @param wTa transform from frame W to frame A
 * @param wTb transform from frame W to frame B
 * @param xTc transform from frame X to frame C
 * @param xTd transform from frame X to frame D
 * @param epsilon minimally acceptable threshold between ||AB|| and ||CD||
 *
 * @return transform betwwen W and X so that CD is aligned to AB and at the center
 *
 *   {W}
 *
 *        {B}
 *        /
 *       /
 *      /
 *     /
 *   {A}
 *          {C} -------- {D}  {X}
 */
Eigen::Isometry3d alignCenterFrames(const Eigen::Isometry3d& wTa, const Eigen::Isometry3d& wTb,
                                    const Eigen::Isometry3d& xTc, const Eigen::Isometry3d& xTd, double epsilon)
{
  // align point C with A wrt. frame W
  Eigen::Isometry3d wTx = wTa * xTc.inverse();
  Eigen::Isometry3d wTc = wTx * xTc;
  Eigen::Isometry3d wTd = wTx * xTd;

  // validate norm between vector AB and CD
  double distance_ab = computeDistance(wTa, wTb);
  double distance_cd = computeDistance(wTc, wTd);

  double distance_error = distance_ab - distance_cd;
  if (std::abs(distance_error) > epsilon)
    throw std::runtime_error("norm of ab - cd " + std::to_string(distance_error) + " is above threshold" +
                             std::to_string(epsilon));

  // Align vector CD to vector AB
  Eigen::Isometry3d aTb = wTa.inverse() * wTb;
  Eigen::Isometry3d cTd = wTc.inverse() * wTd;

  double axis_angle = computeAngle(cTd.translation(), aTb.translation());
  auto s = computeShortestAxisOfRotation(cTd.translation(), aTb.translation());
  s.normalize();

  auto rmat = computeRotationMatrixFromAxisAngle(axis_angle, s);

  Eigen::Isometry3d align_rotation = Eigen::Isometry3d::Identity();
  align_rotation.linear() = rmat;

  wTx = wTa * align_rotation * xTc.inverse();

  std::cout << "norm between point a & c = " << computeDistance(wTa, wTx * xTc) << std::endl;
  std::cout << "norm between point b & d = " << computeDistance(wTb, wTx * xTd) << std::endl;

  // adjust vector CD at the center of vector AB
  if (distance_ab != distance_cd)
  {
    double offset = distance_ab - distance_cd;

    // compute rotation needed to align wTa x axis to vector ab
    auto wTa_xaxis = wTa;
    wTa_xaxis.translate(Eigen::Vector3d(1, 0, 0));
    Eigen::Isometry3d aTxaxis = wTa.inverse() * wTa_xaxis;

    axis_angle = computeAngle(aTxaxis.translation(), aTb.translation());
    std::cout << "axis angle = " << axis_angle << std::endl;

    s = computeShortestAxisOfRotation(aTxaxis.translation(), aTb.translation());
    s.normalize();

    rmat = computeRotationMatrixFromAxisAngle(axis_angle, s);

    Eigen::Isometry3d rotate_axis = Eigen::Isometry3d::Identity();
    rotate_axis.linear() = rmat;

    // rotate wTa towards vector AB and move to half offset
    auto wTa_offset = wTa * rotate_axis;
    wTa_offset.translate(Eigen::Vector3d(offset / 2.0, 0, 0));

    // update frame translation
    wTx.pretranslate(wTa_offset.translation() - wTa.translation());
  }
  std::cout << "norm between point a & c = " << computeDistance(wTa, wTx * xTc) << std::endl;
  std::cout << "norm between point b & d = " << computeDistance(wTb, wTx * xTd) << std::endl;

  return wTx;
}

/* @brief Rotate an axis so that it passes through a point in space
 *
 * @param axis unit vector
 * @param point vector in space wrt. the axis frame
 *
 * @return rotation matrix that aligns the given axis with the point in space
 */
const Eigen::Matrix3d alignAxisToPoint(const Eigen::Vector3d& axis, const Eigen::Vector3d& point)
{
  double axis_angle = computeAngle(axis, point);
  auto s = computeShortestAxisOfRotation(axis, point);
  s.normalize();

  Eigen::Matrix3d rmat = computeRotationMatrixFromAxisAngle(axis_angle, s);

  return rmat;
}

}  // namespace geometry
}  // namespace sodf
