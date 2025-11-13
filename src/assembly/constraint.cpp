#include <sodf/assembly/constraint.h>
#include <algorithm>
#include <cmath>

#include <iostream>
#include <iomanip>

namespace sodf {
namespace assembly {

// ---------- tiny debug helpers (optional) ------------------------------------

void dumpAxis(const char* label, const Axis& a)
{
  std::cout << "      " << label << ".point = [" << a.point.transpose() << "]  dir = [" << a.direction.transpose()
            << "]\n";
}

void dumpPlane(const char* label, const Plane& p)
{
  std::cout << "      " << label << ".point = [" << p.point.transpose() << "]  normal = [" << p.normal.transpose()
            << "]\n";
}

void printTF(const char* label, const Eigen::Isometry3d& T)
{
  const Eigen::Vector3d t = T.translation();
  const Eigen::AngleAxisd aa(T.linear());
  std::cout << std::fixed << std::setprecision(6) << "    " << label << " t = [" << t.x() << ", " << t.y() << ", "
            << t.z() << "]"
            << "  aa = (" << aa.angle() << ", [" << aa.axis().transpose() << "])\n";
}

void dumpPose(const char* label, const Eigen::Isometry3d& P)
{
  printTF(label, P);
}

void dumpLineToLine(const Axis& A, const Axis& B)
{
  const Eigen::Vector3d u = A.direction.normalized();
  const Eigen::Vector3d v = B.direction.normalized();
  const double c = std::clamp(u.dot(v), -1.0, 1.0);
  const double ang = std::acos(c);

  // shortest distance between two (possibly skew) lines
  const Eigen::Vector3d w0 = A.point - B.point;
  const double denom = 1.0 - c * c;
  double dist;
  if (denom < 1e-12)
  {
    // nearly parallel: distance = |(A0-B0) x u|
    dist = w0.cross(u).norm();
  }
  else
  {
    const double s = (w0.dot(u) - w0.dot(v) * c) / denom;
    const Eigen::Vector3d PcA = A.point - s * u;
    dist = (B.point - PcA).cross(v).norm();
  }
  std::cout << "      angle(u,v) = " << ang << " rad, line-line shortest dist ≈ " << dist << " m\n";
}

static inline Eigen::Matrix3d align_dir_to_dir(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  // rotation R such that R * b == a
  const Eigen::Vector3d v = b.cross(a);
  const double s = v.norm();
  const double c = b.dot(a);
  if (s < 1e-15)
  {
    // already aligned or opposite
    if (c > 0.0)
      return Eigen::Matrix3d::Identity();
    // 180°: pick any orthogonal axis
    Eigen::Vector3d axis = b.unitOrthogonal();
    return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
  }
  const Eigen::Matrix3d vx = (Eigen::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
  // Rodrigues: R = I + vx + vx^2 * ((1-c)/s^2)
  return Eigen::Matrix3d::Identity() + vx + vx * vx * ((1.0 - c) / (s * s));
}

Pose solveCoincidentFrameFrame(const Pose& Fh, const Pose& Fg)
{
  return Fh * Fg.inverse();
}

Pose solveCoincidentPlaneFrame(const Plane& Hplane, const Pose& Fg)
{
  Pose T = Pose::Identity();
  // Orient guest +Z to plane normal (keep yaw around normal as-is)
  Eigen::Vector3d gx = Fg.linear().col(0).normalized();
  Eigen::Matrix3d R = align_dir_to_dir(Hplane.normal, gx).transpose();  // rotate guest so gz->normal
  T.linear() = R;

  // Project guest origin onto plane
  Eigen::Vector3d p = Fg.translation();
  double d = (Hplane.point - p).dot(Hplane.normal);
  T.pretranslate(Hplane.normal * d);
  return T;
}

Pose solveCoincidentPointFrame(const Point& Hp, const Pose& Fg)
{
  Pose T = Pose::Identity();
  T.pretranslate(Hp - Fg.translation());
  return T;
}

Pose solveCoincidentPointPoint(const Point& Hp, const Point& Gp)
{
  Pose T = Pose::Identity();
  T.pretranslate(Hp - Gp);
  return T;
}

Pose solveConcentricAxisAxis(const Axis& H, const Axis& G)
{
  Pose T = Pose::Identity();
  // 1) Align directions
  Eigen::Matrix3d R = align_dir_to_dir(H.direction, G.direction);
  T.linear() = R;

  // 2) Translate so G's point lands on H's line (closest point)
  Eigen::Vector3d pG = G.point;
  Eigen::Vector3d pH = H.point;
  Eigen::Vector3d zH = H.direction;

  // transform pG by R (rotation happens about origin)
  pG = R * pG;
  // closest point of pG to the host axis line through pH with dir zH
  Eigen::Vector3d w = pG - pH;
  Eigen::Vector3d pG_on_H = pH + zH * (zH.dot(w));
  T.pretranslate(pG_on_H - pG);
  return T;
}

Pose solveParallelAxisAxis(const Axis& H, const Axis& G)
{
  Pose T = Pose::Identity();
  T.linear() = align_dir_to_dir(H.direction, G.direction);
  return T;
}

Pose solveAngleAxisAxis(const Axis& H, const Axis& G, double radians)
{
  Pose T = Pose::Identity();
  // rotate around host axis so that angle(H,G_new) == radians
  // current angle:
  double cur = std::acos(std::clamp(H.direction.dot(G.direction), -1.0, 1.0));
  double dth = radians - cur;
  if (std::abs(dth) < 1e-12)
    return T;
  T.prerotate(Eigen::AngleAxisd(dth, H.direction));
  return T;
}

Pose solveDistancePlaneFrame(const Plane& Hplane, const Pose& G, double offset)
{
  Pose T = Pose::Identity();
  // Signed distance of G origin to plane
  double d = (G.translation() - Hplane.point).dot(Hplane.normal);
  T.pretranslate((offset - d) * Hplane.normal);
  return T;
}

Pose solveDistanceAxisFrame(const Axis& Haxis, const Pose& G, double distance)
{
  Pose T = Pose::Identity();
  // move along host axis direction
  T.pretranslate(distance * Haxis.direction);
  return T;
}

Pose solveDistancePointPoint(const Point& Hp, const Point& Gp, double distance)
{
  Pose T = Pose::Identity();
  Eigen::Vector3d dir = (Gp - Hp);
  double n = dir.norm();
  if (n < 1e-12)
  {
    // no direction; pick arbitrary (X)
    dir = Eigen::Vector3d::UnitX();
    n = 1.0;
  }
  dir /= n;
  // want ||Hp - (Gp + t*dir)|| == distance -> place Gp at Hp + distance*dir
  Eigen::Vector3d target = Hp + distance * dir;
  T.pretranslate(target - Gp);
  return T;
}

static inline double cone_radius_at(double r0, double r1, double H, double z)
{
  double t = std::clamp(z / std::max(H, 1e-16), 0.0, 1.0);
  return (1.0 - t) * r0 + t * r1;
}

Pose solveSeatConeOnCylinder(double r_cyl, const Axis& Hcyl, double r0_cone, double r1_cone, double H_cone,
                             const Axis& Gcone, double tol, int max_it)
{
  Pose T = Pose::Identity();

  // 1) Align axes
  T.linear() = align_dir_to_dir(Hcyl.direction, Gcone.direction);

  // 2) Find z* where cone radius matches cylinder radius
  double lo = 0.0, hi = H_cone, z = 0.0;
  for (int it = 0; it < max_it; ++it)
  {
    z = 0.5 * (lo + hi);
    double r = cone_radius_at(r0_cone, r1_cone, H_cone, z);
    double err = r - r_cyl;
    if (std::abs(err) < tol)
      break;
    (err > 0.0) ? (lo = z) : (hi = z);
  }

  // 3) Translate so guest cone circle at z seats at cylinder "mouth" plane (through Hcyl.point)
  Eigen::Vector3d pG = T * Gcone.point;  // rotated guest axis anchor
  Eigen::Vector3d pH = Hcyl.point;
  // Move pG by (pH - (pG + z*Hdir))
  Eigen::Vector3d d = pH - (pG + z * Hcyl.direction);
  T.pretranslate(d);
  return T;
}

}  // namespace assembly
}  // namespace sodf
