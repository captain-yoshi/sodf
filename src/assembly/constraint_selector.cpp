#include <sodf/assembly/constraint_selector.h>
#include <stdexcept>

#include <iostream>
#include <iomanip>
#include <algorithm>  // std::clamp
#include <cmath>      // std::acos

namespace sodf {
namespace assembly {

// ---------- tiny debug helpers (optional) ------------------------------------

static inline void dumpAxis(const char* label, const Axis& a)
{
  std::cout << "      " << label << ".point = [" << a.point.transpose() << "]  dir = [" << a.direction.transpose()
            << "]\n";
}

static inline void dumpPlane(const char* label, const Plane& p)
{
  std::cout << "      " << label << ".point = [" << p.point.transpose() << "]  normal = [" << p.normal.transpose()
            << "]\n";
}

// Provided elsewhere in your codebase; we just call it.
extern void printTF(const char* label, const Eigen::Isometry3d& P);

static inline void dumpPose(const char* label, const Eigen::Isometry3d& P)
{
  printTF(label, P);
}

static inline void dumpLineToLine(const Axis& A, const Axis& B)
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

// -------------------- Frame conversion helpers (world ↔ host) ----------------

// NOTE: These helpers are local to this file; we compute the host pose (H_w)
//       from the *host operand* itself via ctx.getFrame(hostRef, ...).

static inline Eigen::Vector3d to_host_point(const Eigen::Isometry3d& H_w, const Eigen::Vector3d& p_w)
{
  return H_w.inverse() * p_w;
}

static inline Eigen::Vector3d to_host_dir(const Eigen::Isometry3d& H_w, const Eigen::Vector3d& d_w)
{
  Eigen::Vector3d dh = H_w.linear().transpose() * d_w;
  const double n2 = dh.squaredNorm();
  if (n2 > 0.0)
    return dh / std::sqrt(n2);
  return Eigen::Vector3d(0.0, 0.0, 1.0);
}

static inline Axis to_host_axis(const Eigen::Isometry3d& H_w, const Axis& a_w)
{
  Axis a;
  a.point = to_host_point(H_w, a_w.point);
  a.direction = to_host_dir(H_w, a_w.direction);
  return a;
}

static inline Plane to_host_plane(const Eigen::Isometry3d& H_w, const Plane& p_w)
{
  Plane p;
  p.point = to_host_point(H_w, p_w.point);
  p.normal = to_host_dir(H_w, p_w.normal);
  return p;
}

static inline Eigen::Isometry3d to_host_pose(const Eigen::Isometry3d& H_w, const Eigen::Isometry3d& Pw)
{
  Eigen::Isometry3d Ph = Eigen::Isometry3d::Identity();
  Ph.linear() = H_w.linear().transpose() * Pw.linear();
  Ph.translation() = to_host_point(H_w, Pw.translation());
  return Ph;
}

static inline Eigen::Isometry3d delta_host_to_world(const Eigen::Isometry3d& H_w, const Eigen::Isometry3d& d_host)
{
  return H_w * d_host * H_w.inverse();
}

// -------------------- Primitive resolvers (Ref-based) ------------------------

Axis resolveAxis(const Ref& r, const SelectorContext& ctx)
{
  // For frames: allow #axisX/#axisY/#axisZ/#axis (default = Z)
  const bool wantX = (r.selector == "axisX");
  const bool wantY = (r.selector == "axisY");
  const bool wantZ = (r.selector == "axisZ" || r.selector.empty() || r.selector == "axis");
  const bool anyAxis = wantX || wantY || wantZ;

  switch (r.kind)
  {
    case NamespaceKind::Insertion:
    {
      SelectorContext::InsertionData ins;
      if (!ctx.getInsertion(r, ins))
        throw std::runtime_error("resolveAxis: insertion not found for '" + r.raw + "'");
      return Axis{ ins.mouth.translation(), ins.axis.normalized() };
    }

    case NamespaceKind::Shape:
    {
      Axis ax;
      double R = 0.0, H = 0.0;
      if (ctx.getCylinder(r, ax, R, H))
        return Axis{ ax.point, ax.direction.normalized() };

      double r0 = 0.0, r1 = 0.0;
      if (ctx.getCone(r, ax, r0, r1, H))
        return Axis{ ax.point, ax.direction.normalized() };

      // If asked for an axis on a plane, use plane normal
      Plane pl;
      if (anyAxis && ctx.getPlane(r, pl))
        return Axis{ pl.point, pl.normal.normalized() };

      break;
    }

    case NamespaceKind::StackedShape:
    {
      Axis ax;
      if (ctx.getStackedPrimaryAxis(r, ax))
        return Axis{ ax.point, ax.direction.normalized() };
      break;
    }

    case NamespaceKind::Link:
    case NamespaceKind::Origin:
    case NamespaceKind::Frame:
    {
      Pose F;
      if (!ctx.getFrame(r, F))
        throw std::runtime_error("resolveAxis: frame not found for '" + r.raw + "'");
      const Eigen::Vector3d dir =
          wantX ? F.linear().col(0) : wantY ? F.linear().col(1) : F.linear().col(2);  // default Z
      return Axis{ F.translation(), dir.normalized() };
    }

    default:
      break;
  }

  throw std::runtime_error("resolveAxis: cannot get axis from '" + r.raw + "'");
}

Plane resolvePlane(const Ref& r, const SelectorContext& ctx)
{
  switch (r.kind)
  {
    case NamespaceKind::Shape:
    {
      Plane pl;
      if (!ctx.getPlane(r, pl))
        throw std::runtime_error("resolvePlane: plane not found for '" + r.raw + "'");
      return Plane{ pl.point, pl.normal.normalized() };
    }

    case NamespaceKind::Link:
    case NamespaceKind::Origin:
    case NamespaceKind::Frame:
    {
      // Frame's XY plane with +Z normal
      Pose F;
      if (!ctx.getFrame(r, F))
        throw std::runtime_error("resolvePlane: frame not found for '" + r.raw + "'");
      return Plane{ F.translation(), F.linear().col(2).normalized() };
    }

    case NamespaceKind::Insertion:
    {
      // Insertion mouth plane: origin at mouth, normal along insertion axis
      SelectorContext::InsertionData ins;
      if (!ctx.getInsertion(r, ins))
        throw std::runtime_error("resolvePlane: insertion not found for '" + r.raw + "'");
      return Plane{ ins.mouth.translation(), ins.axis.normalized() };
    }

    default:
      break;
  }

  throw std::runtime_error("resolvePlane: cannot get plane from '" + r.raw + "'");
}

Pose resolvePose(const Ref& r, const SelectorContext& ctx)
{
  switch (r.kind)
  {
    case NamespaceKind::Link:
    case NamespaceKind::Origin:
    case NamespaceKind::Frame:
    {
      Pose F;
      if (!ctx.getFrame(r, F))
        throw std::runtime_error("resolvePose: frame not found for '" + r.raw + "'");
      return F;
    }
    case NamespaceKind::Insertion:
    {
      SelectorContext::InsertionData ins;
      if (!ctx.getInsertion(r, ins))
        throw std::runtime_error("resolvePose: insertion not found for '" + r.raw + "'");
      return ins.mouth;
    }
    default:
      break;
  }
  throw std::runtime_error("resolvePose: cannot get pose from '" + r.raw + "'");
}

Point resolvePoint(const Ref& r, const SelectorContext& ctx)
{
  // For now: frame center is a point
  if (r.kind == NamespaceKind::Frame || r.kind == NamespaceKind::Link || r.kind == NamespaceKind::Origin)
  {
    Pose F;
    if (!ctx.getFrame(r, F))
      throw std::runtime_error("resolvePoint: frame not found for '" + r.raw + "'");
    return F.translation();
  }

  throw std::runtime_error("resolvePoint: not implemented for '" + r.raw + "'");
}

// -------------------- High-level wrappers (Ref-based, HOST-frame) ------------

Eigen::Isometry3d Coincident(const Ref& host, const Ref& guest, const SelectorContext& ctx)
{
  // We compute everything in the host frame and map the delta back to world:
  // ΔW = H * ΔH * H^-1
  Pose H_w;
  if (!ctx.getFrame(host, H_w))
    throw std::runtime_error("Coincident: cannot fetch host pose for '" + host.raw + "'");

  // Prefer frame↔frame; else plane involvement; else point↔frame

  // Frame ↔ Frame
  try
  {
    Pose Fh_w = resolvePose(host, ctx);
    Pose Fg_w = resolvePose(guest, ctx);
    dumpPose("host pose (world)", Fh_w);
    dumpPose("guest pose (world)", Fg_w);

    Pose Fh_h = to_host_pose(H_w, Fh_w);  // should be ~Identity
    Pose Fg_h = to_host_pose(H_w, Fg_w);

    (void)Fh_h;  // unused; solve is relative to host in host frame

    Pose dH = solveCoincidentFrameFrame(Fh_h, Fg_h);  // host-frame delta
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  // Plane involvement
  try
  {
    Plane Hp_w = resolvePlane(host, ctx);
    dumpPlane("host plane (world)", Hp_w);

    Plane Hp_h = to_host_plane(H_w, Hp_w);

    try
    {
      Pose Fg_w = resolvePose(guest, ctx);
      dumpPose("guest pose (world)", Fg_w);
      Pose Fg_h = to_host_pose(H_w, Fg_w);

      Pose dH = solveCoincidentPlaneFrame(Hp_h, Fg_h);
      return delta_host_to_world(H_w, dH);
    }
    catch (...)
    {
    }

    try
    {
      Point Gp_w = resolvePoint(guest, ctx);
      std::cout << "      guest point (world) = [" << Gp_w.transpose() << "]\n";
      Point Gp_h = to_host_point(H_w, Gp_w);

      Pose dH = solveCoincidentPointPoint(Hp_h.point, Gp_h);
      return delta_host_to_world(H_w, dH);
    }
    catch (...)
    {
    }
  }
  catch (...)
  {
  }

  // Point ↔ Frame
  try
  {
    Point Hp_w = resolvePoint(host, ctx);
    Pose Fg_w = resolvePose(guest, ctx);
    std::cout << "      host point (world) = [" << Hp_w.transpose() << "]\n";
    dumpPose("guest pose (world)", Fg_w);

    Point Hp_h = to_host_point(H_w, Hp_w);
    Pose Fg_h = to_host_pose(H_w, Fg_w);

    Pose dH = solveCoincidentPointFrame(Hp_h, Fg_h);
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  throw std::runtime_error("Coincident: unsupported operands '" + host.raw + "' , '" + guest.raw + "'");
}

Eigen::Isometry3d Concentric(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx)
{
  // Host frame
  Pose H_w;
  if (!ctx.getFrame(host_axis, H_w))
    throw std::runtime_error("Concentric: cannot fetch host pose for '" + host_axis.raw + "'");

  // Resolve in world
  Axis aH_w = resolveAxis(host_axis, ctx);
  Axis aG_w = resolveAxis(guest_axis, ctx);

  // Debug (world)
  dumpAxis("host axis (world)", aH_w);
  dumpAxis("guest axis (world)", aG_w);
  dumpLineToLine(aH_w, aG_w);

  // Convert to host
  Axis aH_h = to_host_axis(H_w, aH_w);
  Axis aG_h = to_host_axis(H_w, aG_w);

  // Solve in host frame
  Eigen::Isometry3d dH = solveConcentricAxisAxis(aH_h, aG_h);

  // Map delta to world
  return delta_host_to_world(H_w, dH);
}

Eigen::Isometry3d Parallel(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host_axis, H_w))
    throw std::runtime_error("Parallel: cannot fetch host pose for '" + host_axis.raw + "'");

  Axis aH_w = resolveAxis(host_axis, ctx);
  Axis aG_w = resolveAxis(guest_axis, ctx);

  dumpAxis("host axis (world)", aH_w);
  dumpAxis("guest axis (world)", aG_w);
  dumpLineToLine(aH_w, aG_w);

  Axis aH_h = to_host_axis(H_w, aH_w);
  Axis aG_h = to_host_axis(H_w, aG_w);

  Eigen::Isometry3d dH = solveParallelAxisAxis(aH_h, aG_h);
  return delta_host_to_world(H_w, dH);
}

Eigen::Isometry3d Angle(const Ref& host_axis, const Ref& guest_axis, double radians, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host_axis, H_w))
    throw std::runtime_error("Angle: cannot fetch host pose for '" + host_axis.raw + "'");

  Axis aH_w = resolveAxis(host_axis, ctx);
  Axis aG_w = resolveAxis(guest_axis, ctx);

  Axis aH_h = to_host_axis(H_w, aH_w);
  Axis aG_h = to_host_axis(H_w, aG_w);

  Eigen::Isometry3d dH = solveAngleAxisAxis(aH_h, aG_h, radians);
  return delta_host_to_world(H_w, dH);
}

Eigen::Isometry3d Distance(const Ref& host, const Ref& guest, double value, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host, H_w))
    throw std::runtime_error("Distance: cannot fetch host pose for '" + host.raw + "'");

  // Prefer plane rule first
  try
  {
    Plane Hp_w = resolvePlane(host, ctx);
    Pose Gp_w = resolvePose(guest, ctx);

    Plane Hp_h = to_host_plane(H_w, Hp_w);
    Pose Gp_h = to_host_pose(H_w, Gp_w);

    Eigen::Isometry3d dH = solveDistancePlaneFrame(Hp_h, Gp_h, value);
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  // Else if either side yields an axis → use host axis as direction
  try
  {
    Axis Ha_w = resolveAxis(host, ctx);
    Pose Gp_w = resolvePose(guest, ctx);

    Axis Ha_h = to_host_axis(H_w, Ha_w);
    Pose Gp_h = to_host_pose(H_w, Gp_w);

    Eigen::Isometry3d dH = solveDistanceAxisFrame(Ha_h, Gp_h, value);
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  // Else point-to-point
  try
  {
    Point Hp_w = resolvePoint(host, ctx);
    Point Gp_w = resolvePoint(guest, ctx);

    Point Hp_h = to_host_point(H_w, Hp_w);
    Point Gp_h = to_host_point(H_w, Gp_w);

    Eigen::Isometry3d dH = solveDistancePointPoint(Hp_h, Gp_h, value);
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  throw std::runtime_error("Distance: unsupported operands '" + host.raw + "' , '" + guest.raw + "'");
}

Eigen::Isometry3d SeatConeOnCylinder(const Ref& host_cyl, const Ref& guest_cone, const SelectorContext& ctx, double tol,
                                     int max_it)
{
  Pose H_w;
  if (!ctx.getFrame(host_cyl, H_w))
    throw std::runtime_error("SeatConeOnCylinder: cannot fetch host pose for '" + host_cyl.raw + "'");

  // 1) host: cylinder dims + axis (or via insertion profile)
  Axis axH_w;
  double r = 0.0, Hcyl = 0.0;
  if (!ctx.getCylinder(host_cyl, axH_w, r, Hcyl))
  {
    SelectorContext::InsertionData insH;
    if (!ctx.getInsertion(host_cyl, insH))
      throw std::runtime_error("SeatConeOnCylinder: host '" + host_cyl.raw + "' is not a cylinder and not an insertion");
    double rtmp = 0.0;
    if (!insH.getCylinderRadius || !insH.getCylinderRadius(rtmp))
      throw std::runtime_error("SeatConeOnCylinder: host '" + host_cyl.raw + "' lacks cylinder radius");
    r = rtmp;
    axH_w = Axis{ insH.mouth.translation(), insH.axis.normalized() };
  }

  // 2) guest: cone dims + axis (or via insertion profile)
  Axis axG_w;
  double r0 = 0.0, r1 = 0.0, Hcone = 0.0;
  if (!ctx.getCone(guest_cone, axG_w, r0, r1, Hcone))
  {
    SelectorContext::InsertionData insG;
    if (!ctx.getInsertion(guest_cone, insG))
      throw std::runtime_error("SeatConeOnCylinder: guest '" + guest_cone.raw + "' is not a cone and not an insertion");
    if (!insG.getConeDims || !insG.getConeDims(r0, r1, Hcone))
      throw std::runtime_error("SeatConeOnCylinder: guest '" + guest_cone.raw + "' lacks cone dims");
    axG_w = Axis{ insG.mouth.translation(), insG.axis.normalized() };
  }

  Axis axH_h = to_host_axis(H_w, axH_w);
  Axis axG_h = to_host_axis(H_w, axG_w);

  Eigen::Isometry3d dH = solveSeatConeOnCylinder(r, axH_h, r0, r1, Hcone, axG_h, tol, max_it);
  return delta_host_to_world(H_w, dH);
}

}  // namespace assembly
}  // namespace sodf
