#include <sodf/assembly/constraint_selector.h>
#include <stdexcept>

#include <sstream>
#include <iomanip>
#include <algorithm>  // std::clamp
#include <cmath>      // std::acos

namespace sodf {
namespace assembly {

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
  return Eigen::Vector3d(1.0, 0.0, 0.0);
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

// -------------------- Axis sign helpers --------------------------------------

// Many assembly constraints are line-based (axis direction is sign-invariant).
// If authoring conventions or mixed operand types flip +axis, we normalize the
// guest axis to point in the same general direction as the host axis.

static inline void align_axis_sign(const Axis& host, Axis& guest)
{
  if (host.direction.dot(guest.direction) < 0.0)
    guest.direction = -guest.direction;
}

// -------------------- Primitive resolvers (Ref-based) ------------------------

Axis resolveAxis(const Ref& r, const SelectorContext& ctx)
{
  // For frames: allow #axisX/#axisY/#axisZ/#axis (default = Z)
  const bool wantX = (r.selector == "axisX" || r.selector.empty() || r.selector == "axis");
  const bool wantY = (r.selector == "axisY");
  const bool wantZ = (r.selector == "axisZ");
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
          wantX ? F.linear().col(0) : wantY ? F.linear().col(1) : F.linear().col(2);  // default X per your legacy
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
      Pose F;
      if (!ctx.getFrame(r, F))
        throw std::runtime_error("resolvePlane: frame not found for '" + r.raw + "'");

      Eigen::Vector3d n;
      if (r.selector == "axisX")
        n = F.linear().col(0);
      else if (r.selector == "axisY")
        n = F.linear().col(1);
      else  // "", "axis", or "axisZ" → default Z normal
        n = F.linear().col(2);

      return Plane{ F.translation(), n.normalized() };
    }

    case NamespaceKind::Insertion:
    {
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
    case NamespaceKind::Shape:
    case NamespaceKind::StackedShape:
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
  switch (r.kind)
  {
    case NamespaceKind::Frame:
    case NamespaceKind::Link:
    case NamespaceKind::Origin:
    case NamespaceKind::Shape:
    case NamespaceKind::StackedShape:
    {
      Pose F;
      if (!ctx.getFrame(r, F))
        throw std::runtime_error("resolvePoint: frame not found for '" + r.raw + "'");
      return F.translation();
    }

    case NamespaceKind::Insertion:
    {
      SelectorContext::InsertionData ins;
      if (!ctx.getInsertion(r, ins))
        throw std::runtime_error("resolvePoint: insertion not found for '" + r.raw + "'");
      return ins.mouth.translation();
    }

    default:
      break;
  }

  throw std::runtime_error("resolvePoint: not implemented for '" + r.raw + "'");
}

// -------------------- High-level wrappers (Ref-based, HOST-frame) ------------

Eigen::Isometry3d Coincident(const Ref& host, const Ref& guest, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host, H_w))
    throw std::runtime_error("Coincident: cannot fetch host pose for '" + host.raw + "'");

  // Frame ↔ Frame
  try
  {
    Pose Fh_w = resolvePose(host, ctx);
    Pose Fg_w = resolvePose(guest, ctx);

    Pose Fh_h = to_host_pose(H_w, Fh_w);
    Pose Fg_h = to_host_pose(H_w, Fg_w);

    (void)Fh_h;

    Pose dH = solveCoincidentFrameFrame(Fh_h, Fg_h);
    return delta_host_to_world(H_w, dH);
  }
  catch (...)
  {
  }

  // Plane involvement
  try
  {
    Plane Hp_w = resolvePlane(host, ctx);
    Plane Hp_h = to_host_plane(H_w, Hp_w);

    try
    {
      Pose Fg_w = resolvePose(guest, ctx);
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

Eigen::Isometry3d CoincidentPoint(const Ref& host, const Ref& guest, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host, H_w))
    throw std::runtime_error("CoincidentPoint: cannot fetch host pose for '" + host.raw + "'");

  // Resolve points in world (fallback to pose translation if needed)
  Point Hp_w;
  Point Gp_w;

  try
  {
    Hp_w = resolvePoint(host, ctx);
  }
  catch (...)
  {
    Pose Fh_w = resolvePose(host, ctx);
    Hp_w = Fh_w.translation();
  }

  try
  {
    Gp_w = resolvePoint(guest, ctx);
  }
  catch (...)
  {
    Pose Fg_w = resolvePose(guest, ctx);
    Gp_w = Fg_w.translation();
  }

  // Convert to host frame
  Point Hp_h = to_host_point(H_w, Hp_w);
  Point Gp_h = to_host_point(H_w, Gp_w);

  Pose dH = solveCoincidentPointPoint(Hp_h, Gp_h);
  return delta_host_to_world(H_w, dH);
}

Eigen::Isometry3d Concentric(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host_axis, H_w))
    throw std::runtime_error("Concentric: cannot fetch host pose for '" + host_axis.raw + "'");

  Axis aH_w = resolveAxis(host_axis, ctx);
  Axis aG_w = resolveAxis(guest_axis, ctx);

  // NEW: sign-invariant axis alignment
  align_axis_sign(aH_w, aG_w);

  Axis aH_h = to_host_axis(H_w, aH_w);
  Axis aG_h = to_host_axis(H_w, aG_w);

  Eigen::Isometry3d dH = solveConcentricAxisAxis(aH_h, aG_h);
  return delta_host_to_world(H_w, dH);
}

Eigen::Isometry3d Parallel(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx)
{
  Pose H_w;
  if (!ctx.getFrame(host_axis, H_w))
    throw std::runtime_error("Parallel: cannot fetch host pose for '" + host_axis.raw + "'");

  Axis aH_w = resolveAxis(host_axis, ctx);
  Axis aG_w = resolveAxis(guest_axis, ctx);

  // NEW: sign-invariant axis alignment
  align_axis_sign(aH_w, aG_w);

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

  // Intentionally NOT aligning sign here to avoid changing any signed-angle semantics.

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

  // NEW: sign-invariant axis alignment
  align_axis_sign(axH_w, axG_w);

  // Axes in host frame
  Axis axH_h = to_host_axis(H_w, axH_w);
  Axis axG_h = to_host_axis(H_w, axG_w);

  Pose dH;
  if (!solveSeatConeOnCylinder(r, axH_h, r0, r1, Hcone, axG_h, dH, tol, max_it))
  {
    std::ostringstream oss;
    oss << "SeatConeOnCylinder: no solution for host='" << host_cyl.raw << "' guest='" << guest_cone.raw
        << "' (r_cyl=" << r << ", cone radius range=[" << std::min(r0, r1) << ", " << std::max(r0, r1) << "])";
    throw std::runtime_error(oss.str());
  }

  Eigen::Isometry3d dW = delta_host_to_world(H_w, dH);
  return dW;
}

}  // namespace assembly
}  // namespace sodf
