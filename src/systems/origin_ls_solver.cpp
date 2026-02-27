// src/systems/origin_ls_solver.cpp
#include <sodf/systems/origin_ls_solver.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <variant>

#include <Eigen/Eigenvalues>

#include <sodf/components/origin_constraint.h>
#include <sodf/geometry/lie.h>
#include <sodf/geometry/frame.h>
#include <sodf/geometry/transform.h>
#include <sodf/systems/scene_graph.h>

#include <sodf/assembly/selector_context_db.h>
#include <sodf/assembly/namespace.h>
#include <sodf/assembly/constraint_selector.h>

namespace {
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Compact residuals (helpers)
struct Residual
{
  double angle_rad = 0.0;
  double dist_m = 0.0;
};

// NEW: pick which guest axis to use based on selector (default +X)
static inline Eigen::Vector3d pick_guest_axis(const std::string& selector, const Eigen::Isometry3d& Fg)
{
  if (selector == "axisY")
    return Fg.linear().col(1);
  if (selector == "axisZ")
    return Fg.linear().col(2);
  // default: +X
  return Fg.linear().col(0);
}

// (keep Residual, concentric_residual, parallel_residual as-is)

// keep plane_frame_residual (used elsewhere); add a selector-aware variant:
static inline Residual plane_frame_residual_axis(const sodf::assembly::Plane& Hp, const Eigen::Isometry3d& Fg,
                                                 int guest_axis_col /*0|1|2*/)
{
  const Eigen::Vector3d n = Hp.normal.normalized();
  const Eigen::Vector3d ag = Fg.linear().col(guest_axis_col).normalized();
  const double c = std::abs(n.dot(ag));
  const double ang = std::acos(std::clamp(c, 0.0, 1.0));
  const double d = (Fg.translation() - Hp.point).dot(n);
  return { ang, std::abs(d) };
}

static inline double unsigned_angle_between_dirs(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  const Eigen::Vector3d an = a.normalized();
  const Eigen::Vector3d bn = b.normalized();
  const double c = std::abs(an.dot(bn));
  return std::acos(std::clamp(c, 0.0, 1.0));  // 0 for parallel OR anti-parallel
}

static inline Residual concentric_residual_unsigned(const sodf::assembly::Axis& H, const sodf::assembly::Axis& G)
{
  const Eigen::Vector3d h = H.direction.normalized();
  const Eigen::Vector3d g = G.direction.normalized();

  const double ang = unsigned_angle_between_dirs(h, g);

  const Eigen::Vector3d w0 = G.point - H.point;
  const Eigen::Vector3d cx = h.cross(g);

  double dist;
  if (cx.squaredNorm() < 1e-16)
    dist = (w0.cross(h)).norm();
  else
    dist = std::abs(w0.dot(cx.normalized()));

  return { ang, dist };
}

static inline Residual concentric_residual(const sodf::assembly::Axis& H, const sodf::assembly::Axis& G)
{
  const Eigen::Vector3d h = H.direction.normalized();
  const Eigen::Vector3d g = G.direction.normalized();
  const double ang = sodf::geometry::angle_between(h, g);

  const Eigen::Vector3d w0 = G.point - H.point;
  const Eigen::Vector3d cx = h.cross(g);
  double dist;
  if (cx.squaredNorm() < 1e-16)
    dist = (w0.cross(h)).norm();
  else
    dist = std::abs(w0.dot(cx.normalized()));
  return { ang, dist };
}

static inline Residual parallel_residual(const sodf::assembly::Axis& H, const sodf::assembly::Axis& G)
{
  const Eigen::Vector3d h = H.direction.normalized();
  const Eigen::Vector3d g = G.direction.normalized();
  return { sodf::geometry::angle_between(h, g), 0.0 };
}

static inline Residual plane_frame_residual(const sodf::assembly::Plane& Hp, const Eigen::Isometry3d& Fg)
{
  const Eigen::Vector3d n = Hp.normal.normalized();
  const Eigen::Vector3d xg = Fg.linear().col(0).normalized();
  const double c = std::abs(n.dot(xg));                   // unsigned
  const double ang = std::acos(std::clamp(c, 0.0, 1.0));  // 0 for parallel or anti-parallel
  const double d = (Fg.translation() - Hp.point).dot(n);
  return { ang, std::abs(d) };
}

static inline Residual axis_frame_distance_residual(const sodf::assembly::Axis& Ha, const Eigen::Isometry3d& Fg,
                                                    double want)
{
  const Eigen::Vector3d n = Ha.direction.normalized();
  const double have = (Fg.translation() - Ha.point).dot(n);
  return { 0.0, std::abs(have - want) };
}

template <typename Accum>
static void add_coincident_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G,
                              const sodf::components::OriginComponent& origin,
                              const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P,
                              Accum&& accum)
{
  using namespace sodf::assembly;
  using namespace sodf::geometry;

  bool done = false;
  try
  {
    Plane Hp = resolvePlane(H, ctx);
    Pose Fg = resolvePose(G, ctx);

    const Eigen::Vector3d n = Hp.normal.normalized();
    const Eigen::Vector3d xg_pos = Fg.translation();
    const double rd = n.dot(xg_pos - Hp.point);

    Eigen::Matrix<double, 1, 6> Jd = Eigen::Matrix<double, 1, 6>::Zero();
    Jd.block<1, 3>(0, 0) = n.transpose() * skew(xg_pos);
    Jd.block<1, 3>(0, 3) = n.transpose();
    accum(Jd, Eigen::VectorXd::Constant(1, rd), P.w_pos);

    const Eigen::Vector3d aG = pick_guest_axis(G.selector, Fg);
    const Eigen::Vector3d r_ang = aG.cross(n);

    Eigen::Matrix<double, 3, 6> Ja = Eigen::Matrix<double, 3, 6>::Zero();
    Ja.block<3, 3>(0, 0) = -skew(n) * skew(aG);
    accum(Ja, Eigen::Map<const Eigen::Vector3d>(r_ang.data()), P.w_ang);

    done = true;
  }
  catch (...)
  {
  }

  if (!done)
  {
    try
    {
      Pose Fh = resolvePose(H, ctx);
      Pose Fg = resolvePose(G, ctx);

      Plane Hp{ Fh.translation(), Fh.linear().col(0) };  // canonical plane

      const Eigen::Vector3d n = Hp.normal.normalized();
      const Eigen::Vector3d xg_pos = Fg.translation();
      const double rd = n.dot(xg_pos - Hp.point);

      Eigen::Matrix<double, 1, 6> Jd = Eigen::Matrix<double, 1, 6>::Zero();
      Jd.block<1, 3>(0, 0) = n.transpose() * skew(xg_pos);
      Jd.block<1, 3>(0, 3) = n.transpose();
      accum(Jd, Eigen::VectorXd::Constant(1, rd), P.w_pos);

      const Eigen::Vector3d aG = pick_guest_axis(G.selector, Fg);
      const Eigen::Vector3d r_ang = aG.cross(n);

      Eigen::Matrix<double, 3, 6> Ja = Eigen::Matrix<double, 3, 6>::Zero();
      Ja.block<3, 3>(0, 0) = -skew(n) * skew(aG);
      accum(Ja, Eigen::Map<const Eigen::Vector3d>(r_ang.data()), P.w_ang);
    }
    catch (...)
    {
    }
  }
}

template <typename Accum>
static void add_coincident_point_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G,
                                    const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P,
                                    Accum&& accum)
{
  using namespace sodf::assembly;
  using namespace sodf::geometry;

  Pose Fh = resolvePose(H, ctx);
  Pose Fg = resolvePose(G, ctx);

  const Eigen::Vector3d Ph = Fh.translation();
  const Eigen::Vector3d Pg = Fg.translation();

  // residual: match points
  Eigen::Vector3d r_pos = (Pg - Ph);

  // Jacobian for point residual in SE3 (same pattern as your concentric lateral term)
  Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
  J.block<3, 3>(0, 0) = skew(Pg);
  J.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  accum(J, Eigen::Map<Eigen::Vector3d>(r_pos.data()), P.w_pos);
}

template <typename Accum>
static void add_concentric_axis_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G,
                                   const sodf::components::OriginComponent& origin,
                                   const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P,
                                   Accum&& accum)
{
  using namespace sodf::assembly;
  using namespace sodf::geometry;

  Axis aH = resolveAxis(H, ctx);
  Axis aG = resolveAxis(G, ctx);

  const Eigen::Vector3d h = aH.direction.normalized();
  const Eigen::Vector3d g = aG.direction.normalized();
  const Eigen::Vector3d Pg = aG.point;
  const Eigen::Vector3d Ph = aH.point;

  // angular part
  Eigen::Vector3d r_ang = g.cross(h);
  Eigen::Matrix<double, 3, 6> J_ang = Eigen::Matrix<double, 3, 6>::Zero();
  J_ang.block<3, 3>(0, 0) = -skew(h) * skew(g);
  accum(J_ang, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);

  // lateral part (orthogonal to host axis)
  Eigen::Matrix3d N = proj_perp(h);
  Eigen::Vector3d r_pos = N * (Pg - Ph);
  Eigen::Matrix<double, 3, 6> J_pos = Eigen::Matrix<double, 3, 6>::Zero();
  J_pos.block<3, 3>(0, 0) = N * skew(Pg);
  J_pos.block<3, 3>(0, 3) = N;
  accum(J_pos, Eigen::Map<Eigen::Vector3d>(r_pos.data()), P.w_pos);
}

template <typename Accum>
void add_parallel_axis_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G,
                          const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P,
                          Accum&& accum)
{
  using namespace sodf::assembly;
  using Eigen::Vector3d;

  Axis aH = resolveAxis(H, ctx);
  Axis aG = resolveAxis(G, ctx);

  const Vector3d h = aH.direction.normalized();
  const Vector3d g = aG.direction.normalized();

  Vector3d r_ang = g.cross(h);
  Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
  J.block<3, 3>(0, 0) = -sodf::geometry::skew(h) * sodf::geometry::skew(g);
  accum(J, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);
}

template <typename Accum>
void add_distance_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G, double value,
                     const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P, Accum&& accum)
{
  using namespace sodf::assembly;

  bool added = false;
  try
  {
    Plane Hp = resolvePlane(H, ctx);
    Pose Fg = resolvePose(G, ctx);

    const Eigen::Vector3d n = Hp.normal.normalized();
    const Eigen::Vector3d xg = Fg.translation();
    double rd = n.dot(xg - Hp.point) - value;

    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    J.block<1, 3>(0, 0) = n.transpose() * sodf::geometry::skew(xg);
    J.block<1, 3>(0, 3) = n.transpose();
    accum(J, Eigen::VectorXd::Constant(1, rd), P.w_pos);
    added = true;
  }
  catch (...)
  {
  }

  if (!added)
  {
    try
    {
      Axis Ha = resolveAxis(H, ctx);
      Pose Fg = resolvePose(G, ctx);

      const Eigen::Vector3d u = Ha.direction.normalized();
      const Eigen::Vector3d xg = Fg.translation();
      double rd = (xg - Ha.point).dot(u) - value;

      Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
      J.block<1, 3>(0, 0) = u.transpose() * sodf::geometry::skew(xg);
      J.block<1, 3>(0, 3) = u.transpose();
      accum(J, Eigen::VectorXd::Constant(1, rd), P.w_pos);
    }
    catch (...)
    {
    }
  }
}

template <typename Accum>
void add_angle_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G, double radians,
                  const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P, Accum&& accum)
{
  using namespace sodf::assembly;

  Axis aH = resolveAxis(H, ctx);
  Axis aG = resolveAxis(G, ctx);

  const Eigen::Vector3d h = aH.direction.normalized();
  const Eigen::Vector3d g = aG.direction.normalized();

  const double cdes = std::cos(radians);
  const double r = g.dot(h) - cdes;  // scalar residual

  Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
  J.block<1, 3>(0, 0) = (g.cross(h)).transpose();
  accum(J, Eigen::VectorXd::Constant(1, r), P.w_ang);
}

template <typename Accum>
void add_frame_ls(const sodf::assembly::Ref& H, const sodf::assembly::Ref& G,
                  const sodf::assembly::SelectorContext& ctx, const sodf::systems::LSSolveParams& P, Accum&& accum)
{
  using namespace sodf::assembly;
  using namespace sodf::geometry;

  Pose Fh = resolvePose(H, ctx);
  Pose Fg = resolvePose(G, ctx);

  // relative transform
  Isometry3d T_err = Fh.inverse() * Fg;

  // 6D log residual
  // residual in se(3)
  Eigen::Matrix<double, 6, 1> r = logSE3(T_err);

  // ---- Correct Jacobian ----
  // J = Ad( T_err^-1 )
  Eigen::Matrix<double, 6, 6> J = sodf::geometry::adjointSE3<Eigen::Isometry3d>(T_err.inverse());

  // Split angular / positional weighting
  accum(J.topRows<3>(), r.head<3>(), P.w_ang);
  accum(J.bottomRows<3>(), r.tail<3>(), P.w_pos);
}

static inline void fill_concentric_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                         const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  try
  {
    auto aH = sodf::assembly::resolveAxis(H, ctx);
    auto aG = sodf::assembly::resolveAxis(G, ctx);
    auto R = concentric_residual_unsigned(aH, aG);
    e.ang_rad = R.angle_rad;
    e.dist_m = R.dist_m;
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("concentric_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "concentric_residual: unknown exception";
  }
}

static inline void fill_parallel_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                       const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  try
  {
    auto aH = sodf::assembly::resolveAxis(H, ctx);
    auto aG = sodf::assembly::resolveAxis(G, ctx);
    auto R = parallel_residual(aH, aG);
    e.ang_rad = R.angle_rad;
    e.dist_m = 0.0;
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("parallel_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "parallel_residual: unknown exception";
  }
}
static inline void fill_distance_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                       const sodf::assembly::Ref& G, double value,
                                       const sodf::assembly::SelectorContext& ctx)
{
  using namespace sodf::assembly;

  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  std::string first_error;  // keep error of attempt 1 (if any)

  // -------- Attempt 1: Plane–Frame distance --------
  try
  {
    auto Hp = resolvePlane(H, ctx);
    auto Fg = resolvePose(G, ctx);

    auto R = plane_frame_residual(Hp, Fg);
    e.ang_rad = R.angle_rad;

    const Eigen::Vector3d n = Hp.normal.normalized();
    const double have = n.dot(Fg.translation() - Hp.point);
    e.dist_m = std::abs(have - value);
    return;  // success
  }
  catch (const std::exception& ex)
  {
    first_error = std::string("attempt 1 (Plane–Frame): ") + ex.what();
  }
  catch (...)
  {
    first_error = "attempt 1 (Plane–Frame): unknown exception";
  }

  // -------- Attempt 2: Axis–Frame distance --------
  try
  {
    auto Ha = resolveAxis(H, ctx);
    auto Fg = resolvePose(G, ctx);

    auto R = axis_frame_distance_residual(Ha, Fg, value);
    e.ang_rad = 0.0;
    e.dist_m = R.dist_m;
    return;  // success
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = first_error;
    if (!e.error_msg.empty())
      e.error_msg += " | ";
    e.error_msg += std::string("attempt 2 (Axis–Frame): ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = first_error;
    if (!e.error_msg.empty())
      e.error_msg += " | ";
    e.error_msg += "attempt 2 (Axis–Frame): unknown exception";
  }
}

static inline void fill_coincident_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                         const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  using namespace sodf::assembly;

  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  std::string first_error;  // keep error of attempt 1 (if any)

  // -------- Attempt 1: host as Plane, guest as Pose --------
  try
  {
    auto Hp = resolvePlane(H, ctx);
    auto Fg = resolvePose(G, ctx);

    int col = 0;
    if (G.selector == "axisY")
      col = 1;
    else if (G.selector == "axisZ")
      col = 2;

    auto R = plane_frame_residual_axis(Hp, Fg, col);
    e.ang_rad = R.angle_rad;
    e.dist_m = R.dist_m;
    return;  // success
  }
  catch (const std::exception& ex)
  {
    first_error = std::string("attempt 1 (Plane host): ") + ex.what();
  }
  catch (...)
  {
    first_error = "attempt 1 (Plane host): unknown exception";
  }

  // -------- Attempt 2: host as Pose → canonical plane (X-axis normal) --------
  try
  {
    auto Fh = resolvePose(H, ctx);
    auto Fg = resolvePose(G, ctx);

    Plane Hp{ Fh.translation(), Fh.linear().col(0) };  // canonical plane from host

    int col = 0;
    if (G.selector == "axisY")
      col = 1;
    else if (G.selector == "axisZ")
      col = 2;

    auto R = plane_frame_residual_axis(Hp, Fg, col);
    e.ang_rad = R.angle_rad;
    e.dist_m = R.dist_m;
    return;  // success
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = first_error;
    if (!e.error_msg.empty())
      e.error_msg += " | ";
    e.error_msg += std::string("attempt 2 (Pose host): ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = first_error;
    if (!e.error_msg.empty())
      e.error_msg += " | ";
    e.error_msg += "attempt 2 (Pose host): unknown exception";
  }
}

static inline void fill_coincident_point_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                               const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  try
  {
    auto Fh = sodf::assembly::resolvePose(H, ctx);
    auto Fg = sodf::assembly::resolvePose(G, ctx);

    e.ang_rad = 0.0;
    e.dist_m = (Fg.translation() - Fh.translation()).norm();
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("coincident_point_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "coincident_point_residual: unknown exception";
  }
}

static inline void fill_angle_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                    const sodf::assembly::Ref& G, double target_radians,
                                    const sodf::assembly::SelectorContext& ctx)
{
  using namespace sodf::assembly;

  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  try
  {
    auto aH = resolveAxis(H, ctx);
    auto aG = resolveAxis(G, ctx);
    double hv = sodf::geometry::angle_between(aH.direction, aG.direction);
    e.ang_rad = std::abs(hv - target_radians);
    e.dist_m = 0.0;
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("angle_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "angle_residual: unknown exception";
  }
}

static inline void fill_seatcone_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                       const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();
  e.kind = "SeatCone";

  try
  {
    auto aH = sodf::assembly::resolveAxis(H, ctx);
    auto aG = sodf::assembly::resolveAxis(G, ctx);

    auto R = concentric_residual_unsigned(aH, aG);
    e.ang_rad = R.angle_rad;
    e.dist_m = R.dist_m;
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("seatcone_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "seatcone_residual: unknown exception";
  }
}

static inline void fill_frame_entry(sodf::systems::ResidualEntry& e, const sodf::assembly::Ref& H,
                                    const sodf::assembly::Ref& G, const sodf::assembly::SelectorContext& ctx)
{
  using namespace sodf::assembly;
  using namespace sodf::geometry;

  e.ok = true;
  e.ang_rad = 0.0;
  e.dist_m = 0.0;
  e.error_msg.clear();

  try
  {
    Pose Fh = resolvePose(H, ctx);
    Pose Fg = resolvePose(G, ctx);

    Isometry3d T_err = Fh.inverse() * Fg;

    Eigen::Matrix<double, 6, 1> xi = logSE3(T_err);

    e.ang_rad = xi.head<3>().norm();
    e.dist_m = xi.tail<3>().norm();
  }
  catch (const std::exception& ex)
  {
    e.ok = false;
    e.error_msg = std::string("frame_residual: ") + ex.what();
  }
  catch (...)
  {
    e.ok = false;
    e.error_msg = "frame_residual: unknown exception";
  }
}

}  // namespace

namespace sodf {
namespace systems {

using sodf::assembly::Axis;
using sodf::assembly::Plane;
using sodf::assembly::Pose;
using sodf::assembly::Ref;

using namespace sodf::geometry;

static JacobianReport summarize_linear_system_from_JTJ(const Eigen::Matrix<double, 6, 6>& JTJ, int rows)
{
  JacobianReport rep;
  rep.rows = rows;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(JTJ);
  if (es.info() != Eigen::Success)
  {
    rep.rank = 0;
    rep.nullity = 6;
    rep.condJ = std::numeric_limits<double>::infinity();
    return rep;
  }

  auto vals = es.eigenvalues();  // ascending λ_i ≥ 0
  const double lmax = vals.maxCoeff();
  const double eps_abs = 1e-12;
  const double eps_rel = 1e-8;
  const double thresh = std::max(eps_abs, eps_rel * lmax);

  int r = 0;
  double lmin_nz = std::numeric_limits<double>::infinity();
  for (int i = 0; i < vals.size(); ++i)
  {
    const double lam = vals[i];
    if (lam > thresh)
    {
      r++;
      lmin_nz = std::min(lmin_nz, lam);
    }
  }
  rep.rank = r;
  rep.nullity = std::max(0, 6 - r);

  if (r == 0 || !std::isfinite(lmin_nz))
    rep.condJ = std::numeric_limits<double>::infinity();
  else
    rep.condJ = std::sqrt(lmax / lmin_nz);  // cond(J) ≈ sqrt(cond(JᵀJ))

  return rep;
}

JacobianReport summarize_linear_system(const LSLinearStats& S)
{
  return summarize_linear_system_from_JTJ(S.JTJ, S.rows);
}

void print_jacobian_report(const JacobianReport& j)
{
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Linearization summary\n";
  std::cout << "  rows    = " << j.rows << "\n";
  std::cout << "  rank(J) = " << j.rank << "\n";
  std::cout << "  nullity = " << j.nullity << "  (remaining DoF)\n";
  std::cout << "  cond(J) ≈ ";
  if (std::isfinite(j.condJ))
    std::cout << std::scientific << j.condJ << std::fixed << "\n";
  else
    std::cout << "inf\n";
  if (j.rows > j.rank)
    std::cout << "  note: rows > rank(J) → system has redundancy (over-constrained but consistent).\n";
  if (j.nullity > 0)
    std::cout << "  note: nullity > 0 → under-constrained (free DoF remain).\n";
}

// ---------------- Gauss–Newton (one step) in WORLD ----------------

Eigen::Isometry3d solve_origin_least_squares_once(database::Database& db, const database::ObjectEntityMap& map,
                                                  database::EntityID /*eid*/, components::OriginComponent& origin,
                                                  const LSSolveParams& P, LSLinearStats* out_stats)
{
  // WORLD-only selector context
  auto ctx = sodf::assembly::makeSelectorContext(db, map);

  Eigen::Matrix<double, 6, 6> JTJ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> JTr = Eigen::Matrix<double, 6, 1>::Zero();
  int rows = 0;

  auto accum = [&](const Eigen::Matrix<double, Eigen::Dynamic, 6>& J, const Eigen::VectorXd& r, double w) {
    if (r.size() == 0)
      return;
    JTJ.noalias() += w * (J.transpose() * J);
    JTr.noalias() += w * (J.transpose() * r);
    rows += static_cast<int>(r.size());
  };

  bool hard_frame_solution = false;
  Eigen::Isometry3d hard_delta = Eigen::Isometry3d::Identity();

  for (const auto& step_any : origin.constraints)
  {
    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, sodf::geometry::Transform>)
          {
            // known type; just don't add LS residuals for it
            return;
          }
          else if constexpr (std::is_same_v<TStep, components::Concentric>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_concentric_axis_ls(H, G, origin, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::Parallel>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_parallel_axis_ls(H, G, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::Coincident>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_coincident_ls(H, G, origin, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::CoincidentPoint>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_coincident_point_ls(H, G, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::Distance>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_distance_ls(H, G, step.value, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::Angle>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            add_angle_ls(H, G, step.radians, ctx, P, accum);
          }
          else if constexpr (std::is_same_v<TStep, components::SeatConeOnCylinder>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host_cyl, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest_cone, origin);

            // Reuse the same concentric axis LS contribution
            add_concentric_axis_ls(H, G, origin, ctx, P, accum);

            // Important: DO NOT constrain motion along the axis here.
            // Keep seating depth for the discrete SeatConeOnCylinder pass.
          }
          else if constexpr (std::is_same_v<TStep, components::Frame>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);

            if (step.mode == components::Frame::Mode::FULL)
            {
              Pose Fh = resolvePose(H, ctx);
              Pose Fg = resolvePose(G, ctx);

              hard_delta = Fh * Fg.inverse();
              hard_frame_solution = true;

              return;  // still returns void
            }
            else
            {
              add_frame_ls(H, G, ctx, P, accum);
            }
          }
          else
          {
            std::ostringstream oss;
            oss << "solve_origin_least_squares_once: unsupported constraint type in OriginComponent::constraints. "
                << "TStep = " << typeid(TStep).name();
            throw std::runtime_error(oss.str());
          }
        },
        step_any);
  }

  if (hard_frame_solution)
  {
    return hard_delta;
  }

  // Solve (JTJ + λI) ξ = -JTr
  Eigen::Matrix<double, 6, 6> A = JTJ;
  A.diagonal().array() += P.lambda;
  Eigen::Matrix<double, 6, 1> b = -JTr;

  Eigen::Matrix<double, 6, 1> xi = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(A);
  if (ldlt.info() == Eigen::Success)
    xi = ldlt.solve(b);
  else
    xi = A.ldlt().solve(b);

  if (out_stats)
  {
    out_stats->JTJ = JTJ;
    out_stats->rows = rows;
  }

  // Update pose (left-multiply)
  return expSE3(xi);
}

// ----------------------- diagnostics (WORLD) -------------------------
std::vector<ResidualEntry> compute_origin_residuals_compact(database::Database& db,
                                                            const database::ObjectEntityMap& map,
                                                            const components::OriginComponent& origin)
{
  using namespace sodf::assembly;
  auto ctx = sodf::assembly::makeSelectorContext(db, map);

  std::vector<ResidualEntry> out;
  out.reserve(origin.constraints.size());

  auto to_ref_str = [](const Ref& r) {
    std::ostringstream os;
    os << r.entity << "/" << r.ns << "/" << r.id;
    if (!r.selector.empty())
      os << "#" << r.selector;
    return os.str();
  };

  for (const auto& step_any : origin.constraints)
  {
    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, sodf::geometry::Transform>)
          {
            return;
          }
          else if constexpr (std::is_same_v<TStep, components::Concentric>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Concentric", to_ref_str(H), to_ref_str(G) };
            fill_concentric_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Parallel>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Parallel", to_ref_str(H), to_ref_str(G) };
            fill_parallel_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Coincident>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Coincident", to_ref_str(H), to_ref_str(G) };
            fill_coincident_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::CoincidentPoint>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "CoincidentPoint", to_ref_str(H), to_ref_str(G) };
            fill_coincident_point_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Distance>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Distance", to_ref_str(H), to_ref_str(G) };
            fill_distance_entry(e, H, G, step.value, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Angle>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Angle", to_ref_str(H), to_ref_str(G) };
            fill_angle_entry(e, H, G, step.radians, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::SeatConeOnCylinder>)
          {
            Ref H = assembly::make_host_ref(step.host_cyl, origin);
            Ref G = assembly::make_guest_ref(step.guest_cone, origin);
            ResidualEntry e{ "SeatCone", to_ref_str(H), to_ref_str(G) };
            fill_seatcone_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Frame>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Frame", to_ref_str(H), to_ref_str(G) };
            fill_frame_entry(e, H, G, ctx);
            out.push_back(std::move(e));
          }
          else
          {
            std::ostringstream oss;
            oss << "compute_origin_residuals_compact: unsupported constraint type. "
                << "TStep = " << typeid(TStep).name();
            throw std::runtime_error(oss.str());
          }
        },
        step_any);
  }

  return out;
}

std::string format_origin_residual_errors(const std::vector<ResidualEntry>& residuals, double tol_ang_rad,
                                          double tol_dist_m)
{
  std::ostringstream oss;

  auto dump_error_lines = [&](const ResidualEntry& e) {
    if (e.error_msg.empty())
      return;

    // error_msg is something like:
    // "attempt 1 (...): ... | attempt 2 (...): ..."
    std::string msg = e.error_msg;
    std::size_t start = 0;
    while (start < msg.size())
    {
      std::size_t pipe = msg.find('|', start);
      std::string part = msg.substr(start, (pipe == std::string::npos ? msg.size() : pipe) - start);

      // trim leading spaces
      auto first_non_space = part.find_first_not_of(" \t");
      if (first_non_space != std::string::npos)
        part.erase(0, first_non_space);

      if (!part.empty())
        oss << "       error: " << part << "\n";

      if (pipe == std::string::npos)
        break;
      start = pipe + 1;
    }
  };

  for (const auto& e : residuals)
  {
    // 1) Hard errors (failed to evaluate residual)
    if (!e.ok)
    {
      oss << " - " << e.kind << " host='" << e.host_ref << "' guest='" << e.guest_ref << "'\n";
      dump_error_lines(e);
      continue;
    }

    // 2) Numerical violations beyond tolerance
    const bool violated = (std::abs(e.ang_rad) > tol_ang_rad) || (std::abs(e.dist_m) > tol_dist_m);

    if (!violated)
      continue;

    oss << " - " << e.kind << " host='" << e.host_ref << "' guest='" << e.guest_ref << "'"
        << " ang=" << sodf::geometry::toDegrees(e.ang_rad) << "deg"
        << " dist=" << (e.dist_m * 1e3) << "mm"
        << "\n";
  }

  return oss.str();
}

}  // namespace systems
}  // namespace sodf
