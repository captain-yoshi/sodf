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
  const Eigen::Vector3d zg = Fg.linear().col(2).normalized();
  const double c = std::abs(n.dot(zg));                   // unsigned
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

}  // namespace

namespace sodf {
namespace systems {

using sodf::assembly::Axis;
using sodf::assembly::Plane;
using sodf::assembly::Pose;
using sodf::assembly::Ref;

using namespace sodf::geometry;

// ---------------- Jacobian diagnostics ----------------

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
                                                  const Eigen::Isometry3d& T0, const LSSolveParams& P,
                                                  LSLinearStats* out_stats)
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

  for (const auto& step_any : origin.constraints)
  {
    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, components::Concentric>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);

            Axis aH = sodf::assembly::resolveAxis(H, ctx);
            Axis aG = sodf::assembly::resolveAxis(G, ctx);

            const Eigen::Vector3d h = aH.direction.normalized();
            const Eigen::Vector3d g = aG.direction.normalized();
            const Eigen::Vector3d Pg = aG.point;
            const Eigen::Vector3d Ph = aH.point;

            // angular residual
            Eigen::Vector3d r_ang = g.cross(h);
            Eigen::Matrix<double, 3, 6> J_ang = Eigen::Matrix<double, 3, 6>::Zero();
            J_ang.block<3, 3>(0, 0) = -skew(h) * skew(g);
            accum(J_ang, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);

            // position residual (orthogonal to host axis)
            Eigen::Matrix3d N = proj_perp(h);
            Eigen::Vector3d r_pos = N * (Pg - Ph);
            Eigen::Matrix<double, 3, 6> J_pos = Eigen::Matrix<double, 3, 6>::Zero();
            J_pos.block<3, 3>(0, 0) = N * skew(Pg);
            J_pos.block<3, 3>(0, 3) = N;
            accum(J_pos, Eigen::Map<Eigen::Vector3d>(r_pos.data()), P.w_pos);
          }
          else if constexpr (std::is_same_v<TStep, components::Parallel>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);

            Axis aH = sodf::assembly::resolveAxis(H, ctx);
            Axis aG = sodf::assembly::resolveAxis(G, ctx);

            const Eigen::Vector3d h = aH.direction.normalized();
            const Eigen::Vector3d g = aG.direction.normalized();
            Eigen::Vector3d r_ang = g.cross(h);
            Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
            J.block<3, 3>(0, 0) = -skew(h) * skew(g);
            accum(J, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);
          }
          else if constexpr (std::is_same_v<TStep, components::Coincident>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            bool done = false;
            try
            {
              Plane Hp = sodf::assembly::resolvePlane(H, ctx);
              Pose Fg = sodf::assembly::resolvePose(G, ctx);
              const Eigen::Vector3d n = Hp.normal.normalized();
              const Eigen::Vector3d xg = Fg.translation();

              double rd = n.dot(xg - Hp.point);
              Eigen::Matrix<double, 1, 6> Jd = Eigen::Matrix<double, 1, 6>::Zero();
              Jd.block<1, 3>(0, 0) = n.transpose() * skew(xg);
              Jd.block<1, 3>(0, 3) = n.transpose();
              accum(Jd, Eigen::VectorXd::Constant(1, rd), P.w_pos);

              Eigen::Vector3d zg = Fg.linear().col(2);
              Eigen::Vector3d r_ang = zg.cross(n);
              Eigen::Matrix<double, 3, 6> Ja = Eigen::Matrix<double, 3, 6>::Zero();
              Ja.block<3, 3>(0, 0) = -skew(n) * skew(zg);
              accum(Ja, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);
              done = true;
            }
            catch (...)
            {
            }

            if (!done)
            {
              try
              {
                Pose Fh = sodf::assembly::resolvePose(H, ctx);
                Pose Fg = sodf::assembly::resolvePose(G, ctx);
                Plane Hp{ Fh.translation(), Fh.linear().col(2) };
                const Eigen::Vector3d n = Hp.normal.normalized();
                const Eigen::Vector3d xg = Fg.translation();

                double rd = n.dot(xg - Hp.point);
                Eigen::Matrix<double, 1, 6> Jd = Eigen::Matrix<double, 1, 6>::Zero();
                Jd.block<1, 3>(0, 0) = n.transpose() * skew(xg);
                Jd.block<1, 3>(0, 3) = n.transpose();
                accum(Jd, Eigen::VectorXd::Constant(1, rd), P.w_pos);

                Eigen::Vector3d zg = Fg.linear().col(2);
                Eigen::Vector3d r_ang = zg.cross(n);
                Eigen::Matrix<double, 3, 6> Ja = Eigen::Matrix<double, 3, 6>::Zero();
                Ja.block<3, 3>(0, 0) = -skew(n) * skew(zg);
                accum(Ja, Eigen::Map<Eigen::Vector3d>(r_ang.data()), P.w_ang);
              }
              catch (...)
              {
              }
            }
          }
          else if constexpr (std::is_same_v<TStep, components::Distance>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            bool added = false;
            try
            {
              Plane Hp = sodf::assembly::resolvePlane(H, ctx);
              Pose Fg = sodf::assembly::resolvePose(G, ctx);
              const Eigen::Vector3d n = Hp.normal.normalized();
              const Eigen::Vector3d xg = Fg.translation();
              double rd = n.dot(xg - Hp.point) - step.value;
              Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
              J.block<1, 3>(0, 0) = n.transpose() * skew(xg);
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
                Axis Ha = sodf::assembly::resolveAxis(H, ctx);
                Pose Fg = sodf::assembly::resolvePose(G, ctx);
                const Eigen::Vector3d u = Ha.direction.normalized();
                const Eigen::Vector3d xg = Fg.translation();
                double rd = (xg - Ha.point).dot(u) - step.value;
                Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
                J.block<1, 3>(0, 0) = u.transpose() * skew(xg);
                J.block<1, 3>(0, 3) = u.transpose();
                accum(J, Eigen::VectorXd::Constant(1, rd), P.w_pos);
              }
              catch (...)
              {
              }
            }
          }
          else if constexpr (std::is_same_v<TStep, components::Angle>)
          {
            Ref H = sodf::assembly::make_host_ref(step.host, origin);
            Ref G = sodf::assembly::make_guest_ref(step.guest, origin);
            Axis aH = sodf::assembly::resolveAxis(H, ctx);
            Axis aG = sodf::assembly::resolveAxis(G, ctx);
            const Eigen::Vector3d h = aH.direction.normalized();
            const Eigen::Vector3d g = aG.direction.normalized();
            const double cdes = std::cos(step.radians);
            const double r = g.dot(h) - cdes;  // scalar residual
            Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
            J.block<1, 3>(0, 0) = (g.cross(h)).transpose();
            accum(J, Eigen::VectorXd::Constant(1, r), P.w_ang);
          }
          // SeatConeOnCylinder and geometry::Transform / Align* are handled elsewhere
        },
        step_any);
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
  Eigen::Isometry3d dT = expSE3(xi);
  return dT * T0;
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

          if constexpr (std::is_same_v<TStep, components::Concentric>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Concentric", to_ref_str(H), to_ref_str(G) };
            try
            {
              auto aH = resolveAxis(H, ctx);
              auto aG = resolveAxis(G, ctx);
              auto R = concentric_residual(aH, aG);
              e.ang_rad = R.angle_rad;
              e.dist_m = R.dist_m;
            }
            catch (...)
            {
              e.ok = false;
            }
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Parallel>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Parallel", to_ref_str(H), to_ref_str(G) };
            try
            {
              auto aH = resolveAxis(H, ctx);
              auto aG = resolveAxis(G, ctx);
              auto R = parallel_residual(aH, aG);
              e.ang_rad = R.angle_rad;
              e.dist_m = 0.0;
            }
            catch (...)
            {
              e.ok = false;
            }
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Coincident>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Coincident", to_ref_str(H), to_ref_str(G) };
            bool ok = false;
            try
            {
              auto Hp = resolvePlane(H, ctx);
              auto Fg = resolvePose(G, ctx);
              auto R = plane_frame_residual(Hp, Fg);
              e.ang_rad = R.angle_rad;
              e.dist_m = R.dist_m;
              ok = true;
            }
            catch (...)
            {
            }
            if (!ok)
            {
              try
              {
                auto Fh = resolvePose(H, ctx);
                auto Fg = resolvePose(G, ctx);
                Plane Hp{ Fh.translation(), Fh.linear().col(2) };
                auto R = plane_frame_residual(Hp, Fg);
                e.ang_rad = R.angle_rad;
                e.dist_m = R.dist_m;
                ok = true;
              }
              catch (...)
              {
              }
            }
            e.ok = ok;
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Distance>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Distance", to_ref_str(H), to_ref_str(G) };
            bool ok = false;
            try
            {
              auto Hp = resolvePlane(H, ctx);
              auto Fg = resolvePose(G, ctx);
              auto R = plane_frame_residual(Hp, Fg);
              e.ang_rad = R.angle_rad;
              const Eigen::Vector3d n = Hp.normal.normalized();
              const double have = n.dot(Fg.translation() - Hp.point);
              e.dist_m = std::abs(have - step.value);
              ok = true;
            }
            catch (...)
            {
            }
            if (!ok)
            {
              try
              {
                auto Ha = resolveAxis(H, ctx);
                auto Fg = resolvePose(G, ctx);
                auto R = axis_frame_distance_residual(Ha, Fg, step.value);
                e.ang_rad = 0.0;
                e.dist_m = R.dist_m;
                ok = true;
              }
              catch (...)
              {
              }
            }
            e.ok = ok;
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::Angle>)
          {
            Ref H = assembly::make_host_ref(step.host, origin);
            Ref G = assembly::make_guest_ref(step.guest, origin);
            ResidualEntry e{ "Angle", to_ref_str(H), to_ref_str(G) };
            try
            {
              auto aH = resolveAxis(H, ctx);
              auto aG = resolveAxis(G, ctx);
              const double have = angle_between(aH.direction, aG.direction);
              e.ang_rad = std::abs(have - step.radians);
              e.dist_m = 0.0;
            }
            catch (...)
            {
              e.ok = false;
            }
            out.push_back(std::move(e));
          }
          else if constexpr (std::is_same_v<TStep, components::SeatConeOnCylinder>)
          {
            Ref H = assembly::make_host_ref(step.host_cyl, origin);
            Ref G = assembly::make_guest_ref(step.guest_cone, origin);
            ResidualEntry e{ "SeatCone", to_ref_str(H), to_ref_str(G) };
            e.ok = true;  // neutral
            out.push_back(std::move(e));
          }
        },
        step_any);
  }

  return out;
}

void print_residual_line(const ResidualEntry& e, double tol_ang, double tol_dist)
{
  const bool ok = (e.ang_rad <= tol_ang) && (e.dist_m <= tol_dist);
  std::cout << "  [" << (ok ? "OK " : "BAD") << "] " << std::left << std::setw(11) << e.kind << " host='" << e.host_ref
            << "'  guest='" << e.guest_ref << "'"
            << "  ang=" << std::fixed << std::setprecision(4) << sodf::geometry::toDegrees(e.ang_rad) << "deg"
            << "  dist=" << std::setprecision(6) << e.dist_m * 1e3 << "mm"
            << "\n";
}

}  // namespace systems
}  // namespace sodf
