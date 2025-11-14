#ifndef SODF_SYSTEMS_ORIGIN_LS_SOLVER_H_
#define SODF_SYSTEMS_ORIGIN_LS_SOLVER_H_

#include <Eigen/Geometry>
#include <sodf/database/database.h>
#include <sodf/components/origin.h>

namespace sodf {
namespace systems {

struct LSSolveParams
{
  double lambda = 1e-8;  // damping (Levenberg–Marquardt style)
  double w_ang = 1.0;    // weight for angular residuals
  double w_pos = 1.0;    // weight for positional residuals
};

struct LSLinearStats
{
  Eigen::Matrix<double, 6, 6> JTJ = Eigen::Matrix<double, 6, 6>::Zero();
  int rows = 0;  // total scalar residuals accumulated in this GN step
};

struct JacobianReport
{
  int rows = 0;                                            // total scalar residuals used this step
  int rank = 0;                                            // numerical rank(J)
  int nullity = 0;                                         // 6 - rank
  double condJ = std::numeric_limits<double>::infinity();  // ~sqrt(lmax/lmin_nonzero)
};

struct ResidualEntry
{
  std::string kind;      // "Concentric", "Parallel", "Coincident", "Distance", "Angle"
  std::string host_ref;  // pretty raw refs (already in your Ref)
  std::string guest_ref;
  double ang_rad = 0.0;  // angular residual (rad), meaning varies per kind (see below)
  double dist_m = 0.0;   // distance residual (m)
  bool ok = true;        // within tolerances?

  std::string error_msg;
};

/**
 * One Gauss–Newton least-squares step that updates the guest root pose to best satisfy
 * all constraints (Concentric/Parallel/Coincident/Distance/Angle) in OriginComponent.
 * SeatConeOnCylinder is intentionally skipped (use your contact solver).
 *
 * @param db   ECS database
 * @param map  Object → Entity map
 * @param eid  Entity to solve for (must have Transform+Origin)
 * @param origin the OriginComponent (constraints list)
 * @param T0   current local pose of the entity’s root frame (starting point)
 * @param P    solver params
 * @return     updated local pose T = exp(xi) * T0
 */
Eigen::Isometry3d solve_origin_least_squares_once(database::Database& db, const database::ObjectEntityMap& map,
                                                  database::EntityID eid, components::OriginComponent& origin,
                                                  const LSSolveParams& P = {}, LSLinearStats* out_stats = nullptr);

JacobianReport summarize_linear_system(const LSLinearStats& S);
void print_jacobian_report(const JacobianReport& j);

std::vector<ResidualEntry> compute_origin_residuals_compact(database::Database& db,
                                                            const database::ObjectEntityMap& map,
                                                            const components::OriginComponent& origin);
std::string format_origin_residual_errors(const std::vector<ResidualEntry>& residuals, double tol_ang_rad,
                                          double tol_dist_m);

}  // namespace systems
}  // namespace sodf

#endif  // SODF_SYSTEMS_ORIGIN_LS_SOLVER_H_
