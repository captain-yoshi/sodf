#include <sodf/database/database.h>

#include <sodf/systems/origin_ls_solver.h>
#include <sodf/systems/scene_graph.h>

#include <sodf/assembly/constraint_selector.h>
#include <sodf/assembly/namespace.h>
#include <sodf/assembly/selector_context_db.h>

#include <sodf/components/object.h>
#include <sodf/components/origin.h>
#include <sodf/components/origin_constraint.h>
#include <sodf/geometry/transform.h>
#include <sodf/geometry/frame.h>

#include <algorithm>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace sodf {
namespace systems {

namespace {

// Origin constraint tolerances
constexpr double kOriginAngTolRad = 1e-4;
constexpr double kOriginDistTolM = 5e-6;  // 5e-6 m

// Least-squares solver defaults
constexpr int kOriginMaxLsIters = 10;
constexpr double kOriginLsStepRotTol = 1e-10;    // rad, step-size emergency escape
constexpr double kOriginLsStepTransTol = 1e-10;  // m

constexpr double kOriginLsLambda = 1e-8;
constexpr double kOriginLsAngleWeight = 1.0;
constexpr double kOriginLsPositionWeight = 1.0;

// SeatCone discrete pass defaults
constexpr int kOriginMaxSeatConeIters = 20;
constexpr double kOriginSeatConeEps = 1e-9;

// Evaluate max angular and distance residual (over all constraints that are ok)
static void eval_origin_residuals(database::Database& db, const database::ObjectEntityMap& map,
                                  const components::OriginComponent& origin, double& max_ang_rad, double& max_dist_m)
{
  auto residuals = sodf::systems::compute_origin_residuals_compact(db, map, origin);

  max_ang_rad = 0.0;
  max_dist_m = 0.0;

  for (const auto& e : residuals)
  {
    if (!e.ok)
      continue;
    if (e.ang_rad > max_ang_rad)
      max_ang_rad = e.ang_rad;
    if (e.dist_m > max_dist_m)
      max_dist_m = e.dist_m;
  }
}

// Infer host_object from the first constraint that mentions a host.
static std::string infer_host_object(const components::OriginComponent& origin)
{
  std::string result;

  for (const auto& any : origin.constraints)
  {
    std::visit(
        [&](const auto& c) {
          using C = std::decay_t<decltype(c)>;
          std::string host_ref;

          if constexpr (std::is_same_v<C, components::Coincident>)
            host_ref = c.host;  // "table/shape/tabletop#frame"
          else if constexpr (std::is_same_v<C, components::Concentric>)
            host_ref = c.host;  // "table/insertion/A1#axis"
          else if constexpr (std::is_same_v<C, components::Parallel>)
            host_ref = c.host;
          else if constexpr (std::is_same_v<C, components::Angle>)
            host_ref = c.host;
          else if constexpr (std::is_same_v<C, components::Distance>)
            host_ref = c.host;
          else if constexpr (std::is_same_v<C, components::SeatConeOnCylinder>)
            host_ref = c.host_cyl;
          else if constexpr (std::is_same_v<C, components::Frame>)
            host_ref = c.host;
          else if constexpr (std::is_same_v<C, components::InsertionMate>)
            host_ref = c.host;

          if (!result.empty() || host_ref.empty())
            return;

          const auto slash = host_ref.find('/');
          if (slash != std::string::npos)
            result = host_ref.substr(0, slash);  // "table"
        },
        any);

    if (!result.empty())
      break;
  }

  return result;
}

// Optional two-pin presnap to give LS a good initial guess
static bool two_pin_presnap(database::Database& db, const database::ObjectEntityMap& map, database::EntityID eid,
                            components::OriginComponent& origin, Eigen::Isometry3d& T_world)
{
  using namespace sodf::components;
  using sodf::assembly::make_guest_ref;
  using sodf::assembly::make_host_ref;
  using sodf::assembly::Ref;
  using sodf::assembly::resolveAxis;

  // ---------------- Collect up to 2 "pin" pairs ----------------
  struct PinPair
  {
    std::string host;
    std::string guest;
  };

  PinPair pins[2];
  int n = 0;

  for (const auto& step_any : origin.constraints)
  {
    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, Concentric>)
          {
            if (n < 2)
            {
              pins[n].host = step.host;
              pins[n].guest = step.guest;
              ++n;
            }
          }
          else if constexpr (std::is_same_v<TStep, SeatConeOnCylinder>)
          {
            if (n < 2)
            {
              // Treat SeatCone like a pin between host_cyl and guest_cone
              pins[n].host = step.host_cyl;
              pins[n].guest = step.guest_cone;
              ++n;
            }
          }
        },
        step_any);
  }

  if (n != 2)
    return false;  // need exactly 2 pin-like constraints

  // ---------------- Temporarily write T_world into DB ----------------
  auto* tcomp = db.get<components::TransformComponent>(eid);
  if (!tcomp || tcomp->elements.empty())
    return false;

  auto& root_node = tcomp->elements.front().second;

  std::string parent_backup = root_node.parent;
  Eigen::Isometry3d local_backup = root_node.local;

  root_node.parent = {};  // WORLD
  root_node.local = T_world;
  root_node.dirty = true;
  sodf::systems::update_entity_global_transforms(db, eid, map);

  auto ctx = sodf::assembly::makeSelectorContext(db, map);

  // Resolve host/guest axes in WORLD for both pins
  Ref H1 = make_host_ref(pins[0].host, origin);
  Ref H2 = make_host_ref(pins[1].host, origin);
  Ref G1 = make_guest_ref(pins[0].guest, origin);
  Ref G2 = make_guest_ref(pins[1].guest, origin);

  auto aH1 = resolveAxis(H1, ctx);
  auto aH2 = resolveAxis(H2, ctx);
  auto aG1 = resolveAxis(G1, ctx);
  auto aG2 = resolveAxis(G2, ctx);

  // ---------------- Build 2D vectors (XY plane) ----------------
  Eigen::Vector3d vH = aH2.point - aH1.point;
  Eigen::Vector3d vG = aG2.point - aG1.point;

  Eigen::Vector2d vH2(vH.x(), vH.y());
  Eigen::Vector2d vG2(vG.x(), vG.y());

  if (vH2.norm() < 1e-6 || vG2.norm() < 1e-6)
  {
    // restore and bail
    root_node.parent = parent_backup;
    root_node.local = local_backup;
    root_node.dirty = true;
    sodf::systems::update_entity_global_transforms(db, eid, map);
    return false;
  }

  // ---------------- Yaw to align host and guest pin directions ----------------
  const double angH = std::atan2(vH2.y(), vH2.x());
  const double angG = std::atan2(vG2.y(), vG2.x());
  const double yaw = angH - angG;  // can be ~π in the rotated case

  Eigen::Isometry3d Rz = Eigen::Isometry3d::Identity();
  Rz.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // ---------------- Translation to align midpoints ----------------
  const Eigen::Vector3d mH = 0.5 * (aH1.point + aH2.point);
  const Eigen::Vector3d mG = 0.5 * (aG1.point + aG2.point);

  // After we apply Rz in WORLD, the guest midpoint moves to Rz * mG.
  const Eigen::Vector3d mG_rot = Rz * mG;
  const Eigen::Vector3d t = mH - mG_rot;

  Eigen::Isometry3d Delta = Eigen::Isometry3d::Identity();
  Delta.linear() = Rz.linear();
  Delta.translation() = t;

  // ---------------- Apply pre-snap: rotation + translation ----------------
  T_world = Delta * T_world;

  // ---------------- Restore node; LS will write T_world again ----------------
  root_node.parent = parent_backup;
  root_node.local = local_backup;
  root_node.dirty = true;
  sodf::systems::update_entity_global_transforms(db, eid, map);

  return true;
}

}  // namespace

static void expand_insertion_mates(database::Database& db, const database::ObjectEntityMap& map, database::EntityID eid,
                                   components::OriginComponent& origin)
{
  using namespace sodf::components;
  using ConstraintT = decltype(origin.constraints)::value_type;

  std::vector<ConstraintT> expanded;
  expanded.reserve(origin.constraints.size() * 3);

  auto ctx = sodf::assembly::makeSelectorContext(db, map);

  for (const auto& step_any : origin.constraints)
  {
    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, InsertionMate>)
          {
            // -----------------------------------------
            // 1) Always align insertion axes
            // -----------------------------------------
            expanded.emplace_back(Concentric{ step.host + "#axis", step.guest + "#axis" });

            // -----------------------------------------
            // Resolve insertion data
            // -----------------------------------------
            sodf::assembly::Ref host_ref = sodf::assembly::make_host_ref(step.host, origin);

            sodf::assembly::Ref guest_ref = sodf::assembly::make_guest_ref(step.guest, origin);

            sodf::assembly::SelectorContext::InsertionData H, G;

            if (!ctx.getInsertion(host_ref, H) || !ctx.getInsertion(guest_ref, G))
            {
              throw std::runtime_error("InsertionMate: failed to resolve insertion data");
            }

            const bool roles_differ = (H.role != G.role);

            // -----------------------------------------
            // 2) Axial constraint
            // -----------------------------------------
            double depth_value = 0.0;
            bool add_distance = false;

            switch (step.depth_mode)
            {
              case InsertionDepthMode::NONE:
              {
                add_distance = false;
                break;
              }

              case InsertionDepthMode::EXPLICIT:
              {
                add_distance = true;
                depth_value = step.depth;
                break;
              }

              case InsertionDepthMode::AUTO:
              {
                // Only meaningful for Insert<->Receptacle
                if (H.role == G.role)
                {
                  add_distance = false;
                  break;
                }

                // magnitude
                if (step.clamp_to_min_depth)
                  depth_value = std::min(H.max_depth, G.max_depth);
                else
                {
                  // choose receptacle's depth as the authoritative one
                  depth_value = (H.role == InsertionRole::Receptacle) ? H.max_depth : G.max_depth;
                }

                // sign so that motion is "into receptacle" along receptacle axis
                // If host is receptacle -> +depth along host axis (good)
                // If host is insert      -> -depth along host axis (because receptacle is guest)
                depth_value = (H.role == InsertionRole::Receptacle) ? +depth_value : -depth_value;

                // expanded.emplace_back(Distance{ step.host + "#axis", step.guest, signed_depth });
                add_distance = true;  // we already emitted it
                break;
              }
            }

            if (add_distance)
            {
              expanded.emplace_back(Distance{ step.host + "#axis", step.guest, depth_value });
            }

            // -----------------------------------------
            // 3) Reference axis alignment
            // -----------------------------------------
            if (step.align_reference_axis)
            {
              expanded.emplace_back(Parallel{ step.host + "#ref", step.guest + "#ref" });
            }
          }
          else
          {
            expanded.emplace_back(step_any);
          }
        },
        step_any);
  }

  origin.constraints = std::move(expanded);
}

// ------------------------ single-Origin solver -----------------------------

static void solve_single_origin(database::Database& db, const database::ObjectEntityMap& map, database::EntityID eid,
                                components::TransformComponent& tcomp, components::OriginComponent& origin)
{
  using namespace sodf::components;

  // ---------- Ensure a root slot exists ----------
  if (tcomp.elements.empty())
    tcomp.elements.emplace_back("root", geometry::TransformNode{});

  auto& root_pair = tcomp.elements.front();
  auto& root_node = root_pair.second;

  // ---------- Pin guest_object to THIS entity (throw if we cannot infer) ----------
  auto object_id_from_entity_or_empty = [&](database::EntityID q) -> std::string {
    for (const auto& kv : map)
      if (kv.second == q)
        return kv.first;
    return {};
  };

  if (origin.guest_object.empty() || map.find(origin.guest_object) == map.end() || map.at(origin.guest_object) != eid)
  {
    const std::string obj = object_id_from_entity_or_empty(eid);
    if (obj.empty())
      throw std::runtime_error("[Origin] Cannot infer guest_object for entity.");
    origin.guest_object = obj;
  }

  // Expand InsertionMate into primitives
  expand_insertion_mates(db, map, eid, origin);

  // host_object is already inferred in apply_origin_constraints (topology phase)

  // ---------- Establish baseline T0 in WORLD (initial guess) ----------
  Eigen::Isometry3d T0 = root_node.local;  // default baseline = current local (assumed world)
  std::string new_parent = root_node.parent;

  bool baseline_found = false;
  for (const auto& step_any : origin.constraints)
  {
    if (baseline_found)
      break;

    std::visit(
        [&](const auto& step) {
          using TStep = std::decay_t<decltype(step)>;

          if constexpr (std::is_same_v<TStep, sodf::geometry::Transform>)
          {
            // Absolute world placement; parent may be "" (WORLD)
            T0 = step.tf;
            new_parent = step.parent;
            baseline_found = true;
          }
        },
        step_any);
  }

  // Current world guess we will refine with LS and SeatCone passes
  Eigen::Isometry3d T_world = T0;

  // Optional presnap for the two-pin case (good initial guess)
  two_pin_presnap(db, map, eid, origin, T_world);

  // ---------- Solve (WORLD-only LS) ----------
  systems::LSSolveParams lp;
  lp.lambda = kOriginLsLambda;
  lp.w_ang = kOriginLsAngleWeight;
  lp.w_pos = kOriginLsPositionWeight;

  {
    // Temporarily treat the root as WORLD-parented during LS
    std::string ls_parent_backup = root_node.parent;
    Eigen::Isometry3d ls_local_backup = root_node.local;

    LSLinearStats stats{};
    const int max_ls_iters = kOriginMaxLsIters;

    // Step-size thresholds: emergency escape only
    const double rot_tol = kOriginLsStepRotTol;      // rad
    const double trans_tol = kOriginLsStepTransTol;  // m

    // Track best pose seen during LS (by residual)
    Eigen::Isometry3d T_best = T_world;
    double best_ang_rad = std::numeric_limits<double>::infinity();
    double best_dist_m = std::numeric_limits<double>::infinity();

    for (int it = 0; it < max_ls_iters; ++it)
    {
      // 1) Write current guess into DB as WORLD pose
      root_node.parent = {};  // WORLD
      root_node.local = T_world;
      root_node.dirty = true;
      sodf::systems::update_entity_global_transforms(db, eid, map);

      // 2) Evaluate residuals at THIS pose
      double max_ang = 0.0;
      double max_dist = 0.0;
      eval_origin_residuals(db, map, origin, max_ang, max_dist);

      // 3) Track best pose so far
      if (max_dist < best_dist_m || (max_dist == best_dist_m && max_ang < best_ang_rad))
      {
        best_dist_m = max_dist;
        best_ang_rad = max_ang;
        T_best = T_world;
      }

      // 4) Residual-based stopping: if already good enough, stop LS
      if (max_ang <= kOriginAngTolRad && max_dist <= kOriginDistTolM)
        break;

      // 5) One GN step around *current DB pose*
      Eigen::Isometry3d dT = systems::solve_origin_least_squares_once(db, map, eid, origin, lp, &stats);

      // 6) Apply increment (left-multiply)
      T_world = dT * T_world;

      // 7) Step-size based emergency escape: if updates are vanishingly small, bail
      Eigen::AngleAxisd aa(dT.linear());
      const double rot_norm = std::abs(aa.angle());
      const double trans_norm = dT.translation().norm();
      if (rot_norm < rot_tol && trans_norm < trans_tol)
        break;
    }

    // Restore best pose found during LS, not the last iterate
    T_world = T_best;

    // Restore whatever was there before LS; we'll overwrite it correctly below anyway
    root_node.parent = ls_parent_backup;
    root_node.local = ls_local_backup;
    root_node.dirty = true;
    sodf::systems::update_entity_global_transforms(db, eid, map);
  }

  // ---------- Discrete SeatCone pass *after* LS ----------
  {
    // Start from the LS solution
    Eigen::Isometry3d T_world_seat = T_world;

    // For seating, temporarily treat the root as world-parented
    std::string seat_parent_backup = root_node.parent;
    Eigen::Isometry3d local_backup = root_node.local;

    root_node.parent = {};  // WORLD
    root_node.local = T_world_seat;
    root_node.dirty = true;
    sodf::systems::update_entity_global_transforms(db, eid, map);

    bool has_seatcone = false;

    const int max_seat_iters = kOriginMaxSeatConeIters;
    const double seat_eps = kOriginSeatConeEps;

    // // --- First: apply all InsertionMate analytically ONCE ---
    // {
    //   auto ctx = sodf::assembly::makeSelectorContext(db, map);

    //   for (const auto& step_any : origin.constraints)
    //   {
    //     std::visit(
    //         [&](const auto& step) {
    //           using TStep = std::decay_t<decltype(step)>;

    //           if constexpr (std::is_same_v<TStep, components::InsertionMate>)
    //           {
    //             sodf::assembly::Ref H = sodf::assembly::make_host_ref(step.host, origin);
    //             sodf::assembly::Ref G = sodf::assembly::make_guest_ref(step.guest, origin);

    //             Eigen::Isometry3d dW = sodf::assembly::InsertionMate(H, G, step.depth, step.clamp_to_min_depth,
    //                                                                  step.align_reference_axis, ctx);

    //             if (!dW.isApprox(Eigen::Isometry3d::Identity(), 1e-12))
    //             {
    //               // Print before applying delta
    //               Eigen::Vector3d before = root_node.local.translation();

    //               std::cout << "\n--- InsertionMate step ---\n";
    //               std::cout << "Before world Z: " << before.z() << "\n";
    //               std::cout << "dW.translation: " << dW.translation().transpose() << "\n";

    //               // Apply delta
    //               T_world_seat = dW * T_world_seat;

    //               Eigen::Vector3d after = T_world_seat.translation();

    //               std::cout << "After world Z:  " << after.z() << "\n";
    //               std::cout << "Delta Z:        " << (after.z() - before.z()) << "\n";

    //               root_node.local = T_world_seat;
    //               root_node.dirty = true;
    //               sodf::systems::update_entity_global_transforms(db, eid, map);
    //             }
    //           }
    //         },
    //         step_any);
    //   }
    // }

    int iter = 0;
    for (;;)
    {
      bool changed = false;
      auto ctx = sodf::assembly::makeSelectorContext(db, map);

      for (const auto& step_any : origin.constraints)
      {
        std::visit(
            [&](const auto& step) {
              using TStep = std::decay_t<decltype(step)>;
              if constexpr (std::is_same_v<TStep, components::SeatConeOnCylinder>)
              {
                sodf::assembly::Ref H = sodf::assembly::make_host_ref(step.host_cyl, origin);
                sodf::assembly::Ref G = sodf::assembly::make_guest_ref(step.guest_cone, origin);

                Eigen::Isometry3d dW = sodf::assembly::SeatConeOnCylinder(H, G, ctx, step.tol, step.max_it);

                if (!dW.isApprox(Eigen::Isometry3d::Identity(), seat_eps))
                {
                  has_seatcone = true;
                  changed = true;

                  // Update running world pose
                  T_world_seat = dW * T_world_seat;

                  // Write back so subsequent SeatCone sees updated pose
                  root_node.local = T_world_seat;
                  root_node.dirty = true;
                  sodf::systems::update_entity_global_transforms(db, eid, map);
                }
              }
            },
            step_any);
      }

      if (!changed)
        break;  // all SeatCone constraints are satisfied up to seat_eps

      if (++iter >= max_seat_iters)
      {
        // --- 1) Put the guest at the *last* seated pose for diagnostics
        root_node.parent = {};           // WORLD for simplicity
        root_node.local = T_world_seat;  // last pose we tried
        root_node.dirty = true;
        sodf::systems::update_entity_global_transforms(db, eid, map);

        // --- 2) Compute residuals at this pose
        auto residuals = sodf::systems::compute_origin_residuals_compact(db, map, origin);

        std::ostringstream diag;
        for (const auto& e : residuals)
        {
          if (e.kind.find("SeatCone") == std::string::npos && e.kind.find("SeatConeOnCylinder") == std::string::npos)
            continue;

          diag << "   - " << e.kind << " host='" << e.host_ref << "'"
               << " guest='" << e.guest_ref << "'"
               << " ang=" << sodf::geometry::toDegrees(e.ang_rad) << "deg"
               << " dist=" << (e.dist_m * 1e3) << "mm\n";
        }

        // --- 3) Restore original pose before going back to caller
        root_node.parent = seat_parent_backup;
        root_node.local = local_backup;
        root_node.dirty = true;
        sodf::systems::update_entity_global_transforms(db, eid, map);

        // --- 4) Build error message
        std::ostringstream oss;
        oss << "[Origin] SeatCone discrete pass failed to converge after " << max_seat_iters
            << " iterations for guest_object='" << origin.guest_object << "'.\n"
            << "Likely inconsistent SeatConeOnCylinder constraints (e.g., multiple seats fighting each other).";

        if (!diag.str().empty())
        {
          oss << "\nDiagnostics (SeatCone residuals at last iteration):\n" << diag.str();
        }

        throw std::runtime_error(oss.str());
      }
    }

    if (has_seatcone)
    {
      // T_world becomes the seated pose
      T_world = T_world_seat;
    }

    // restore local/parent; we'll overwrite them correctly below anyway
    root_node.parent = seat_parent_backup;
    root_node.local = local_backup;
    root_node.dirty = true;
    sodf::systems::update_entity_global_transforms(db, eid, map);
  }

  // ---------- Re-express final world pose in host frame (if any) ----------
  Eigen::Isometry3d T_local = T_world;

  if (!origin.host_object.empty())
  {
    // Find host entity
    auto it = map.find(origin.host_object);
    if (it == map.end())
      throw std::runtime_error("[Origin] host_object '" + origin.host_object + "' not found in ObjectEntityMap");

    // Get host's global transform W_H
    const Eigen::Isometry3d W_H = sodf::systems::get_root_global_transform(db, map, origin.host_object);

    // Express guest pose in host frame
    T_local = W_H.inverse() * T_world;
    new_parent = origin.host_object;  // parent under host
  }
  else
  {
    new_parent.clear();  // WORLD
  }

  // ---------- Write back final local pose ----------
  root_node.local = T_local;
  root_node.parent = new_parent;
  root_node.dirty = true;

  sodf::systems::update_entity_global_transforms(db, eid, map);

  // ---------- Validate in WORLD; throw on failure (no rollback) ----------
  auto residuals = sodf::systems::compute_origin_residuals_compact(db, map, origin);

  std::size_t bad = 0;
  for (const auto& e : residuals)
  {
    bool violated = false;

    if (!e.ok)
    {
      violated = true;
    }
    else
    {
      const bool ang_bad = (std::abs(e.ang_rad) > kOriginAngTolRad);
      const bool dist_bad = (std::abs(e.dist_m) > kOriginDistTolM);
      violated = ang_bad || dist_bad;
    }

    if (violated)
      ++bad;
  }

  if (bad)
  {
    std::string why = sodf::systems::format_origin_residual_errors(residuals, kOriginAngTolRad, kOriginDistTolM);

    throw std::runtime_error("Origin solve violated " + std::to_string(bad) + " constraint(s) beyond tolerance:\n" +
                             why);
  }
}

// ------------------------ multi-Origin driver (topological) -----------------

namespace {

struct OriginNode
{
  database::EntityID eid;
  std::string object_id;
  std::string host_object;  // may be empty (WORLD or external)
};

}  // namespace

void apply_origin_constraints(database::Database& db, const database::ObjectEntityMap& obj_map)
{
  using EntityID = database::EntityID;
  using components::ObjectComponent;
  using components::OriginComponent;
  using components::TransformComponent;

  // ---------------- Collect all entities that have an Origin ----------------
  std::unordered_map<EntityID, OriginNode> nodes;
  std::unordered_map<EntityID, int> indegree;
  std::unordered_map<EntityID, std::vector<EntityID>> children;

  db.each([&](EntityID eid, ObjectComponent& obj, OriginComponent& origin) {
    if (origin.host_object.empty())
      origin.host_object = infer_host_object(origin);

    OriginNode node;
    node.eid = eid;
    node.object_id = obj.id;  // "thermal-cycler", "table", ...
    node.host_object = origin.host_object;

    nodes[eid] = node;
    indegree[eid] = 0;
  });

  if (nodes.empty())
    return;  // no Origin components at all

  // ---------------- Build dependency edges host -> guest --------------------
  for (auto& [eid, node] : nodes)
  {
    const std::string& host_id = node.host_object;
    if (host_id.empty())
      continue;

    auto it_host = obj_map.find(host_id);
    if (it_host == obj_map.end())
    {
      continue;
    }

    EntityID host_eid = it_host->second;

    auto it_node_host = nodes.find(host_eid);
    if (it_node_host == nodes.end())
      continue;

    children[host_eid].push_back(eid);
    indegree[eid] += 1;
  }

  // ---------------- Topological order (Kahn's algorithm) --------------------
  std::queue<EntityID> q;
  for (auto& [eid, deg] : indegree)
  {
    if (deg == 0)
      q.push(eid);
  }

  std::unordered_map<EntityID, std::string> last_error;
  std::size_t processed = 0;

  while (!q.empty())
  {
    EntityID eid = q.front();
    q.pop();
    ++processed;

    auto* tcomp = db.get<TransformComponent>(eid);
    auto* origin = db.get<OriginComponent>(eid);

    if (!tcomp || !origin)
    {
      last_error[eid] = "Missing TransformComponent or OriginComponent";
    }
    else
    {
      try
      {
        solve_single_origin(db, obj_map, eid, *tcomp, *origin);
        last_error.erase(eid);
      }
      catch (const std::exception& ex)
      {
        last_error[eid] = ex.what();
      }
    }

    auto it_children = children.find(eid);
    if (it_children != children.end())
    {
      for (EntityID child : it_children->second)
      {
        if (--indegree[child] == 0)
          q.push(child);
      }
    }
  }

  // ---------------- Summary of non-cycle failures -----------------------
  if (!last_error.empty())
  {
    std::ostringstream oss;
    oss << "[Origin] One or more Origin components failed to solve.\n";

    for (const auto& [eid, node] : nodes)
    {
      auto it_err = last_error.find(eid);
      if (it_err == last_error.end())
        continue;

      oss << " - object='" << node.object_id << "'";
      if (!node.host_object.empty())
        oss << " host='" << node.host_object << "'";
      oss << " error=\"" << it_err->second << "\"\n";
    }

    throw std::runtime_error(oss.str());
  }

  // ---------------- Detect cycles / unresolved dependencies -----------------
  if (processed != nodes.size())
  {
    std::ostringstream oss;
    oss << "[Origin] Could not solve all Origin components (likely cycle or missing hosts).\n";

    for (auto& [eid, node] : nodes)
    {
      if (indegree[eid] > 0)
      {
        oss << " - object='" << node.object_id << "'";
        if (!node.host_object.empty())
          oss << " host='" << node.host_object << "'";
        auto it_err = last_error.find(eid);
        if (it_err != last_error.end())
          oss << " last_error=\"" << it_err->second << "\"";
        oss << "\n";
      }
    }

    throw std::runtime_error(oss.str());
  }
}

}  // namespace systems
}  // namespace sodf
