// src/systems/origin_constraints.cpp
#include <sodf/systems/origin_ls_solver.h>
#include <sodf/systems/scene_graph.h>

#include <sodf/assembly/constraint_selector.h>
#include <sodf/assembly/namespace.h>
#include <sodf/assembly/selector_context_db.h>

#include <sodf/components/origin.h>
#include <sodf/components/origin_constraint.h>
#include <sodf/geometry/transform.h>
#include <sodf/geometry/frame.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace sodf {
namespace systems {

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

void apply_origin_constraints(database::Database& db, const database::ObjectEntityMap& map)
{
  using namespace sodf::components;

  db.each([&](database::EntityID eid, TransformComponent& tcomp, OriginComponent& origin) {
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

    if (origin.host_object.empty())
    {
      origin.host_object = infer_host_object(origin);
    }

    // ---------- Establish baseline T0 in WORLD ----------
    // Empty parent is fine and means WORLD.
    Eigen::Isometry3d T0 = root_node.local;  // default baseline = current local
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
            else if constexpr (std::is_same_v<TStep, sodf::geometry::AlignFrames>)
            {
              // Align a local frame of THIS entity to a frame on another object.
              const auto Fsrc = sodf::systems::get_local_to_root(db, eid, step.source_tf);

              auto it = map.find(step.target_id);
              if (it == map.end())
                throw std::runtime_error("[Origin] AlignFrames target '" + step.target_id + "' not found.");

              const auto Ftgt = sodf::systems::get_local_to_root(db, it->second, step.target_tf);

              T0 = Ftgt * Fsrc.inverse();   // WORLD baseline
              new_parent = step.target_id;  // parent under the target object (non-empty)
              baseline_found = true;
            }
            else if constexpr (std::is_same_v<TStep, sodf::geometry::AlignPairFrames>)
            {
              auto it = map.find(step.target_id);
              if (it == map.end())
                throw std::runtime_error("[Origin] AlignPairFrames target '" + step.target_id + "' not found.");

              const auto S1 = sodf::systems::get_local_to_root(db, eid, step.source_tf1);
              const auto S2 = sodf::systems::get_local_to_root(db, eid, step.source_tf2);
              const auto T1 = sodf::systems::get_local_to_root(db, it->second, step.target_tf1);
              const auto T2 = sodf::systems::get_local_to_root(db, it->second, step.target_tf2);

              T0 = sodf::geometry::alignCenterFrames(T1, T2, S1, S2, step.tolerance);  // WORLD baseline
              new_parent = step.target_id;  // parent under the target object (non-empty)
              std::cout << "new local parent = " << new_parent << std::endl;
              baseline_found = true;
            }
          },
          step_any);
    }

    // ---------- Solve (WORLD-only LS) ----------
    systems::LSSolveParams lp;
    lp.lambda = 1e-8;
    lp.w_ang = 1.0;
    lp.w_pos = 1.0;

    LSLinearStats stats{};
    Eigen::Isometry3d T_world = systems::solve_origin_least_squares_once(db, map, eid, origin, T0, lp, &stats);

    Eigen::Isometry3d T_local = T_world;

    if (!origin.host_object.empty())
    {
      // Find host entity
      auto it = map.find(origin.host_object);
      if (it == map.end())
        throw std::runtime_error("[Origin] host_object '" + origin.host_object + "' not found in ObjectEntityMap");

      const auto host_eid = it->second;

      // Get host's global transform W_H (use whatever helper you already have)
      // e.g. something like:
      const Eigen::Isometry3d W_H = sodf::systems::get_root_global_transform(db, map, origin.host_object);

      // Express guest pose in host frame
      T_local = W_H.inverse() * T_world;
      new_parent = origin.host_object;  // parent under host
    }
    else
    {
      new_parent.clear();  // WORLD
    }

    // Write back
    root_node.local = T_local;
    root_node.parent = new_parent;
    root_node.dirty = true;

    sodf::systems::update_entity_global_transforms(db, eid, map);

    // ---------- Validate in WORLD; throw on failure (no rollback) ----------
    const double kAngTolRad = sodf::geometry::toRadians(0.00573);  // ~0.00573 deg
    const double kDistTolM = 0.005e-3;                             // 0.005 mm

    auto residuals = sodf::systems::compute_origin_residuals_compact(db, map, origin);

    std::size_t bad = 0;
    std::ostringstream why;
    for (const auto& e : residuals)
    {
      const bool ok = e.ok && (e.ang_rad <= kAngTolRad) && (e.dist_m <= kDistTolM);
      if (!ok)
      {
        ++bad;
        why << " - " << e.kind << " host='" << e.host_ref << "' guest='" << e.guest_ref
            << "' ang=" << sodf::geometry::toDegrees(e.ang_rad) << "deg"
            << " dist=" << (e.dist_m * 1e3) << "mm\n";
      }
    }

    if (bad)
    {
      // Leave the current (bad) pose as-is so the user can inspect it, but signal failure.
      throw std::runtime_error("Origin solve violated " + std::to_string(bad) + " constraint(s) beyond tolerance:\n" +
                               why.str());
    }
  });
}

}  // namespace systems
}  // namespace sodf
