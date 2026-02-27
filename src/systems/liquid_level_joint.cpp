#include <sodf/systems/liquid_level_joint.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include <sodf/components/container.h>
#include <sodf/components/transform.h>
#include <sodf/components/joint.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/shape.h>

namespace sodf {
namespace systems {

namespace {

using EntityID = database::EntityID;

// Utility: fetch an element value via diff, failing with a good error.
// Assumes DatabaseDiff::get_element<C>(eid, key) returns the RESOLVED definition
// type for DefOrRef-backed components (per your refactor).
template <class C, class Key>
static const auto& require_elem_const(const database::DatabaseDiff& diff, EntityID eid, const Key& key,
                                      const std::string& what)
{
  const auto* v = diff.template get_element<C>(eid, key);
  if (!v)
  {
    std::ostringstream os;
    os << "[liquid_level/diff] " << what << " '" << key << "' not found on entity.";
    throw std::runtime_error(os.str());
  }
  return *v;
}

// Utility: mark a transform frame dirty via diff patching
static void mark_tf_dirty(database::DatabaseDiff& diff, EntityID eid, const std::string& frame_id)
{
  if (frame_id.empty())
    return;

  const auto* tf = diff.get_element<components::TransformComponent>(eid, frame_id);
  if (!tf)
  {
    std::ostringstream os;
    os << "[liquid_level/diff] Transform frame '" << frame_id << "' not found on entity.";
    throw std::runtime_error(os.str());
  }

  auto tf_copy = *tf;
  tf_copy.dirty = true;

  diff.add_or_replace<components::TransformComponent>(eid, frame_id, tf_copy);
}

}  // namespace

// -----------------------------------------------------------------------------
// Base-db version
//
// Updated for new Container schema + new XML convention:
//
// 1) PayloadDomain no longer has frame_id.
// 2) The Transform frame for the payload domain is keyed by
//    c.payload.domain_shape_id (your DomainShapeRef instance id).
//
// Assumes Database::get_element<StackedShapeComponent>(eid, id) returns a
// RESOLVED geometry::StackedShape* even though storage is DefOrRef.
// -----------------------------------------------------------------------------
void update_liquid_level_joints_from_domain(database::Database& db, Eigen::Vector3d& gravity_world)
{
  db.each([&](database::EntityID eid, components::ContainerComponent& containers, components::TransformComponent& tcomp,
              components::JointComponent& jcomp, components::DomainShapeComponent& dcomp,
              components::StackedShapeComponent& /*scomp*/) {
    for (auto& [name, c] : containers.elements)
    {
      auto* fluid = std::get_if<components::FluidDomain>(&c.runtime);
      if (!fluid)
      {
        continue;
      }

      if (c.payload.volume <= 0.0 || c.payload.domain_shape_id.empty() || fluid->liquid_level_joint_id.empty())
      {
        continue;
      }

      // DomainShape (instance id)
      physics::DomainShape* dom = database::get_element(dcomp.elements, c.payload.domain_shape_id);
      if (!dom)
      {
        std::ostringstream os;
        os << "[liquid_level] DomainShape '" << c.payload.domain_shape_id << "' not found on entity for container '"
           << name << "'.";
        throw std::runtime_error(os.str());
      }

      // Joint
      auto* joint = database::get_element(jcomp.elements, fluid->liquid_level_joint_id);
      if (!joint)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << fluid->liquid_level_joint_id << "' not found for container '" << name << "'.";
        throw std::runtime_error(os.str());
      }
      if (joint->actuation != components::JointActuation::VIRTUAL)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << fluid->liquid_level_joint_id << "' must be VIRTUAL (is "
           << components::jointActuationToString(joint->actuation) << ").";
        throw std::runtime_error(os.str());
      }
      if (joint->type != components::JointType::PRISMATIC || joint->dof() != 1)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << fluid->liquid_level_joint_id << "' must be 1-DOF PRISMATIC.";
        throw std::runtime_error(os.str());
      }

      // payload/domain frame is keyed by domain_shape_id
      auto* tf_payload = database::get_element(tcomp.elements, c.payload.domain_shape_id);
      if (!tf_payload)
      {
        std::ostringstream os;
        os << "[liquid_level] DomainShape frame '" << c.payload.domain_shape_id << "' not found for container '" << name
           << "'. (Expected Transform element with the same id as payload.domain_shape_id)";
        throw std::runtime_error(os.str());
      }

      Eigen::Vector3d p_base_world = tf_payload->global.translation();

      physics::FillEnv env;
      env.T_world_domain = tf_payload->global;
      env.p_base_world = p_base_world;
      env.g_down_world = gravity_world.normalized();

      const double tol = 1e-15;
      double h = 0.0;

      const bool tilted = !dom->canUseAnalyticNow(env);

      // Build mesh cache if needed
      if (tilted && !dom->hasMesh())
      {
        if (dom->stacked_shape_id.empty())
        {
          std::ostringstream os;
          os << "[liquid_level] DomainShape '" << c.payload.domain_shape_id
             << "' requires mesh but has no stacked_shape_id.";
          throw std::runtime_error(os.str());
        }

        // IMPORTANT: use database API (resolved DefOrRef)
        auto* stack = db.get_element<components::StackedShapeComponent>(eid, dom->stacked_shape_id);
        if (!stack)
        {
          std::ostringstream os;
          os << "[liquid_level] StackedShape '" << dom->stacked_shape_id << "' not found for DomainShape '"
             << c.payload.domain_shape_id << "'.";
          throw std::runtime_error(os.str());
        }

        dom->setMeshCacheFromStack(*stack, /*radial_res=*/64, /*axial_res_spherical=*/16, /*weld_tol=*/1e-9);
      }

      double h_world = 0.0;
      try
      {
        h_world = dom->heightFromVolume(c.payload.volume, env, tol);
      }
      catch (const std::exception& e)
      {
        std::ostringstream os;
        os << "[liquid_level] heightFromVolume failed for domain '" << c.payload.domain_shape_id << "': " << e.what();
        throw std::runtime_error(os.str());
      }

      if (!tilted)
      {
        double hmax_world = 0.0;
        try
        {
          hmax_world = dom->maxFillHeight(env);
        }
        catch (...)
        {
        }
        h = (hmax_world > 0.0) ? std::clamp(h_world, 0.0, hmax_world) : h_world;
      }
      else
      {
        const Eigen::Vector3d g_up = -env.g_down_world.normalized();
        const Eigen::Vector3d axis_world = dom->heightAxisWorld(env);
        const double cos_theta = axis_world.dot(g_up);
        const double eps = 1e-12;

        if (std::abs(cos_theta) < eps)
        {
          std::ostringstream os;
          os << "[liquid_level] Domain height axis nearly perpendicular to gravity for container '" << name
             << "'; cannot map vertical height to axis displacement.";
          throw std::runtime_error(os.str());
        }

        double h_axis = h_world / cos_theta;

        double hmax_world = 0.0;
        try
        {
          hmax_world = dom->maxFillHeight(env);
        }
        catch (...)
        {
        }

        if (hmax_world > 0.0)
        {
          const double hmax_axis = hmax_world / cos_theta;
          if (hmax_axis >= 0.0)
            h_axis = std::clamp(h_axis, 0.0, hmax_axis);
          else
            h_axis = std::clamp(h_axis, hmax_axis, 0.0);
        }

        h = h_axis;
      }

      // Clamp to joint hard limits
      if (joint->limit.min_position.size() == 1 && joint->limit.max_position.size() == 1)
      {
        const double jmin = joint->limit.min_position[0];
        const double jmax = joint->limit.max_position[0];
        if (jmax > jmin)
          h = std::clamp(h, jmin, jmax);
      }

      if (joint->position.size() != 1)
        joint->resize(1);
      joint->position[0] = h;

      // Mark dependent frames dirty
      if (!fluid->liquid_level_frame_id.empty())
      {
        if (auto* tf_ll = database::get_element(tcomp.elements, fluid->liquid_level_frame_id))
          tf_ll->dirty = true;
        else
        {
          std::ostringstream os;
          os << "[liquid_level] liquid_level_frame_id '" << fluid->liquid_level_frame_id
             << "' not found for container '" << name << "'.";
          throw std::runtime_error(os.str());
        }
      }

      if (auto* tf_joint = database::get_element(tcomp.elements, fluid->liquid_level_joint_id))
        tf_joint->dirty = true;
      else
      {
        std::ostringstream os;
        os << "[liquid_level] transform for joint '" << fluid->liquid_level_joint_id << "' not found for container '"
           << name << "'.";
        throw std::runtime_error(os.str());
      }
    }
  });
}

// -----------------------------------------------------------------------------
// Diff-safe version
//
// Updated for new Container schema + new XML convention.
//
// Assumes DatabaseDiff::get_element<StackedShapeComponent>(eid, id) returns a
// RESOLVED geometry::StackedShape* even though storage is DefOrRef.
// -----------------------------------------------------------------------------
void update_liquid_level_joints_from_domain(database::DatabaseDiff& diff, Eigen::Vector3d& gravity_world)
{
  using components::ContainerComponent;
  using components::DomainShapeComponent;
  using components::JointComponent;
  using components::StackedShapeComponent;
  using components::TransformComponent;

  diff.each([&](EntityID eid, const ContainerComponent& containers, const TransformComponent& /*tcomp*/,
                const JointComponent& /*jcomp*/, const DomainShapeComponent& /*dcomp*/,
                const StackedShapeComponent& /*scomp*/) {
    for (const auto& kv : containers.elements)
    {
      const std::string& name = kv.first;
      const auto& c = kv.second;

      const auto* fluid = std::get_if<components::FluidDomain>(&c.runtime);
      if (!fluid)
      {
        continue;
      }

      if (c.payload.volume <= 0.0 || c.payload.domain_shape_id.empty() || fluid->liquid_level_joint_id.empty())
      {
        continue;
      }

      // ---------------- Resolve immutable inputs through the diff ----------------

      const auto& dom_base =
          require_elem_const<DomainShapeComponent>(diff, eid, c.payload.domain_shape_id, "DomainShape");

      const auto& joint_base = require_elem_const<JointComponent>(diff, eid, fluid->liquid_level_joint_id, "Joint");

      if (joint_base.actuation != components::JointActuation::VIRTUAL)
      {
        std::ostringstream os;
        os << "[liquid_level/diff] Joint '" << fluid->liquid_level_joint_id << "' must be VIRTUAL (is "
           << components::jointActuationToString(joint_base.actuation) << ").";
        throw std::runtime_error(os.str());
      }
      if (joint_base.type != components::JointType::PRISMATIC || joint_base.dof() != 1)
      {
        std::ostringstream os;
        os << "[liquid_level/diff] Joint '" << fluid->liquid_level_joint_id << "' must be 1-DOF PRISMATIC.";
        throw std::runtime_error(os.str());
      }

      // NEW: payload/domain frame is keyed by domain_shape_id
      const auto& tf_payload =
          require_elem_const<TransformComponent>(diff, eid, c.payload.domain_shape_id, "DomainShape frame");

      Eigen::Vector3d p_base_world = tf_payload.global.translation();

      physics::FillEnv env;
      env.T_world_domain = tf_payload.global;
      env.p_base_world = p_base_world;
      env.g_down_world = gravity_world.normalized();

      const double tol = 1e-15;

      // Work on a copy of DomainShape
      auto dom_work = dom_base;

      const bool tilted = !dom_work.canUseAnalyticNow(env);

      if (tilted && !dom_work.hasMesh())
      {
        if (dom_work.stacked_shape_id.empty())
        {
          std::ostringstream os;
          os << "[liquid_level/diff] DomainShape '" << c.payload.domain_shape_id
             << "' requires mesh but has no stacked_shape_id.";
          throw std::runtime_error(os.str());
        }

        // IMPORTANT: rely on diff API (resolved DefOrRef)
        const auto* stack = diff.get_element<StackedShapeComponent>(eid, dom_work.stacked_shape_id);
        if (!stack)
        {
          std::ostringstream os;
          os << "[liquid_level/diff] StackedShape '" << dom_work.stacked_shape_id << "' not found for DomainShape '"
             << c.payload.domain_shape_id << "'.";
          throw std::runtime_error(os.str());
        }

        dom_work.setMeshCacheFromStack(*stack, /*radial_res=*/64, /*axial_res_spherical=*/16, /*weld_tol=*/1e-9);

        // Patch updated DomainShape (mesh cache) into the diff
        diff.add_or_replace<DomainShapeComponent>(eid, c.payload.domain_shape_id, dom_work);
      }

      double h_world = 0.0;
      try
      {
        h_world = dom_work.heightFromVolume(c.payload.volume, env, tol);
      }
      catch (const std::exception& e)
      {
        std::ostringstream os;
        os << "[liquid_level/diff] heightFromVolume failed for domain '" << c.payload.domain_shape_id
           << "': " << e.what();
        throw std::runtime_error(os.str());
      }

      double h = 0.0;

      if (!tilted)
      {
        double hmax_world = 0.0;
        try
        {
          hmax_world = dom_work.maxFillHeight(env);
        }
        catch (...)
        {
        }
        h = (hmax_world > 0.0) ? std::clamp(h_world, 0.0, hmax_world) : h_world;
      }
      else
      {
        const Eigen::Vector3d g_up = -env.g_down_world.normalized();
        const Eigen::Vector3d axis_world = dom_work.heightAxisWorld(env);
        const double cos_theta = axis_world.dot(g_up);
        const double eps = 1e-12;

        if (std::abs(cos_theta) < eps)
        {
          std::ostringstream os;
          os << "[liquid_level/diff] Domain height axis nearly perpendicular to gravity for container '" << name
             << "'; cannot map vertical height to axis displacement.";
          throw std::runtime_error(os.str());
        }

        double h_axis = h_world / cos_theta;

        double hmax_world = 0.0;
        try
        {
          hmax_world = dom_work.maxFillHeight(env);
        }
        catch (...)
        {
        }

        if (hmax_world > 0.0)
        {
          const double hmax_axis = hmax_world / cos_theta;
          if (hmax_axis >= 0.0)
            h_axis = std::clamp(h_axis, 0.0, hmax_axis);
          else
            h_axis = std::clamp(h_axis, hmax_axis, 0.0);
        }

        h = h_axis;
      }

      // Joint hard limits
      if (joint_base.limit.min_position.size() == 1 && joint_base.limit.max_position.size() == 1)
      {
        const double jmin = joint_base.limit.min_position[0];
        const double jmax = joint_base.limit.max_position[0];
        if (jmax > jmin)
          h = std::clamp(h, jmin, jmax);
      }

      // ---------------- Patch the joint element ----------------

      auto joint_work = joint_base;
      if (joint_work.position.size() != 1)
        joint_work.resize(1);
      joint_work.position[0] = h;

      diff.add_or_replace<JointComponent>(eid, fluid->liquid_level_joint_id, joint_work);

      // ---------------- Patch dirty flags on relevant frames ----------------

      if (!fluid->liquid_level_frame_id.empty())
        mark_tf_dirty(diff, eid, fluid->liquid_level_frame_id);

      mark_tf_dirty(diff, eid, fluid->liquid_level_joint_id);
    }
  });
}

}  // namespace systems
}  // namespace sodf
