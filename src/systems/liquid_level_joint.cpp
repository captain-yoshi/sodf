#include <sodf/systems/liquid_level_joint.h>

#include <algorithm>
#include <sstream>
#include <iostream>

namespace sodf {
namespace systems {

void update_liquid_level_joints_from_domain(database::Database& db, Eigen::Vector3d& gravity_world)
{
  db.each([&](database::EntityID eid, components::ContainerComponent& containers, components::TransformComponent& tcomp,
              components::JointComponent& jcomp, components::DomainShapeComponent& dcomp,
              components::StackedShapeComponent& scomp) {
    for (auto& [name, c] : containers.elements)
    {
      // Skip benign cases
      if (c.payload.volume <= 0.0 || c.payload.domain_shape_id.empty() || c.fluid.liquid_level_joint_id.empty() ||
          c.payload.frame_id.empty())
      {
        continue;
      }

      // Resolve domain shape
      physics::DomainShape* dom = database::get_element(dcomp.elements, c.payload.domain_shape_id);
      if (!dom)
      {
        std::ostringstream os;
        os << "[liquid_level] DomainShape '" << c.payload.domain_shape_id << "' not found on entity for container '"
           << name << "'.";
        throw std::runtime_error(os.str());
      }

      // Resolve joint + validate actuation/type/DOF
      auto* joint = database::get_element(jcomp.elements, c.fluid.liquid_level_joint_id);
      if (!joint)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << c.fluid.liquid_level_joint_id << "' not found for container '" << name
           << "'.";
        throw std::runtime_error(os.str());
      }
      if (joint->actuation != components::JointActuation::VIRTUAL)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << c.fluid.liquid_level_joint_id << "' must be VIRTUAL (is "
           << components::jointActuationToString(joint->actuation) << ").";
        throw std::runtime_error(os.str());
      }
      if (joint->type != components::JointType::PRISMATIC || joint->dof() != 1)
      {
        std::ostringstream os;
        os << "[liquid_level] Joint '" << c.fluid.liquid_level_joint_id << "' must be 1-DOF PRISMATIC.";
        throw std::runtime_error(os.str());
      }

      // Frames
      auto* tf_payload = database::get_element(tcomp.elements, c.payload.frame_id);
      if (!tf_payload)
      {
        std::ostringstream os;
        os << "[liquid_level] Payload frame '" << c.payload.frame_id << "' not found for container '" << name << "'.";
        throw std::runtime_error(os.str());
      }

      Eigen::Vector3d p_base_world = tf_payload->global.translation();

      // FillEnv
      physics::FillEnv env;
      env.T_world_domain = tf_payload->global;
      env.p_base_world = p_base_world;
      env.g_down_world = gravity_world.normalized();

      const double tol = 1e-15;
      double h = 0.0;

      const bool tilted = !dom->canUseAnalyticNow(env);
      if (tilted && !dom->hasMesh())
      {
        if (dom->stacked_shape_id.empty())
        {
          std::ostringstream os;
          os << "[liquid_level] DomainShape '" << c.payload.domain_shape_id
             << "' requires mesh but has no stacked_shape_id.";
          throw std::runtime_error(os.str());
        }
        auto* stack = database::get_element(scomp.elements, dom->stacked_shape_id);
        if (!stack)
        {
          std::ostringstream os;
          os << "[liquid_level] StackedShape '" << dom->stacked_shape_id << "' not found for DomainShape '"
             << c.payload.domain_shape_id << "'.";
          throw std::runtime_error(os.str());
        }
        dom->setMeshCacheFromStack(*stack, /*radial_res=*/64, /*axial_res_spherical=*/16, /*weld_tol=*/1e-9);
      }

      // Compute world-vertical height
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
        // Clamp in world space
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
        // Convert to axis displacement and clamp in axis space
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

        h = h_axis;  // axis coords from here on
      }

      // Joint hard limits (axis units)
      if (joint->limit.min_position.size() == 1 && joint->limit.max_position.size() == 1)
      {
        const double jmin = joint->limit.min_position[0];
        const double jmax = joint->limit.max_position[0];
        if (jmax > jmin)
          h = std::clamp(h, jmin, jmax);
      }

      // Write joint state
      if (joint->position.size() != 1)
        joint->resize(1);
      joint->position[0] = h;

      // Mark frames dirty
      if (!c.fluid.liquid_level_frame_id.empty())
      {
        if (auto* tf_ll = database::get_element(tcomp.elements, c.fluid.liquid_level_frame_id))
          tf_ll->dirty = true;
        else
        {
          std::ostringstream os;
          os << "[liquid_level] liquid_level_frame_id '" << c.fluid.liquid_level_frame_id
             << "' not found for container '" << name << "'.";
          throw std::runtime_error(os.str());
        }
      }
      if (auto* tf_joint = database::get_element(tcomp.elements, c.fluid.liquid_level_joint_id))
        tf_joint->dirty = true;
      else
      {
        std::ostringstream os;
        os << "[liquid_level] transform for joint '" << c.fluid.liquid_level_joint_id << "' not found for container '"
           << name << "'.";
        throw std::runtime_error(os.str());
      }
    }
  });
}

}  // namespace systems
}  // namespace sodf
