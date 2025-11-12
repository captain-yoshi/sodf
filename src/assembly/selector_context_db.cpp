#include <sodf/assembly/selector_context_db.h>
#include <sodf/assembly/namespace.h>
#include <sodf/assembly/constraint_selector.h>
#include <sodf/database/database.h>
#include <sodf/database/registry_p.h>

#include <sodf/components/transform.h>
#include <sodf/components/insertion.h>
#include <sodf/components/shape.h>
#include <sodf/components/domain_shape.h>
#include <sodf/components/container.h>

#include <sodf/geometry/shape.h>
#include <sodf/systems/scene_graph.h>

#include <stdexcept>
#include <cmath>

using sodf::database::Database;

namespace sodf {
namespace assembly {

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static inline std::string ref_key(const Ref& r)
{
  return r.ns.empty() ? r.id : (r.ns + "/" + r.id);  // "insert/M24"
}

// Local → world transforms for vectors/points (vectors rotate only)
static inline Eigen::Vector3d rot_vec(const Eigen::Isometry3d& T, const Eigen::Vector3d& v_local)
{
  Eigen::Vector3d v = T.linear() * v_local;
  const double n2 = v.squaredNorm();
  if (n2 == 0.0 || !std::isfinite(n2))
    return Eigen::Vector3d::UnitX();  // safe fallback
  return v / std::sqrt(n2);
}

static inline Eigen::Vector3d xform_point(const Eigen::Isometry3d& T, const Eigen::Vector3d& p_local)
{
  return T.translation() + T.linear() * p_local;
}

// Rotate a canonical (local) axis into the frame (and normalize)
static inline Eigen::Vector3d transformAxis(const Eigen::Isometry3d& T, const Eigen::Vector3d& axis_local)
{
  Eigen::Vector3d v = T.linear() * axis_local;
  const double n2 = v.squaredNorm();
  if (n2 <= 0.0 || !std::isfinite(n2))
    return Eigen::Vector3d(1.0, 0.0, 0.0);  // safe fallback
  return v / std::sqrt(n2);
}

static database::Database::entity_type must_entity(const database::ObjectEntityMap& objects,
                                                   const std::string& object_id)
{
  if (object_id.empty())
  {
    throw std::runtime_error(
        "Unknown object/entity: ''. A local reference was likely used without scope.\n"
        "Ensure OriginComponent.host_object / guest_object are set, or add <Scope host=\"...\" guest=\"...\"/>.");
  }
  auto it = objects.find(object_id);
  if (it == objects.end())
    throw std::runtime_error("Unknown object/entity: '" + object_id + "'");
  return it->second;
}

// -----------------------------------------------------------------------------
// Shape readers (prefer actual ShapeComponent; fallback to DomainShapeComponent)
// -----------------------------------------------------------------------------

static bool read_cylinder_from_shape(Database& db, database::Database::entity_type e, const std::string& key,
                                     Axis& out_axis, double& out_radius, double& out_height)
{
  using components::ShapeComponent;
  if (auto* s = db.get_element<ShapeComponent>(e, key))
  {
    if (s->type == geometry::ShapeType::Cylinder)
    {
      const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
      out_axis.point = T.translation();
      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
      out_radius = geometry::getShapeBaseRadius(*s);  // cyl: base==top
      out_height = geometry::getShapeHeight(*s);
      return true;
    }
  }
  return false;
}

static bool read_cone_from_shape(Database& db, database::Database::entity_type e, const std::string& key,
                                 Axis& out_axis, double& r0, double& r1, double& H)
{
  using components::ShapeComponent;
  if (auto* s = db.get_element<ShapeComponent>(e, key))
  {
    if (s->type == geometry::ShapeType::Cone)
    {
      const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
      out_axis.point = T.translation();
      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
      r0 = geometry::getShapeBaseRadius(*s);
      r1 = geometry::getShapeTopRadius(*s);
      H = geometry::getShapeHeight(*s);
      return true;
    }
  }
  return false;
}

static bool read_plane_from_shape(Database& db, database::Database::entity_type e, const std::string& key,
                                  Plane& out_plane)
{
  using components::ShapeComponent;
  if (auto* s = db.get_element<ShapeComponent>(e, key))
  {
    if (s->type == geometry::ShapeType::Plane ||
        s->type == geometry::ShapeType::Rectangle)  // treat bounded rectangle as plane, too
    {
      const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
      out_plane.point = T.translation();
      // For 2D shapes, Primary is the plane normal by your convention.
      out_plane.normal = transformAxis(T, geometry::getShapePrimaryAxis(*s));
      return true;
    }
  }
  return false;
}

// Resolve cylinder dims from a DomainShape by following its stacked_shape_id.
// Pose (axis point/dir) comes from the DOMAIN frame at `key`.
static bool read_cylinder_from_domain(Database& db, database::Database::entity_type e, const std::string& key,
                                      Axis& out_axis, double& out_radius, double& out_height)
{
  using components::DomainShapeComponent;   // returns physics::DomainShape*
  using components::ShapeComponent;         // geometry::Shape
  using components::StackedShapeComponent;  // geometry::StackedShape

  // 1) Pose from the domain's own frame
  const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
  out_axis.point = T.translation();

  // 2) The domain instance itself
  const auto* dsc = db.get_element<DomainShapeComponent>(e, key);
  if (!dsc)
    return false;

  // 3) Follow provenance to stacked/shape geometry (preferred source of dims)
  const std::string& src_id = dsc->stacked_shape_id;  // may be empty
  if (!src_id.empty())
  {
    // 3a) Try a stacked shape first
    if (const auto* ss = db.get_element<StackedShapeComponent>(e, src_id))
    {
      for (const auto& entry : ss->shapes)
      {
        const geometry::Shape& s = entry.shape;
        if (s.type == geometry::ShapeType::Cylinder)
        {
          out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(s));
          out_radius = geometry::getShapeBaseRadius(s);  // cyl: base==top
          out_height = geometry::getShapeHeight(s);
          return true;
        }
      }
    }
    // 3b) Or a single analytic shape
    if (const auto* s = db.get_element<ShapeComponent>(e, src_id))
    {
      if (s->type == geometry::ShapeType::Cylinder)
      {
        out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
        out_radius = geometry::getShapeBaseRadius(*s);
        out_height = geometry::getShapeHeight(*s);
        return true;
      }
    }
  }

  // 4) Last-chance: some projects co-locate the analytic shape under the same `key`
  if (const auto* s = db.get_element<ShapeComponent>(e, key))
  {
    if (s->type == geometry::ShapeType::Cylinder)
    {
      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
      out_radius = geometry::getShapeBaseRadius(*s);
      out_height = geometry::getShapeHeight(*s);
      return true;
    }
  }

  return false;
}

// Resolve cone dims from a DomainShape by following its stacked_shape_id.
// Pose (axis point/dir) comes from the DOMAIN frame at `key`.
static bool read_cone_from_domain(Database& db, database::Database::entity_type e, const std::string& key,
                                  Axis& out_axis, double& r0, double& r1, double& H)
{
  using components::DomainShapeComponent;
  using components::ShapeComponent;
  using components::StackedShapeComponent;

  // 1) Pose from the domain's own frame
  const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
  out_axis.point = T.translation();

  // 2) The domain instance itself
  const auto* dsc = db.get_element<DomainShapeComponent>(e, key);
  if (!dsc)
    return false;

  // 3) Follow provenance to stacked/shape geometry (preferred source of dims)
  const std::string& src_id = dsc->stacked_shape_id;  // may be empty
  if (!src_id.empty())
  {
    // 3a) Prefer a cone from the stacked shape
    if (const auto* ss = db.get_element<StackedShapeComponent>(e, src_id))
    {
      for (const auto& entry : ss->shapes)
      {
        const geometry::Shape& s = entry.shape;
        if (s.type == geometry::ShapeType::Cone)
        {
          out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(s));
          r0 = geometry::getShapeBaseRadius(s);
          r1 = geometry::getShapeTopRadius(s);
          H = geometry::getShapeHeight(s);
          return true;
        }
      }
    }
    // 3b) Or a single analytic shape
    if (const auto* s = db.get_element<ShapeComponent>(e, src_id))
    {
      if (s->type == geometry::ShapeType::Cone)
      {
        out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
        r0 = geometry::getShapeBaseRadius(*s);
        r1 = geometry::getShapeTopRadius(*s);
        H = geometry::getShapeHeight(*s);
        return true;
      }
    }
  }

  // 4) Last-chance: analytic shape under the same `key`
  if (const auto* s = db.get_element<ShapeComponent>(e, key))
  {
    if (s->type == geometry::ShapeType::Cone)
    {
      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(*s));
      r0 = geometry::getShapeBaseRadius(*s);
      r1 = geometry::getShapeTopRadius(*s);
      H = geometry::getShapeHeight(*s);
      return true;
    }
  }

  return false;
}

static bool read_plane_from_frame(Database& db, database::Database::entity_type e, const std::string& key,
                                  Plane& out_plane)
{
  // Fallback: any frame is a plane with +Z normal (frame convention)
  Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
  out_plane.point = T.translation();
  out_plane.normal = T.linear().col(2).normalized();  // +Z
  return true;
}

// -----------------------------------------------------------------------------
// Insertion (and optional profile) reader
// -----------------------------------------------------------------------------

static SelectorContext::InsertionData read_insertion(Database& db, database::Database::entity_type e,
                                                     const std::string& key)
{
  using components::InsertionComponent;
  using components::ShapeComponent;
  using components::StackedShapeComponent;

  SelectorContext::InsertionData out;

  // 1) Base pose (the "mouth" of the insertion) comes from the insertion frame itself.
  const Eigen::Isometry3d T_ins = sodf::systems::get_global_transform(db, e, key);
  out.mouth = T_ins;

  // Defaults from the frame (already in world)
  out.axis = T_ins.linear().col(2).normalized();  // +Z as insertion axis by frame convention
  out.ref = T_ins.linear().col(0).normalized();   // +X as reference

  // 2) Fetch the Insertion struct (DB returns the element directly).
  const auto* ins = db.get_element<InsertionComponent>(e, key);
  if (!ins)
  {
    // No metadata; we still have a valid mouth/axis/ref from the frame.
    return out;
  }

  // 2a) If author provided explicit LOCAL axes, rotate them into world
  if (ins->axis_insertion.squaredNorm() > 0.0)
  {
    out.axis = rot_vec(T_ins, ins->axis_insertion);
  }
  if (ins->axis_reference.squaredNorm() > 0.0)
  {
    out.ref = rot_vec(T_ins, ins->axis_reference);
  }

  // Keep ref ⟂ axis (defensive)
  out.ref = (out.ref - out.axis * out.axis.dot(out.ref)).normalized();
  if (!std::isfinite(out.ref.squaredNorm()) || out.ref.squaredNorm() < 1e-12)
  {
    // choose any perpendicular
    Eigen::Vector3d tmp = std::abs(out.axis.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    out.ref = (tmp - out.axis * out.axis.dot(tmp)).normalized();
  }

  // 3) Try to derive seat profile & axes from the referenced stacked shape.
  //    NOTE: we read dims from geometry::Shape (intrinsic), not from transforms.
  const std::string& stack_id = ins->stacked_shape_id;           // e.g. "stacked_shape/hole.1_4-20"
  const std::string& stack_frame = ins->stacked_shape_frame_id;  // e.g. "insertion/container/fit1" (optional pose)

  if (!stack_frame.empty())
  {
    // If a specific frame for the stacked shape is provided, we can optionally
    // use its axes if author didn't set custom axes.
    try
    {
      const Eigen::Isometry3d T_stack = sodf::systems::get_global_transform(db, e, stack_frame);

      // If user didn't provide custom local axes, try to fetch from stacked shape geometry:
      if (ins->axis_insertion.squaredNorm() == 0.0 || ins->axis_reference.squaredNorm() == 0.0)
      {
        if (const auto* ss = db.get_element<StackedShapeComponent>(e, stack_id))
        {
          // take the first shape as reference for canonical axes
          for (const auto& entry : ss->shapes)
          {
            const geometry::Shape& s = entry.shape;

            if (ins->axis_insertion.squaredNorm() == 0.0)
              out.axis = transformAxis(T_stack, geometry::getShapePrimaryAxis(s));

            if (ins->axis_reference.squaredNorm() == 0.0)
              out.ref = transformAxis(T_stack, geometry::getShapeUAxis(s));

            break;
          }
        }
        else if (const auto* sc = db.get_element<ShapeComponent>(e, stack_id))
        {
          if (ins->axis_insertion.squaredNorm() == 0.0)
            out.axis = transformAxis(T_stack, geometry::getShapePrimaryAxis(*sc));
          if (ins->axis_reference.squaredNorm() == 0.0)
            out.ref = transformAxis(T_stack, geometry::getShapeUAxis(*sc));
        }
      }
      // If no geometry available, fall back to frame axes (already set earlier)
    }
    catch (...)
    { /* ignore if missing */
    }
  }

  // Wire optional radius/cone providers (if geometry present)
  auto wire_cylinder = [&](double R) {
    out.getCylinderRadius = [R](double& r) {
      r = R;
      return true;
    };
  };
  auto wire_cone = [&](double r0, double r1, double H) {
    out.getConeDims = [r0, r1, H](double& o0, double& o1, double& oH) {
      o0 = r0;
      o1 = r1;
      oH = H;
      return true;
    };
  };

  if (!stack_id.empty())
  {
    if (const auto* stack = db.get_element<StackedShapeComponent>(e, stack_id))
    {
      bool have_cyl = false, have_cone = false;
      double cyl_R = 0.0;
      double cone_r0 = 0.0, cone_r1 = 0.0, cone_H = 0.0;

      for (const auto& entry : stack->shapes)
      {
        const geometry::Shape& s = entry.shape;
        switch (s.type)
        {
          case geometry::ShapeType::Cone:
            cone_r0 = geometry::getShapeBaseRadius(s);
            cone_r1 = geometry::getShapeTopRadius(s);
            cone_H = geometry::getShapeHeight(s);
            have_cone = true;
            break;
          case geometry::ShapeType::Cylinder:
            cyl_R = geometry::getShapeBaseRadius(s);  // base==top
            have_cyl = true;
            break;
          default:
            break;
        }
        if (have_cone)
          break;  // prefer cone if both present
      }

      if (have_cone)
        wire_cone(cone_r0, cone_r1, cone_H);
      else if (have_cyl)
        wire_cylinder(cyl_R);
    }
    else
    {
      if (const auto* s = db.get_element<ShapeComponent>(e, stack_id))
      {
        if (s->type == geometry::ShapeType::Cone)
          wire_cone(geometry::getShapeBaseRadius(*s), geometry::getShapeTopRadius(*s), geometry::getShapeHeight(*s));
        else if (s->type == geometry::ShapeType::Cylinder)
          wire_cylinder(geometry::getShapeBaseRadius(*s));
      }
    }
  }

  return out;
}

// -----------------------------------------------------------------------------
// Public factory
// -----------------------------------------------------------------------------

SelectorContext makeSelectorContext(database::Database& db, const database::ObjectEntityMap& objects)
{
  SelectorContext ctx;

  // FRAMES  ── bool (Ref, Pose& out)
  ctx.getFrame = [&db, &objects](const Ref& r, Pose& out) -> bool {
    try
    {
      auto e = must_entity(objects, r.entity);
      const Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, ref_key(r));
      out = Pose{ T };
      return true;
    }
    catch (...)
    {
      return false;
    }
  };

  // INSERTION  ── bool (Ref, InsertionData& out)
  ctx.getInsertion = [&db, &objects](const Ref& r, SelectorContext::InsertionData& out) -> bool {
    try
    {
      auto e = must_entity(objects, r.entity);
      out = read_insertion(db, e, ref_key(r));
      return true;
    }
    catch (...)
    {
      return false;
    }
  };

  // SHAPE: cylinder
  ctx.getCylinder = [&db, &objects](const Ref& r, Axis& out_axis, double& radius, double& height) -> bool {
    auto e = must_entity(objects, r.entity);
    const std::string key = ref_key(r);
    if (read_cylinder_from_shape(db, e, key, out_axis, radius, height))
      return true;
    if (read_cylinder_from_domain(db, e, key, out_axis, radius, height))
      return true;
    return false;
  };

  // SHAPE: cone
  ctx.getCone = [&db, &objects](const Ref& r, Axis& out_axis, double& r0, double& r1, double& height) -> bool {
    auto e = must_entity(objects, r.entity);
    const std::string key = ref_key(r);
    if (read_cone_from_shape(db, e, key, out_axis, r0, r1, height))
      return true;
    if (read_cone_from_domain(db, e, key, out_axis, r0, r1, height))
      return true;
    return false;
  };

  // SHAPE: plane
  ctx.getPlane = [&db, &objects, &ctx](const Ref& r, Plane& out_plane) -> bool {
    auto e = must_entity(objects, r.entity);
    const std::string key = ref_key(r);
    if (read_plane_from_shape(db, e, key, out_plane))
      return true;

    // Fallback to frame plane (+Z)
    Pose F;
    if (!ctx.getFrame(r, F))
      return false;
    out_plane.point = F.translation();
    out_plane.normal = F.linear().col(2).normalized();
    return true;
  };

  // STACKED_SHAPE: primary axis
  ctx.getStackedPrimaryAxis = [&db, &objects](const Ref& r, Axis& out_axis) -> bool {
    using components::StackedShapeComponent;
    auto e = must_entity(objects, r.entity);
    const std::string key = ref_key(r);

    if (auto* ss = db.get_element<StackedShapeComponent>(e, key))
    {
      Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
      // Use frame pose; primary axis of the stacked shape is intrinsic, but we do
      // not crack the list here—this function historically returns the frame's
      // primary (we keep behavior stable). If you want intrinsic, resolve a shape.
      out_axis.point = T.translation();
      out_axis.direction = T.linear().col(2).normalized();  // keep legacy behavior for stacked frame (+Z)
      return true;
    }

    // Fallback: any frame's +Z
    try
    {
      Eigen::Isometry3d T = sodf::systems::get_global_transform(db, e, key);
      out_axis.point = T.translation();
      out_axis.direction = T.linear().col(2).normalized();
      return true;
    }
    catch (...)
    {
      return false;
    }
  };

  return ctx;
}

namespace {

// map world→host for pose/axis/plane
static inline sodf::assembly::Pose to_host_pose(const Eigen::Isometry3d& W_H, const sodf::assembly::Pose& Pw)
{
  sodf::assembly::Pose out = sodf::assembly::Pose::Identity();
  out.linear() = W_H.linear() * Pw.linear();
  out.translation() = W_H * Pw.translation();
  return out;
}

static inline sodf::assembly::Axis to_host_axis(const Eigen::Isometry3d& W_H, const sodf::assembly::Axis& aW)
{
  sodf::assembly::Axis a;
  a.point = W_H * aW.point;
  a.direction = (W_H.linear() * aW.direction).normalized();
  return a;
}

static inline sodf::assembly::Plane to_host_plane(const Eigen::Isometry3d& W_H, const sodf::assembly::Plane& pW)
{
  sodf::assembly::Plane p;
  p.point = W_H * pW.point;
  p.normal = (W_H.linear() * pW.normal).normalized();
  return p;
}

}  // namespace

// Public helper: return a SelectorContext whose outputs are expressed in HOST frame
SelectorContext makeSelectorContextInHostSpace(sodf::database::Database& db,
                                               const sodf::database::ObjectEntityMap& objects,
                                               const std::string& host_object_id)
{
  using sodf::systems::get_global_transform;

  // base = world-space context
  sodf::assembly::SelectorContext base = sodf::assembly::makeSelectorContext(db, objects);

  // resolve host world pose (root)
  auto it = objects.find(host_object_id);
  if (it == objects.end())
    throw std::runtime_error("makeSelectorContextInHostSpace: unknown host object '" + host_object_id + "'");

  sodf::systems::update_all_global_transforms(db);
  const Eigen::Isometry3d H_W = get_global_transform(db, it->second, "root");
  const Eigen::Isometry3d W_H = H_W.inverse();

  sodf::assembly::SelectorContext ctx = base;  // copy & wrap

  // Wrap getFrame
  {
    auto base_getFrame = base.getFrame;  // copy the original functor

    ctx.getFrame = [base_getFrame, &db, &objects, W_H]  // <-- explicit captures only
        (const sodf::assembly::Ref& r, sodf::assembly::Pose& out) -> bool {
      sodf::assembly::Pose Pw;
      if (!base_getFrame(r, Pw))
        return false;               // world pose from base
      out = to_host_pose(W_H, Pw);  // convert world -> host
      return true;
    };
  }

  // Wrap getInsertion
  {
    auto base_getInsertion = base.getInsertion;
    ctx.getInsertion = [base_getInsertion, W_H](const sodf::assembly::Ref& r,
                                                sodf::assembly::SelectorContext::InsertionData& out) -> bool {
      sodf::assembly::SelectorContext::InsertionData tmp;
      if (!base_getInsertion(r, tmp))
        return false;
      // pose
      tmp.mouth = to_host_pose(W_H, tmp.mouth);
      // axes
      tmp.axis = (W_H.linear() * tmp.axis).normalized();
      tmp.ref = (W_H.linear() * tmp.ref).normalized();
      out = std::move(tmp);
      return true;
    };
  }

  // Wrap getCylinder
  {
    auto base_getCylinder = base.getCylinder;
    ctx.getCylinder = [base_getCylinder, W_H](const sodf::assembly::Ref& r, sodf::assembly::Axis& ax, double& R,
                                              double& H) -> bool {
      sodf::assembly::Axis axW;
      double RW = 0, HW = 0;
      if (!base_getCylinder(r, axW, RW, HW))
        return false;
      ax = to_host_axis(W_H, axW);
      R = RW;
      H = HW;
      return true;
    };
  }

  // Wrap getCone
  {
    auto base_getCone = base.getCone;
    ctx.getCone = [base_getCone, W_H](const sodf::assembly::Ref& r, sodf::assembly::Axis& ax, double& r0, double& r1,
                                      double& H) -> bool {
      sodf::assembly::Axis axW;
      double r0W = 0, r1W = 0, HW = 0;
      if (!base_getCone(r, axW, r0W, r1W, HW))
        return false;
      ax = to_host_axis(W_H, axW);
      r0 = r0W;
      r1 = r1W;
      H = HW;
      return true;
    };
  }

  // Wrap getPlane
  {
    auto base_getPlane = base.getPlane;
    ctx.getPlane = [base_getPlane, W_H](const sodf::assembly::Ref& r, sodf::assembly::Plane& pl) -> bool {
      sodf::assembly::Plane pw;
      if (!base_getPlane(r, pw))
        return false;
      pl = to_host_plane(W_H, pw);
      return true;
    };
  }

  // Wrap getStackedPrimaryAxis
  {
    auto base_getStackedPrimaryAxis = base.getStackedPrimaryAxis;
    ctx.getStackedPrimaryAxis = [base_getStackedPrimaryAxis, W_H](const sodf::assembly::Ref& r,
                                                                  sodf::assembly::Axis& ax) -> bool {
      sodf::assembly::Axis axW;
      if (!base_getStackedPrimaryAxis(r, axW))
        return false;
      ax = to_host_axis(W_H, axW);
      return true;
    };
  }

  return ctx;
}

}  // namespace assembly
}  // namespace sodf
