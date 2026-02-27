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
#include <string>
#include <string_view>
#include <algorithm>

using sodf::database::Database;

namespace sodf {
namespace assembly {

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static inline std::string ref_key(const Ref& r)
{
  return r.ns.empty() ? r.id : (r.ns + "/" + r.id);
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

// Rotate a canonical (local) axis into the frame (and normalize)
static inline Eigen::Vector3d transformAxis(const Eigen::Isometry3d& T, const Eigen::Vector3d& axis_local)
{
  Eigen::Vector3d v = T.linear() * axis_local;
  const double n2 = v.squaredNorm();
  if (n2 <= 0.0 || !std::isfinite(n2))
    return Eigen::Vector3d(1.0, 0.0, 0.0);  // safe fallback
  return v / std::sqrt(n2);
}

static database::Database::entity_type must_entity(database::Database& db, const std::string& object_id)
{
  if (object_id.empty())
  {
    throw std::runtime_error(
        "Unknown object/entity: ''. A local reference was likely used without scope.\n"
        "Ensure OriginComponent.host_object / guest_object are set, or add <Scope host=\"...\" guest=\"...\"/>.");
  }

  auto maybe = db.find_object(object_id);
  if (!maybe)
    throw std::runtime_error("Unknown object/entity: '" + object_id + "'");

  return *maybe;
}

// STRICT transform accessor with contextual error
static inline Eigen::Isometry3d must_global_transform(Database& db, database::Database::entity_type e,
                                                      const std::string& key, const std::string& what)
{
  try
  {
    return sodf::systems::get_global_transform(db, e, key);
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error("[assembly] Missing transform for " + what + " frame '" + key + "': " + ex.what());
  }
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
      const Eigen::Isometry3d T = must_global_transform(db, e, key, "cylinder(shape)");
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
      const Eigen::Isometry3d T = must_global_transform(db, e, key, "cone(shape)");
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
      const Eigen::Isometry3d T = must_global_transform(db, e, key, "plane(shape)");
      out_plane.point = T.translation();
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
  using components::DomainShapeComponent;
  using components::ShapeComponent;
  using components::StackedShapeComponent;

  const Eigen::Isometry3d T = must_global_transform(db, e, key, "domain(cylinder)");
  out_axis.point = T.translation();

  const auto* dsc = db.get_element<DomainShapeComponent>(e, key);
  if (!dsc)
    return false;

  const std::string& src_id = dsc->stacked_shape_id;
  if (!src_id.empty())
  {
    if (const auto* ss = db.get_element<StackedShapeComponent>(e, src_id))
    {
      for (const auto& entry : ss->shapes)
      {
        const geometry::Shape& s = entry.shape;
        if (s.type == geometry::ShapeType::Cylinder)
        {
          out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(s));
          out_radius = geometry::getShapeBaseRadius(s);
          out_height = geometry::getShapeHeight(s);
          return true;
        }
      }
    }

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

// -----------------------------------------------------------------------------
// StackedShape dimension helpers
// -----------------------------------------------------------------------------

static bool stacked_cone_profile(const geometry::StackedShape& stack, double& out_r0, double& out_r1, double& out_H)
{
  using geometry::getShapeBaseRadius;
  using geometry::getShapeHeight;
  using geometry::getShapeTopRadius;
  using geometry::Shape;
  using geometry::ShapeType;

  bool have_any = false;
  double r_top = 0.0;
  double r_bottom = 0.0;
  double H_total = 0.0;

  for (const auto& entry : stack.shapes)
  {
    const Shape& s = entry.shape;

    H_total += getShapeHeight(s);
    switch (s.type)
    {
      case ShapeType::Cylinder:
      {
        const double r = getShapeBaseRadius(s);
        if (!have_any)
        {
          r_top = r;
          r_bottom = r;
        }
        else
        {
          r_bottom = r;
        }
        have_any = true;
        break;
      }

      case ShapeType::Cone:
      {
        const double rr0 = getShapeBaseRadius(s);
        const double rr1 = getShapeTopRadius(s);
        if (!have_any)
        {
          r_top = rr0;
          r_bottom = rr1;
        }
        else
        {
          r_bottom = rr1;
        }
        have_any = true;
        break;
      }

      default:
        break;
    }
  }

  if (!have_any)
    return false;

  out_r0 = r_top;
  out_r1 = r_bottom;
  out_H = H_total;
  return true;
}

// Heuristic: interpret a stacked shape as "cylinder-like".
static bool read_cylinder_from_stacked(Database& db, database::Database::entity_type e, const std::string& key,
                                       Axis& out_axis, double& out_radius, double& out_height)
{
  using components::StackedShapeComponent;
  using geometry::ShapeType;

  const auto* ss = db.get_element<StackedShapeComponent>(e, key);
  if (!ss || ss->shapes.empty())
    return false;

  const Eigen::Isometry3d T = must_global_transform(db, e, key, "stacked_shape(cylinder)");

  // Look for cylinder segments; choose max radius; height = sum of cylinder heights if any,
  // otherwise sum of all segments as a fallback.
  bool have_cyl = false;
  double Rmax = 0.0;
  double Hcyl = 0.0;
  double Hall = 0.0;

  for (const auto& entry : ss->shapes)
  {
    const geometry::Shape& s = entry.shape;
    const double h = geometry::getShapeHeight(s);
    Hall += h;

    if (s.type == ShapeType::Cylinder)
    {
      have_cyl = true;
      Hcyl += h;
      Rmax = std::max(Rmax, geometry::getShapeBaseRadius(s));
    }
  }

  if (!have_cyl)
    return false;

  // Axis direction based on first cylinder segment we find
  for (const auto& entry : ss->shapes)
  {
    const geometry::Shape& s = entry.shape;
    if (s.type == ShapeType::Cylinder)
    {
      out_axis.point = T.translation();
      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(s));
      break;
    }
  }

  out_radius = Rmax;
  out_height = (Hcyl > 0.0) ? Hcyl : Hall;
  if (out_height <= 0.0)
    out_height = 1e-6;

  return true;
}

// Heuristic: interpret a stacked shape as "cone-like" using the stacked profile
// across Cylinder/Cone segments.
static bool read_cone_from_stacked(Database& db, database::Database::entity_type e, const std::string& key,
                                   Axis& out_axis, double& r0, double& r1, double& H)
{
  using components::StackedShapeComponent;

  const auto* ss = db.get_element<StackedShapeComponent>(e, key);
  if (!ss || ss->shapes.empty())
    return false;

  double rr0 = 0.0, rr1 = 0.0, HH = 0.0;
  if (!stacked_cone_profile(*ss, rr0, rr1, HH))
    return false;

  const Eigen::Isometry3d T = must_global_transform(db, e, key, "stacked_shape(cone)");
  out_axis.point = T.translation();

  // Use primary axis of the first segment as canonical direction
  out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(ss->shapes.front().shape));

  r0 = rr0;
  r1 = rr1;
  H = (HH > 0.0) ? HH : 1e-6;

  return true;
}

// Resolve cone dims from a DomainShape by following its stacked_shape_id.
// Pose (axis point/dir) comes from the DOMAIN frame at `key`.
static bool read_cone_from_domain(Database& db, database::Database::entity_type e, const std::string& key,
                                  Axis& out_axis, double& r0, double& r1, double& H)
{
  using components::DomainShapeComponent;
  using components::ShapeComponent;
  using components::StackedShapeComponent;

  const Eigen::Isometry3d T = must_global_transform(db, e, key, "domain(cone)");
  out_axis.point = T.translation();

  const auto* dsc = db.get_element<DomainShapeComponent>(e, key);
  if (!dsc)
    return false;

  const std::string& src_id = dsc->stacked_shape_id;
  if (!src_id.empty())
  {
    if (const auto* ss = db.get_element<StackedShapeComponent>(e, src_id))
    {
      double rr0 = 0.0, rr1 = 0.0, HH = 0.0;
      if (!stacked_cone_profile(*ss, rr0, rr1, HH))
        return false;

      out_axis.direction = transformAxis(T, geometry::getShapePrimaryAxis(ss->shapes.front().shape));
      r0 = rr0;
      r1 = rr1;
      H = HH;
      return true;
    }

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
  const Eigen::Isometry3d T = must_global_transform(db, e, key, "frame(plane)");
  out_plane.point = T.translation();
  out_plane.normal = T.linear().col(2).normalized();
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

  // 1) Base pose comes from the insertion frame itself (STRICT)
  const Eigen::Isometry3d T_ins = must_global_transform(db, e, key, "insertion");
  out.mouth = T_ins;

  // Defaults from the frame
  out.axis = T_ins.linear().col(2).normalized();
  out.ref = T_ins.linear().col(0).normalized();

  // 2) Fetch the Insertion metadata
  const auto* ins = db.get_element<InsertionComponent>(e, key);

  if (!ins)
  {
    // Legacy fallback for metadata only
    constexpr std::string_view prefix = "insertion/";
    if (key.rfind(prefix, 0) == 0 && key.size() > prefix.size())
    {
      const std::string short_key = key.substr(prefix.size());
      ins = db.get_element<InsertionComponent>(e, short_key);
    }
  }

  if (!ins)
    return out;

  if (ins)
  {
    out.max_depth = ins->max_depth;
    out.role = ins->role;
  }

  // 2a) Author provided explicit LOCAL axes
  if (ins->axis_insertion.squaredNorm() > 0.0)
    out.axis = rot_vec(T_ins, ins->axis_insertion);

  if (ins->axis_reference.squaredNorm() > 0.0)
    out.ref = rot_vec(T_ins, ins->axis_reference);

  // Keep ref ⟂ axis (defensive)
  out.ref = (out.ref - out.axis * out.axis.dot(out.ref)).normalized();
  if (!std::isfinite(out.ref.squaredNorm()) || out.ref.squaredNorm() < 1e-12)
  {
    Eigen::Vector3d tmp = std::abs(out.axis.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    out.ref = (tmp - out.axis * out.axis.dot(tmp)).normalized();
  }

  // 3) Derive seat profile & axes from referenced stacked shape
  const std::string& stack_id = ins->stacked_shape_id;

  if (!stack_id.empty())
  {
    // STRICT if metadata references a stacked shape frame
    const Eigen::Isometry3d T_stack = must_global_transform(db, e, stack_id, "stacked_shape(insertion)");

    if (ins->axis_insertion.squaredNorm() == 0.0 || ins->axis_reference.squaredNorm() == 0.0)
    {
      if (const auto* ss = db.get_element<StackedShapeComponent>(e, stack_id))
      {
        if (!ss->shapes.empty())
        {
          const geometry::Shape& s = ss->shapes.front().shape;

          if (ins->axis_insertion.squaredNorm() == 0.0)
            out.axis = transformAxis(T_stack, geometry::getShapePrimaryAxis(s));

          if (ins->axis_reference.squaredNorm() == 0.0)
            out.ref = transformAxis(T_stack, geometry::getShapeUAxis(s));
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

    // Wire optional profile providers
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

    if (const auto* stack = db.get_element<StackedShapeComponent>(e, stack_id))
    {
      bool have_cyl = false;
      double cyl_R = 0.0;

      for (const auto& entry : stack->shapes)
      {
        const geometry::Shape& s = entry.shape;
        if (s.type == geometry::ShapeType::Cylinder)
        {
          const double R = geometry::getShapeBaseRadius(s);
          if (!have_cyl || R > cyl_R)
          {
            have_cyl = true;
            cyl_R = R;
          }
        }
      }

      if (have_cyl)
        wire_cylinder(cyl_R);

      double r0 = 0.0, r1 = 0.0, HH = 0.0;
      if (stacked_cone_profile(*stack, r0, r1, HH))
        wire_cone(r0, r1, HH);
    }
    else if (const auto* s = db.get_element<ShapeComponent>(e, stack_id))
    {
      if (s->type == geometry::ShapeType::Cylinder)
      {
        wire_cylinder(geometry::getShapeBaseRadius(*s));
      }
      else if (s->type == geometry::ShapeType::Cone)
      {
        wire_cone(geometry::getShapeBaseRadius(*s), geometry::getShapeTopRadius(*s), geometry::getShapeHeight(*s));
      }
    }
  }

  return out;
}

// -----------------------------------------------------------------------------
// Public factory
// -----------------------------------------------------------------------------

SelectorContext makeSelectorContext(database::Database& db)
{
  SelectorContext ctx;

  // FRAMES  ── bool (Ref, Pose& out)
  // STRICT: throws if transform not found.
  ctx.getFrame = [&db](const Ref& r, Pose& out) -> bool {
    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);
    const Eigen::Isometry3d T = must_global_transform(db, e, key, "frame");
    out = Pose{ T };
    return true;
  };

  // INSERTION  ── bool (Ref, InsertionData& out)
  ctx.getInsertion = [&db](const Ref& r, SelectorContext::InsertionData& out) -> bool {
    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);
    out = read_insertion(db, e, key);
    return true;
  };

  // SHAPE: cylinder
  ctx.getCylinder = [&db](const Ref& r, Axis& out_axis, double& radius, double& height) -> bool {
    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);

    if (read_cylinder_from_shape(db, e, key, out_axis, radius, height))
      return true;

    if (read_cylinder_from_stacked(db, e, key, out_axis, radius, height))
      return true;

    if (read_cylinder_from_domain(db, e, key, out_axis, radius, height))
      return true;

    return false;
  };

  // SHAPE: cone
  ctx.getCone = [&db](const Ref& r, Axis& out_axis, double& r0, double& r1, double& height) -> bool {
    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);

    if (read_cone_from_shape(db, e, key, out_axis, r0, r1, height))
      return true;

    if (read_cone_from_stacked(db, e, key, out_axis, r0, r1, height))
      return true;

    if (read_cone_from_domain(db, e, key, out_axis, r0, r1, height))
      return true;

    return false;
  };

  // SHAPE: plane
  ctx.getPlane = [&db, &ctx](const Ref& r, Plane& out_plane) -> bool {
    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);

    if (read_plane_from_shape(db, e, key, out_plane))
      return true;

    Pose F;
    (void)ctx.getFrame(r, F);
    out_plane.point = F.translation();
    out_plane.normal = F.linear().col(2).normalized();
    return true;
  };

  // STACKED_SHAPE: primary axis
  ctx.getStackedPrimaryAxis = [&db](const Ref& r, Axis& out_axis) -> bool {
    using components::StackedShapeComponent;

    auto e = must_entity(db, r.entity);
    const std::string key = ref_key(r);

    if (auto* ss = db.get_element<StackedShapeComponent>(e, key))
    {
      (void)ss;
      const Eigen::Isometry3d T = must_global_transform(db, e, key, "stacked_shape");
      out_axis.point = T.translation();
      out_axis.direction = T.linear().col(0).normalized();
      return true;
    }

    const Eigen::Isometry3d T = must_global_transform(db, e, key, "stacked_primary_axis(frame)");
    out_axis.point = T.translation();
    out_axis.direction = T.linear().col(0).normalized();
    return true;
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
SelectorContext makeSelectorContextInHostSpace(sodf::database::Database& db, const std::string& host_object_id)
{
  using sodf::systems::get_global_transform;

  // base = world-space context
  sodf::assembly::SelectorContext base = sodf::assembly::makeSelectorContext(db);

  // resolve host world pose (root)
  auto maybe_host = db.find_object(host_object_id);
  if (!maybe_host)
    throw std::runtime_error("makeSelectorContextInHostSpace: unknown host object '" + host_object_id + "'");

  sodf::systems::update_all_global_transforms(db);

  const Eigen::Isometry3d H_W = get_global_transform(db, *maybe_host, "root");

  const Eigen::Isometry3d W_H = H_W.inverse();

  sodf::assembly::SelectorContext ctx = base;  // copy & wrap

  // Wrap getFrame
  {
    auto base_getFrame = base.getFrame;

    ctx.getFrame = [base_getFrame, W_H](const sodf::assembly::Ref& r, sodf::assembly::Pose& out) -> bool {
      sodf::assembly::Pose Pw;
      if (!base_getFrame(r, Pw))
        return false;
      out = to_host_pose(W_H, Pw);
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
      tmp.mouth = to_host_pose(W_H, tmp.mouth);
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
