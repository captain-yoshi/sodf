#ifndef SODF_ASSEMBLY_CONSTRAINT_SELECTOR_H_
#define SODF_ASSEMBLY_CONSTRAINT_SELECTOR_H_
#include <sodf/assembly/namespace.h>
#include <sodf/assembly/constraint.h>
#include <functional>
#include <vector>

namespace sodf {
namespace assembly {

struct SelectorContext
{
  struct InsertionData
  {
    Pose mouth;
    Eigen::Vector3d axis;
    Eigen::Vector3d ref;
    std::function<bool(double& radius)> getCylinderRadius;
    std::function<bool(double& r0, double& r1, double& H)> getConeDims;
  };

  // Prefer passing Ref so DB lookups can use r.local_path()
  std::function<bool(const Ref&, Pose& out)> getFrame;
  std::function<bool(const Ref&, InsertionData& out)> getInsertion;
  std::function<bool(const Ref&, Axis&, double&, double&)> getCylinder;
  std::function<bool(const Ref&, Axis&, double&, double&, double&)> getCone;
  std::function<bool(const Ref&, Plane&)> getPlane;
  std::function<bool(const Ref&, Axis&)> getStackedPrimaryAxis;
};

// Primitive resolvers
Axis resolveAxis(const Ref& r, const SelectorContext& ctx);
Plane resolvePlane(const Ref& r, const SelectorContext& ctx);
Pose resolvePose(const Ref& r, const SelectorContext& ctx);
Point resolvePoint(const Ref& r, const SelectorContext& ctx);

// ===== Core (Ref-based) constraint functions =====
Eigen::Isometry3d Coincident(const Ref& host, const Ref& guest, const SelectorContext& ctx);
Eigen::Isometry3d Concentric(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx);
Eigen::Isometry3d Parallel(const Ref& host_axis, const Ref& guest_axis, const SelectorContext& ctx);
Eigen::Isometry3d Angle(const Ref& host_axis, const Ref& guest_axis, double radians, const SelectorContext& ctx);
Eigen::Isometry3d Distance(const Ref& host, const Ref& guest, double value, const SelectorContext& ctx);
Eigen::Isometry3d SeatConeOnCylinder(const Ref& host_cyl, const Ref& guest_cone, const SelectorContext& ctx,
                                     double tol = 1e-9, int max_it = 60);

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_CONSTRAINT_SELECTOR_H_
