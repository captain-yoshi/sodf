#ifndef SODF_ASSEMBLY_CONSTRAINT_H_
#define SODF_ASSEMBLY_CONSTRAINT_H_

#include <Eigen/Geometry>
#include <optional>
#include <string>

namespace sodf {
namespace assembly {

// ---------- Primitive types ----------
struct Axis
{
  Eigen::Vector3d point;      // a point on the line
  Eigen::Vector3d direction;  // unit
};

struct Plane
{
  Eigen::Vector3d point;   // any point on the plane
  Eigen::Vector3d normal;  // unit, oriented
};

using Pose = Eigen::Isometry3d;
using Point = Eigen::Vector3d;

// ---------- Primitive solvers (pure geometry) ----------

// Coincident
Pose solveCoincidentFrameFrame(const Pose& Fh, const Pose& Fg);
Pose solveCoincidentPlaneFrame(const Plane& Hplane, const Pose& Fg);
Pose solveCoincidentPointFrame(const Point& Hp, const Pose& Fg);
Pose solveCoincidentPointPoint(const Point& Hp, const Point& Gp);

// Concentric (axes coincide)
Pose solveConcentricAxisAxis(const Axis& H, const Axis& G);

// Parallel (rotation only)
Pose solveParallelAxisAxis(const Axis& H, const Axis& G);

// Angle (rotation only) — set angle between axes; rotate around host axis
Pose solveAngleAxisAxis(const Axis& H, const Axis& G, double radians);

// Distance
//  - host plane: move guest along plane normal to specified offset from plane
Pose solveDistancePlaneFrame(const Plane& Hplane, const Pose& G, double offset);
//  - host axis: move guest along host axis by signed distance (component wise)
Pose solveDistanceAxisFrame(const Axis& Haxis, const Pose& G, double distance);
//  - point to point: translate along line connecting them to reach distance (signed by host->guest)
Pose solveDistancePointPoint(const Point& Hp, const Point& Gp, double distance);

// SeatConeOnCylinder (axes aligned + seating depth)
// Inputs: host cylinder radius r, axis H; guest cone radii (r0 at z=0, r1 at z=H), height Hcone, axis G
Pose solveSeatConeOnCylinder(double r_cyl, const Axis& Hcyl, double r0_cone, double r1_cone, double H_cone,
                             const Axis& Gcone, double tol = 1e-9, int max_it = 60);

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_CONSTRAINT_H_
