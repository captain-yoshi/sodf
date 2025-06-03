#ifndef SODF_COMPONENT_GRASP_H_
#define SODF_COMPONENT_GRASP_H_

#include <Eigen/Geometry>

#include <sodf/ecs.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

enum class ParallelGraspApproach : uint8_t
{
  INTERNAL,  // Approach must be greater then the gap size
  EXTERNAL,  // Approach must be lower then the gap size
};

struct ParallelGrasp
{
  double gap_size;
  Eigen::Vector3d axis_of_rotation;
  ParallelGraspApproach approach = ParallelGraspApproach::INTERNAL;

  // References to the true contact surfaces for this grasp (e.g., actual mesh or shape components).
  // Used for contact, collision, or quality evaluation.
  // For 3D shapes (e.g., cylinders), the reference is the lateral (side) surface. Bases (top/bottom faces)
  // should be represented as separate surfaces if relevant.
  std::array<std::string, 2> real_surfaces;

  // Synthetic grasp region, used for canonical frame definition and rotational symmetry.
  // For symmetric objects (e.g., cylinders), encodes the infinite family of grasps as a line.
  geometry::Shape virtual_surface;

  // Number of allowed orientations around axis_of_rotation
  // E.g. 1 = unique grasp, 2 = 0 / 180 degrees, 0 = infinite symmetry
  uint32_t rotational_symmetry = 1;
};

struct ParallelGraspComponent
{
  FlatMap<std::string, ParallelGrasp> grasp_map;
};

inline std::ostream& operator<<(std::ostream& os, ParallelGraspApproach approach)
{
  switch (approach)
  {
    case ParallelGraspApproach::INTERNAL:
      os << "INTERNAL";
      break;
    case ParallelGraspApproach::EXTERNAL:
      os << "EXTERNAL";
      break;
    default:
      os << "UNKNOWN";
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const ParallelGrasp& grasp)
{
  os << "ParallelGrasp(\n"
     << "  gap_size=" << grasp.gap_size << ",\n"
     << "  axis_of_rotation=" << grasp.axis_of_rotation.transpose() << ",\n"
     << "  approach=" << grasp.approach << ",\n"
     << "  real_surfaces=[" << grasp.real_surfaces[0] << ", " << grasp.real_surfaces[1] << "],\n"
     << "  virtual_surface=" << grasp.virtual_surface << ",\n"
     << "  rotational_symmetry=" << grasp.rotational_symmetry << "\n"
     << ")";
  return os;
}

}  // namespace components
}  // namespace sodf

#endif
