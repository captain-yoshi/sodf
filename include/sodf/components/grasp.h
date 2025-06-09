#ifndef SODF_COMPONENT_GRASP_H_
#define SODF_COMPONENT_GRASP_H_

#include <Eigen/Geometry>

#include <sodf/ecs.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

struct ParallelGrasp
{
  enum class ApproachType : uint8_t
  {
    INTERNAL,  // Approach must be greater then the gap size
    EXTERNAL,  // Approach must be lower then the gap size
  };

  double gap_size;
  Eigen::Vector3d axis_of_rotation;
  ApproachType approach = ApproachType::INTERNAL;

  // References of shape/s contacted by the gripper jaws for this grasp.
  // Typically one 3D shape or a pair of opposing shapes, may include multiple pairs of symmetric regions.
  std::vector<std::string> contact_shape_ids;

  // Synthetic grasp region, used for canonical frame definition and rotational symmetry.
  // For symmetric objects (e.g., cylinders), encodes the infinite family of grasps as a line.
  geometry::Shape canonical_surface;

  // Number of allowed orientations around axis_of_rotation
  // E.g. 1 = unique grasp, 2 = 0 / 180 degrees, 0 = infinite symmetry
  uint32_t rotational_symmetry = 1;
};

struct ParallelGraspComponent
{
  FlatMap<std::string, ParallelGrasp> grasp_map;
};

inline std::ostream& operator<<(std::ostream& os, ParallelGrasp::ApproachType approach)
{
  switch (approach)
  {
    case ParallelGrasp::ApproachType::INTERNAL:
      os << "INTERNAL";
      break;
    case ParallelGrasp::ApproachType::EXTERNAL:
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
     << "  contact_shape_ids=[";
  for (size_t i = 0; i < grasp.contact_shape_ids.size(); ++i)
  {
    os << grasp.contact_shape_ids[i];
    if (i + 1 < grasp.contact_shape_ids.size())
      os << ", ";
  }
  os << "],\n"
     << "  canonical_surface=" << grasp.canonical_surface << ",\n"
     << "  rotational_symmetry=" << grasp.rotational_symmetry << "\n"
     << ")";
  return os;
}

}  // namespace components
}  // namespace sodf

#endif
