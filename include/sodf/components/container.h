#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <vector>
#include <sodf/components/data_type.h>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct PayloadDomain
{
  // Defines the occupied volume of the payload. Must resolve to something
  // you can turn into a mesh / volume curve (e.g. StackedShape).
  std::string domain_shape_id;  // e.g. "fluid/well_200ul"

  // Frame in TransformComponent where this payload volume lives for THIS container.
  // e.g. "container/A1/payload"
  std::string frame_id;

  // How much is present.
  // Fluid:       volume [m^3]
  // BulkSolid:   bulk volume [m^3]
  // RigidInsert: 1.0 = present
  double volume = 0.0;
};

struct FluidDomain
{
  // Virtual prismatic joint that drives the fluid surface height.
  // Must exist in JointComponent.
  // e.g. "container/A1/liquid_level"
  std::string liquid_level_joint_id;

  // Frame attached to the live fluid surface.
  // Must exist in TransformComponent.
  // e.g. "container/A1/liquid_level_frame"
  std::string liquid_level_frame_id;
};

struct Container
{
  // Payload geometry and quantity.
  PayloadDomain payload;

  // Fluid-only runtime info (valid if payload_type == Fluid).
  FluidDomain fluid;

  std::string insertion_id;

  // Semantic / UI / planning info.
  // e.g. "PBS", "lysis_buffer", "M3x8_screw", "p20_tip_rack"
  std::string content_type;

  // Material of the container shell (polypropylene, stainless, etc.).
  std::string material_id;
};

struct ContainerComponent
{
  ElementMap<std::string, Container> elements;
};

}  // namespace components
}  // namespace sodf

#endif
