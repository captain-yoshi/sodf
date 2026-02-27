#ifndef SODF_COMPONENTS_CONTAINER_H_
#define SODF_COMPONENTS_CONTAINER_H_

#include <string>
#include <variant>
#include <vector>

#include <sodf/components/data_type.h>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

// -----------------------------------------------------------------------------
// Domain classification (planner-level semantics)
// -----------------------------------------------------------------------------
enum class DomainType
{
  Fluid,
  // Continuous, gravity-driven free-surface domain.
  // Example: water, buffer, liquid reagent.

  BulkSolid,
  // Continuous, non-flowing material with meaningful surface.
  // Example: agar plate, gel slab, powder layer (static model).

  Discrete
  // Countable rigid items.
  // Example: pipette tips in rack, screws in bin, tubes in rack.
};

// -----------------------------------------------------------------------------
// Shared geometric payload description
// -----------------------------------------------------------------------------
struct PayloadDomain
{
  // Must resolve to something convertible to DomainShape (StackedShape).
  std::string domain_shape_id;

  // Continuous volume (Fluid / BulkSolid)
  // Ignored for Discrete.
  double volume = 0.0;
};

// -----------------------------------------------------------------------------
// Runtime domain specializations
// -----------------------------------------------------------------------------
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

struct BulkSolidDomain
{
  // Static surface frame for interaction
  std::string surface_frame_id;
};

struct DiscreteDomain
{
  std::size_t count = 0;
  std::size_t capacity = 0;

  // Approximate envelope volume per item (optional, for bin filling logic)
  double item_volume = 0.0;

  std::string item_type;

  double occupiedVolume() const
  {
    return count * item_volume;
  }

  bool isFull() const
  {
    return capacity > 0 && count >= capacity;
  }
};

// Variant holding runtime behavior
using DomainRuntime = std::variant<std::monostate, FluidDomain, BulkSolidDomain, DiscreteDomain>;

// -----------------------------------------------------------------------------
// Container
// -----------------------------------------------------------------------------
struct Container
{
  PayloadDomain payload;

  DomainRuntime runtime;

  std::string content_type;
  std::string material_id;

  // --- Convenience helpers ---------------------------------------------------

  bool isFluid() const
  {
    return std::holds_alternative<FluidDomain>(runtime);
  }

  bool isBulkSolid() const
  {
    return std::holds_alternative<BulkSolidDomain>(runtime);
  }

  bool isDiscrete() const
  {
    return std::holds_alternative<DiscreteDomain>(runtime);
  }

  FluidDomain* fluid()
  {
    return std::get_if<FluidDomain>(&runtime);
  }

  BulkSolidDomain* bulk()
  {
    return std::get_if<BulkSolidDomain>(&runtime);
  }

  DiscreteDomain* discrete()
  {
    return std::get_if<DiscreteDomain>(&runtime);
  }
};

// -----------------------------------------------------------------------------
// ECS component
// -----------------------------------------------------------------------------
struct ContainerComponent
{
  ElementMap<std::string, Container> elements;
};

}  // namespace components
}  // namespace sodf

#endif
