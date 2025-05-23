#ifndef SODF_COMPONENTS_BUTTON_H_
#define SODF_COMPONENTS_BUTTON_H_

#include <variant>
#include <Eigen/Geometry>

#include <sodf/ecs.h>

namespace sodf {
namespace components {

enum class ButtonType
{
  PUSH,
  ROTARY,
  ANALOG,
  VIRTUAL
};

struct PushButton
{
  double activation_force;         // [N]
  Eigen::Vector3d direction_axis;  // direction axis (unit vector, e.g. -X)

  double min_hold_duration = 0.0;  // Minimum time the button must be held for activation [s]
};

struct RotaryButton
{
  double diameter = 0.0;                                     // size across the circular face [m]
  double depth = 0.0;                                        // depth along axis [m]
  Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();  // Rotation axis [unit vector]

  double min_angle;  // [rad]
  double max_angle;
  double detent_spacing;  // Click resolution
};

struct AnalogButton
{
  Eigen::Vector3d direction_axis;  // direction axis (unit vector, e.g. -X)

  double min_pressure;  // Minimum pressure (analog start)
  double max_pressure;  // Max pressure
};

struct VirtualButton
{
  double activation_force = 0.0;   // [N]
  Eigen::Vector3d direction_axis;  // direction axis (unit vector, e.g. -X)

  double min_hold_duration = 0.0;  // Minimum time the button must be held for activation [s]
};

using ButtonVariant = std::variant<PushButton, RotaryButton, AnalogButton, VirtualButton>;

struct ButtonEntry
{
  ButtonType type;
  ButtonVariant data;
};

struct ButtonComponent
{
  FlatMap<std::string, ButtonEntry> button_map;
};

}  // namespace components
}  // namespace sodf

#endif
