#ifndef SODF_COMPONENTS_BUTTON_H_
#define SODF_COMPONENTS_BUTTON_H_

#include <variant>
#include <Eigen/Geometry>

#include <sodf/ecs.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

struct Button
{
  enum class Type
  {
    /**
     * Mechanical push button:
     * - Typically uses a PRISMATIC joint (linear travel along press axis).
     * - Example: keyboard key, pushbutton, trigger.
     */
    PUSH,

    /**
     * Rotary knob or switch:
     * - Typically uses a REVOLUTE joint (rotation about axis).
     * - Example: rotary knob, selector switch.
     */
    ROTARY,

    /**
     * Analog trigger (linear travel, analog sensing):
     * - PRISMATIC joint (linear, analog value).
     * - Example: analog trigger (Gamecube L/R), pressure-sensitive trigger.
     */
    ANALOG_PRISMATIC,

    /**
     * Analog thumbstick (rotational tilt, analog sensing):
     * - SPHERICAL joint (2 DOF rotation, constrained).
     * - Example: gamepad left/right stick.
     */
    ANALOG_SPHERICAL,

    /**
     * Analog rotary (rotary with analog sensing):
     * - REVOLUTE joint (rotation, analog value).
     * - Example: analog trigger (PS5 L2/R2), jog wheel, continuous encoder, analog rotary potentiometer.
     */
    ANALOG_REVOLUTE,

  };

  Type type;
  std::string link_id;   // Reference to Link for shape
  std::string joint_id;  // Reference to Joint for motion, limits, dynamics

  double detent_spacing = 0;  // detent_spacing = 0 means smooth, continuous travel

  /**
   * Optional override for stable mechanical states.
   * - If set, defines multiple rest positions for latching/toggle buttons.
   * - Mechanical example:
   *     - GameCube shoulder trigger: [0.0] for analog travel (rest at unpressed)
   *     - 2-state toggle button: [0.0, 1.0] for OFF/ON positions.
   */
  std::vector<double> rest_positions;  // Can override joint

  /**
   * Optional position-dependent stiffness override.
   * - If set, defines a custom stiffness profile for the button motion.
   * - Mechanical examples:
   *     - GameCube shoulder trigger: soft stiffness [0.2] through most of travel,
   *       sharply increasing [2.0, 6.0] near end (progressive spring).
   *     - 2-state pushbutton: uniform high stiffness at each detent.
   */
  std::vector<double> stiffness_profile;
};

struct VirtualButton
{
  std::string shape_id;        // 2D/3D hitbox shape
  Eigen::Vector3d press_axis;  // Direction to activate

  std::string label;      // UI/display tex
  std::string image_uri;  // Path, URI, or URL for button icon/image
};

struct ButtonComponent
{
  FlatMap<std::string, Button> button_map;
};

struct VirtualButtonComponent
{
  FlatMap<std::string, VirtualButton> button_map;
};

inline std::string buttonTypeToString(Button::Type type)
{
  switch (type)
  {
    case Button::Type::PUSH:
      return "Push";
    case Button::Type::ROTARY:
      return "Rotary";
    case Button::Type::ANALOG_PRISMATIC:
      return "AnalogPrismatic";
    case Button::Type::ANALOG_REVOLUTE:
      return "AnalogRevolute";
    case Button::Type::ANALOG_SPHERICAL:
      return "AnalogSpherical";
    default:
      return "UNKNOWN";
  }
}

inline Button::Type buttonTypeFromString(const std::string& str)
{
  if (str == "Push")
    return Button::Type::PUSH;
  if (str == "Rotary")
    return Button::Type::ROTARY;
  if (str == "AnalogPrismatic")
    return Button::Type::ANALOG_PRISMATIC;
  if (str == "AnalogRevolute")
    return Button::Type::ANALOG_REVOLUTE;
  if (str == "AnalogSpherical")
    return Button::Type::ANALOG_SPHERICAL;
  throw std::runtime_error("Unknown Button::Type: '" + str + "'");
}


}  // namespace components
}  // namespace sodf

#endif
