#ifndef SODF_COMPONENTS_TOUCHSCREEN_H_
#define SODF_COMPONENTS_TOUCHSCREEN_H_

#include <Eigen/Geometry>
#include <sodf/components/data_type.h>

namespace sodf {
namespace components {

struct Touchscreen
{
  double width = 0.05;                           // screen width [m]
  double height = 0.05;                          //  screen height [m]
  Eigen::Vector3d surface_normal = { 0, 0, 1 };  // Surface normal towards screen (e.g., Z+)
  double min_pressure = 0.0;                     // Minimum pressure to register a valid touch
  double max_pressure = 5.0;                     // Optional: for analog pressure support

  double touch_radius = 0.01;  // Expected touch radius (e.g. finger or stylus tip)
  bool multi_touch = false;    // Whether this zone supports multitouch
  bool allow_drag = true;      // Whether this zone supports sliding gestures

  std::string model;    // e.g., "TS-5R-100"
  std::string version;  // e.g., "v2.1.4"
  std::string driver;   // e.g., "fts_i2c"
};

struct TouchscreenComponent
{
  ElementMap<std::string, Touchscreen> elements;
};

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_TOUCHSCREEN_H_
