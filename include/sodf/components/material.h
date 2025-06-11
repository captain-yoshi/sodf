#ifndef SODF_COMPONENTS_MATERIAL_H_
#define SODF_COMPONENTS_MATERIAL_H_

#include <sodf/ecs.h>

#include <vector>
#include <utility>

#include <Eigen/Geometry>

namespace sodf {
namespace components {

struct Material
{
  std::string id;
  std::string name;
  std::string color_hex;

  double density = 0.0;           // kg/m^3
  double static_friction = 0.0;   // mu_s, for contact start
  double dynamic_friction = 0.0;  // mu_d, for contact sliding
  double restitution = 0.0;       // 0 (inelastic) to 1 (fully elastic)
  double young_modulus = 0.0;     // Pa
  double poisson_ratio = 0.0;     // -1.0 to 0.5 (most materials 0-0.5)
  double damping = 0.0;           // Contact damping

  // Optionally:
  // double conductivity;
  // double specific_heat;
  // double reflectivity;
  // double transmissivity;
};

struct MaterialComponent
{
  ElementMap<std::string, Material> elements;
};

}  // namespace components
}  // namespace sodf

#endif
