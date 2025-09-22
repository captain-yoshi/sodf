#ifndef SODF_COMPONENTS_OBJECT_H_
#define SODF_COMPONENTS_OBJECT_H_

#include <string>

namespace sodf {
namespace components {

struct ObjectComponent
{
  std::string id;
  std::string name;
  std::string model;
  std::string serial_number;
  std::string vendor;
};

}  // namespace components
}  // namespace sodf

#endif
