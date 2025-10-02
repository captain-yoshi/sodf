#ifndef SODF_COMPONENT_TYPE_H_
#define SODF_COMPONENT_TYPE_H_

#include <string>

namespace sodf {

enum class ComponentType
{
  Unknown = 0,  // For error handling or untagged types
  Product,
  Button,
  VirtualButton,
  Touchscreen,
  Joint,
  Link,
  Insertion,
  Origin,
  FSM,
  ActionMap,
  Transform,
  // Add more types as needed (LED, Slider, etc.)
};

inline const char* componentTypeToString(ComponentType type)
{
  switch (type)
  {
    case ComponentType::Unknown:
      return "Unknown";
    case ComponentType::Product:
      return "Product";
    case ComponentType::Button:
      return "Button";
    case ComponentType::VirtualButton:
      return "VirtualButton";
    case ComponentType::Touchscreen:
      return "Touchscreen";
    case ComponentType::Joint:
      return "Joint";
    case ComponentType::Link:
      return "Link";
    case ComponentType::Insertion:
      return "Insertion";
    case ComponentType::Origin:
      return "Origin";
    case ComponentType::FSM:
      return "FSM";
    case ComponentType::ActionMap:
      return "ActionMap";
    case ComponentType::Transform:
      return "Transform";
    // Add more here as needed
    default:
      return "Unknown";
  }
}

inline ComponentType componentTypeFromString(const std::string& type_str)
{
  // Only accept exact matches that start with uppercase (PascalCase)
  if (type_str == "Product")
    return ComponentType::Product;
  if (type_str == "Button")
    return ComponentType::Button;
  if (type_str == "VirtualButton")
    return ComponentType::VirtualButton;
  if (type_str == "Touchscreen")
    return ComponentType::Touchscreen;
  if (type_str == "Joint")
    return ComponentType::Joint;
  if (type_str == "Link")
    return ComponentType::Link;
  if (type_str == "Insertion")
    return ComponentType::Insertion;
  if (type_str == "Origin")
    return ComponentType::Origin;
  if (type_str == "FSM")
    return ComponentType::FSM;
  if (type_str == "ActionMap")
    return ComponentType::ActionMap;
  if (type_str == "Transform")
    return ComponentType::Transform;
  // Add more here as needed
  return ComponentType::Unknown;
}

}  // namespace sodf

#endif  // SODF_COMPONENT_TYPE_H_
