#ifndef SODF_COMPONENTS_DEF_OR_REF_H_
#define SODF_COMPONENTS_DEF_OR_REF_H_

#include <string>
#include <variant>

namespace sodf {
namespace components {

/// A lightweight "inline definition OR alias id" container.
///
/// Usage pattern:
///   - Definition-store components can use:
///       elements: ElementMap<std::string, DefOrRef<T>>
///
///   - The map key is the element ID.
///     By your convention, this ID is also the frame ID in TransformComponent.
///
///   - If value holds T:
///       → inline definition
///     If value holds std::string:
///       → alias to a definition ID (local or library)
///
/// NOTE:
///   In this header, alias resolution is LOCAL ONLY:
///   the string is treated as another key in the same component's elements.
///
template <class T>
using DefOrRef = std::variant<T, std::string>;

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_DEF_OR_REF_H_
