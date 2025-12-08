#ifndef SODF_COMPONENTS_SHAPE_H_
#define SODF_COMPONENTS_SHAPE_H_

#include <sodf/components/data_type.h>
#include <sodf/components/def_or_ref.h>
#include <sodf/geometry/shape.h>

namespace sodf {
namespace components {

/// Stores reusable Shape definitions OR aliases to them.
///
/// Element key conventions:
///   - Keys may represent either:
///       (a) definition IDs  (e.g. "shape/well")
///       (b) instance IDs    (e.g. "shape/well/A1")
///
/// Value meaning:
///   - DefOrRef<Shape> holds:
///       - geometry::Shape         → inline definition
///       - std::string (id)        → alias to definition ID
///
/// Placement:
///   - For instance keys, placement is found in TransformComponent
///     element with the same key (your global convention).
struct ShapeComponent
{
  using definition_type = geometry::Shape;
  ElementMap<std::string, DefOrRef<definition_type>> elements;
};

/// Stores reusable StackedShape definitions OR aliases to them.
/// Same conventions as ShapeComponent.
struct StackedShapeComponent
{
  using definition_type = geometry::StackedShape;
  ElementMap<std::string, DefOrRef<definition_type>> elements;
};

}  // namespace components
}  // namespace sodf

#endif  // SODF_COMPONENTS_SHAPE_H_
