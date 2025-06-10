#ifndef SODF_COMPONENTS_ACTION_MAP_H_
#define SODF_COMPONENTS_ACTION_MAP_H_

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

#include <sodf/ecs.h>
#include <sodf/component_type.h>

namespace sodf {
namespace components {

struct ActionMapEntry
{
  std::string trigger;         // e.g. "press"
  std::string trigger_params;  // empty for simple
  std::string action;          // e.g. "incubate"

  ComponentType component_type;
  std::string component_id;  // empty for components without collection, e.g. the Origin component.
};

struct ActionMap
{
  std::vector<ActionMapEntry> mappings;
};

struct ActionMapComponent
{
  FlatMap<std::string /* Domain specific,e.g. FSM  */, ActionMap> action_map;
};

}  // namespace components
}  // namespace sodf

#endif
