#ifndef SODF_COMPONENTS_FINITE_STATE_MACHINE_H_
#define SODF_COMPONENTS_FINITE_STATE_MACHINE_H_

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

#include <sodf/ecs.h>

namespace sodf {
namespace components {

inline int try_transition(const std::vector<std::vector<int>>& transitions, int current_state, int action_id,
                          int invalid_value = -1)
{
  if (current_state < 0 || current_state >= static_cast<int>(transitions.size()))
    return invalid_value;

  const auto& row = transitions[current_state];
  if (action_id < 0 || action_id >= static_cast<int>(row.size()))
    return invalid_value;

  return row[action_id];
}

struct FSMLabels
{
  std::unordered_map<std::string, int> string_to_id;
  std::vector<std::string> id_to_string;

  void add(const std::string& label)
  {
    if (string_to_id.count(label))
      return;
    int id = static_cast<int>(id_to_string.size());
    string_to_id[label] = id;
    id_to_string.push_back(label);
  }

  int to_id(const std::string& label) const
  {
    return string_to_id.at(label);
  }

  const std::string& to_string(int id) const
  {
    return id_to_string.at(id);
  }

  bool has_label(const std::string& label) const
  {
    return string_to_id.count(label);
  }

  bool has_id(int id) const
  {
    return id >= 0 && id < static_cast<int>(id_to_string.size());
  }
};

struct FSM
{
  int current_state;
  int start_state;

  FSMLabels state_labels;
  FSMLabels action_labels;

  std::vector<std::vector<int>> transitions;  // transitions[from_state][action_id] = to_state
};

struct FSMComponent
{
  FlatMap<std::string, FSM> fsm_map;
};

struct ActionMapEntry
{
  std::string trigger;  // e.g. "press"
  std::string action;   // e.g. "incubate"
};

struct ActionMap
{
  std::string action_id;  // e.g. "btn/incubate"
  std::vector<ActionMapEntry> mappings;
};

struct ActionMapComponent
{
  FlatMap<std::string /* FSM ID */, ActionMap> action_map;
};

}  // namespace components
}  // namespace sodf

#endif
