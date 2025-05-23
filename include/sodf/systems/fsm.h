#ifndef SODF_SYSTEM_FSM_GRAPH_H_
#define SODF_SYSTEM_FSM_GRAPH_H_

#include <sodf/ecs.h>

#include <iostream>
#include <vector>
#include <string>
#include <sodf/ecs.h>
#include <sodf/components/finite_state_machine.h>
#include <sodf/components/button.h>

namespace sodf {
namespace system {

using namespace components;

struct FSMActionStep
{
  int from_state;
  int action_id;
  int to_state;
  std::string action_label;
  std::optional<std::pair<std::string, std::string>> trigger_info;
};

std::vector<FSMActionStep> simulate_fsm_sequence_strict(const FSM& fsm, const ActionMapComponent& map_comp,
                                                        const std::string& fsm_id,
                                                        const std::vector<std::string>& action_labels)
{
  std::vector<FSMActionStep> result;
  int current = fsm.current_state;

  for (const auto& label : action_labels)
  {
    if (!fsm.action_labels.has_label(label))
    {
      std::cerr << "ERROR: Action label '" << label << "' is not defined in FSM\n";
      break;
    }

    int action_id = fsm.action_labels.to_id(label);
    int next = try_transition(fsm.transitions, current, action_id);

    if (next < 0)
    {
      std::cerr << "ERROR: No valid transition for action '" << label << "' in state '"
                << fsm.state_labels.to_string(current) << "'\n";
      break;
    }

    FSMActionStep step{ current, action_id, next, label, std::nullopt };

    for (const auto& [mapped_fsm, action_map] : map_comp.action_map)
    {
      if (mapped_fsm != fsm_id)
        continue;
      for (const auto& entry : action_map.mappings)
      {
        if (entry.action == label)
        {
          step.trigger_info = std::make_pair(action_map.action_id, entry.trigger);
          break;
        }
      }
    }

    result.push_back(step);
    current = next;
  }

  return result;
}

void simulate_action_sequence_on_all(ginseng::database& db, const std::string& fsm_id,
                                     const std::vector<std::string>& actions)
{
  db.visit([&](const FSMComponent& fsm_comp, const ActionMapComponent& map_comp) {
    for (const auto& [fsm_key, fsm] : fsm_comp.fsm_map)
    {
      if (fsm_key != fsm_id)
        continue;

      auto steps = simulate_fsm_sequence_strict(fsm, map_comp, fsm_key, actions);
      std::cout << "\nSimulated FSM Path for FSM: " << fsm_key << "\n";
      for (const auto& step : steps)
      {
        std::cout << "  " << fsm.state_labels.to_string(step.from_state) << " + " << step.action_label << " -> "
                  << fsm.state_labels.to_string(step.to_state) << "\n";
      }

      std::cout << "Required ActionMap triggers:\n";
      for (const auto& step : steps)
      {
        if (step.trigger_info.has_value())
        {
          const auto& [component, trigger] = *step.trigger_info;
          std::cout << "  trigger='" << trigger << "' on component='" << component << "'\n";
        }
        else
        {
          std::cerr << "  ERROR: No ActionMap found for action '" << step.action_label << "'\n";
        }
      }
    }
  });
}

}  // namespace system
}  // namespace sodf

#endif  // SODF_SYSTEM_FSM_GRAPH_H_
