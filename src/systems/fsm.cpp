#include <sodf/systems/fsm.h>

#include <algorithm>
#include <iostream>
#include <string>

namespace sodf {
namespace system {

std::vector<FSMActionStep> simulate_fsm_sequence_strict(const components::FSM& fsm,
                                                        const components::ActionMapComponent& map_comp,
                                                        const std::string& fsm_id,
                                                        const std::vector<std::string>& action_labels)
{
  std::vector<FSMActionStep> result;
  int current = fsm.current_state;

  // Find ActionMap once for this FSM id.
  auto it = std::find_if(map_comp.elements.begin(), map_comp.elements.end(),
                         [&](const auto& pair) { return pair.first == fsm_id; });
  const components::ActionMap* action_map = (it != map_comp.elements.end()) ? &it->second : nullptr;

  for (const auto& label : action_labels)
  {
    if (!fsm.action_labels.has_label(label))
    {
      std::cerr << "ERROR: Action label '" << label << "' is not defined in FSM\n";
      break;
    }

    const int action_id = fsm.action_labels.to_id(label);
    const int next = components::try_transition(fsm.transitions, current, action_id);

    if (next < 0)
    {
      std::cerr << "ERROR: No valid transition for action '" << label << "' in state '"
                << fsm.state_labels.to_string(current) << "'\n";
      break;
    }

    FSMActionStep step{ current, action_id, next, label, std::nullopt };

    // Use only the ActionMap for this FSM
    if (action_map)
    {
      for (const auto& entry : action_map->mappings)
      {
        if (entry.action == label)
        {
          step.trigger_info = std::make_pair(entry.component_id, entry.trigger);
          break;  // first match only
        }
      }
    }
    else
    {
      std::cerr << "WARNING: No ActionMap for FSM '" << fsm_id << "'\n";
    }

    result.push_back(step);
    current = next;
  }

  return result;
}

void simulate_action_sequence_on_all(ecs::Database& db, const std::string& fsm_id,
                                     const std::vector<std::string>& actions)
{
  db.each([&](ecs::Database::entity_type /*eid*/, const components::FSMComponent& fsm_comp,
              const components::ActionMapComponent& map_comp) {
    for (const auto& [key, fsm] : fsm_comp.elements)
    {
      if (key != fsm_id)
        continue;

      auto steps = simulate_fsm_sequence_strict(fsm, map_comp, key, actions);

      std::cout << "\nSimulated FSM Path for FSM: " << key << "\n";
      for (const auto& step : steps)
      {
        std::cout << "  " << fsm.state_labels.to_string(step.from_state) << " + " << step.action_label << " -> "
                  << fsm.state_labels.to_string(step.to_state) << "\n";
      }

      std::cout << "Required ActionMap triggers:\n";
      for (const auto& step : steps)
      {
        if (step.trigger_info)
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
