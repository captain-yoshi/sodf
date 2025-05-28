#ifndef SODF_SYSTEM_FSM_GRAPH_H_
#define SODF_SYSTEM_FSM_GRAPH_H_

#include <iostream>
#include <vector>
#include <string>

#include <sodf/ecs.h>
#include <sodf/components/finite_state_machine.h>

namespace sodf {
namespace system {

struct FSMActionStep
{
  int from_state;
  int action_id;
  int to_state;
  std::string action_label;
  std::optional<std::pair<std::string, std::string>> trigger_info;
};

std::vector<FSMActionStep> simulate_fsm_sequence_strict(const components::FSM& fsm,
                                                        const components::ActionMapComponent& map_comp,
                                                        const std::string& fsm_id,
                                                        const std::vector<std::string>& action_labels);

void simulate_action_sequence_on_all(ginseng::database& db, const std::string& fsm_id,
                                     const std::vector<std::string>& actions);

}  // namespace system
}  // namespace sodf

#endif  // SODF_SYSTEM_FSM_GRAPH_H_
