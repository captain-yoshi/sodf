#ifndef STATE_H_
#define STATE_H_

#include <map>
#include <string>
#include <string_view>


#include <sodf/graph/directed_graph.h>

namespace sodf {
namespace behavior {

using State = int;
using Action = int;

using ActionTransitions = std::map<Action, State>;

struct ActionSequence
{
  std::vector<Action> actions;
  State final_state;
};

using ActionFromStringCallback = std::function<behavior::Action(const std::string&)>;
using StateFromStringCallback = std::function<behavior::Action(const std::string&)>;

class StateNode
{
public:
  StateNode(State state);

  State state() const;

  /* void addStateTransition(State state, Action); */
  void addActionTransition(Action action, State state);

  /* const StateTransitions& getStateTransitions() const; */
  const ActionTransitions& getActionTransitions() const;

private:
  State state_;
  /* StateTransitions state_transitions_; */
  ActionTransitions action_transitions_;
};

using StateNodeMap = std::map<State, StateNode>;

class StateManager
{
public:
  StateManager(StateNodeMap&& node_map, ActionFromStringCallback action_from_string_cb,
               StateFromStringCallback state_from_string_cb);

  ActionSequence computeActions(State start, State end, const std::vector<Action>& end_actions);
  ActionSequence computeActions(State start, const std::string& end, const std::vector<std::string>& end_actions);
  ActionSequence computeActions(const std::string& start, const std::string& end,
                                const std::vector<std::string>& end_actions);

private:
  std::unique_ptr<graph::DirectedGraph> digraph_;

  StateNodeMap node_map_;

  ActionFromStringCallback action_from_string_cb_;
  StateFromStringCallback state_from_string_cb_;
};

}  // namespace behavior
}  // namespace sodf

#endif  // STATE_H_
