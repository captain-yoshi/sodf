#ifndef STATE_H_
#define STATE_H_

#include <map>
#include <string>
#include <string_view>

#include <sodf/element.h>
#include <sodf/graph/directed_graph.h>

namespace sodf {
namespace behavior {

using State = int;
using Action = int;

/* using StateTransitions = std::map<State, ElementID>; */
/* using ActionTransitions = std::map<Action, std::pair<ElementID, State>>; */
/* using StateTransitions = std::map<State, ElementID>; */
using ActionTransitions = std::map<Action, State>;

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
  StateManager(StateNodeMap&& node_map);

  std::vector<Action> computeActions(State begin, State end, const std::vector<Action>& end_actions, State& final);

private:
  std::unique_ptr<graph::DirectedGraph> digraph_;

  StateNodeMap node_map_;
};

}  // namespace behavior
}  // namespace sodf

#endif  // STATE_H_
