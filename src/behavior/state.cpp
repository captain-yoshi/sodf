#include <sodf/behavior/state.h>

namespace sodf {
namespace behavior {

StateNode::StateNode(State state) : state_(state)
{
}

State StateNode::state() const
{
  return state_;
}

// void StateNode::addStateTransition(State state, const ElementID& element_id)
// {
//   auto node = state_transitions_.find(state);
//   if (node != state_transitions_.end())
//     throw std::runtime_error("state '" + std::to_string(state) + "' is already added");

//   state_transitions_.emplace(state, element_id);
// }

void StateNode::addActionTransition(Action action, State state)
{
  // if (state < 0)
  //   throw std::runtime_error("state '" + std::to_string(state) + "' must be positive defnied (including 0)");
  // if (action <= 0)
  //   throw std::runtime_error("action '" + std::to_string(action) + "' must be positive defnied");

  auto node = action_transitions_.find(action);
  if (node != action_transitions_.end())
    throw std::runtime_error("action '" + std::to_string(action) + "' is already added");

  action_transitions_.emplace(action, state);
}

// const StateTransitions& StateNode::getStateTransitions() const
// {
//   return state_transitions_;
// }

const ActionTransitions& StateNode::getActionTransitions() const
{
  return action_transitions_;
}
StateManager::StateManager(StateNodeMap&& node_map, ActionFromStringCallback action_from_string_cb,
                           StateFromStringCallback state_from_string_cb)

  : node_map_(std::move(node_map))
  , action_from_string_cb_(action_from_string_cb)
  , state_from_string_cb_(state_from_string_cb)

{
  // Create graph edges
  std::vector<graph::DirectedGraph::Edge> edges;
  for (const auto& node : node_map_)
  {
    const auto& action_transitions = node.second.getActionTransitions();
    for (const auto& action : action_transitions)
    {
      edges.emplace_back(graph::DirectedGraph::Edge(node.second.state(), action.first));  //
      edges.emplace_back(graph::DirectedGraph::Edge(action.first, action.second));        // state, action
    }
  }

  // create directed graph
  std::vector<int> weights(edges.size(), 1);
  digraph_ = std::make_unique<graph::DirectedGraph>(std::move(edges), std::move(weights));
}

ActionSequence StateManager::computeActions(State start, State end, const std::vector<Action>& end_actions)
{
  // find shortest state path
  std::deque<int> path;
  if (!digraph_->findShortestPath(start, end, path))
    throw std::runtime_error("Could not find shortest path");

  // store sequence of elements wrt. the path
  ActionSequence action_seq;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    const auto& state = path[i];

    // path actions are on odd indexes
    if (i % 2)
      action_seq.actions.emplace_back(state);
  }

  // store sequence of elements wrt. end_actions
  State current_state = path.back();
  for (const auto& action : end_actions)
  {
    const auto& it = node_map_.find(current_state);
    if (it == node_map_.end())
      throw std::runtime_error("Could not find state in node_map");

    const auto& action_transitions = it->second.getActionTransitions();

    const auto& it_transitions = action_transitions.find(action);
    if (it_transitions == action_transitions.end())
      throw std::runtime_error("Could not find action '" + std::to_string(action) + "' in state '" +
                               std::to_string(current_state) + "'");

    action_seq.actions.emplace_back(action);
    current_state = it_transitions->second;
  }
  action_seq.final_state = current_state;
  return action_seq;
}

}  // namespace behavior
}  // namespace sodf
