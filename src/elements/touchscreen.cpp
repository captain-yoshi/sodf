#include <sodf/elements/touchscreen.h>

namespace sodf {
namespace elements {

Touchscreen::Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z,
                         behavior::State current_state, behavior::StateManager&& state_manager,
                         ActionToElementIDCallback callback)

  : center_tf_(center_tf)
  , size_y_(size_y)
  , size_z_(size_z)
  , current_state_(current_state)
  , state_manager_(std::move(state_manager))
  , callback_(callback)
  , Element()

{
}

const geometry::Transform& Touchscreen::centerTF() const
{
  return center_tf_;
}

behavior::State Touchscreen::currentState() const
{
  return current_state_;
}

void Touchscreen::setCurrentState(behavior::State state)
{
  current_state_ = state;
}

behavior::ActionSequence Touchscreen::computeActions(behavior::State start, behavior::State end,
                                                     const std::vector<behavior::Action>& end_actions)
{
  return state_manager_.computeActions(start, end, end_actions);
}

ElementSequence Touchscreen::computeActions(const std::string& end, const std::vector<std::string>& end_actions)
{
  auto action_seq = state_manager_.computeActions(currentState(), end, end_actions);

  ElementSequence element_seq;
  element_seq.final_state = action_seq.final_state;

  for (const auto& action : action_seq.actions)
    element_seq.elements.emplace_back(callback_(action));

  return element_seq;
}

ElementSequence Touchscreen::computeActions(const std::string& start, const std::string& end,
                                            const std::vector<std::string>& end_actions)
{
  auto action_seq = state_manager_.computeActions(start, end, end_actions);

  ElementSequence element_seq;
  element_seq.final_state = action_seq.final_state;

  for (const auto& action : action_seq.actions)
    element_seq.elements.emplace_back(callback_(action));

  return element_seq;
}

}  // namespace elements
}  // namespace sodf
