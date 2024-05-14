#include <sodf/elements/touchscreen.h>

namespace sodf {
namespace elements {

Touchscreen::Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z, int current_state,
                         behavior::StateManager&& state_manager, ActionsToElementIDCallback callback)

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

int Touchscreen::currentState() const
{
  return current_state_;
}

void Touchscreen::setCurrentState(int state)
{
  current_state_ = state;
}

std::vector<ElementID> Touchscreen::computeActions(behavior::State start, behavior::State end,
                                                   const std::vector<behavior::Action>& end_actions,
                                                   behavior::State& final_state)
{
  std::vector<behavior::Action> actions = state_manager_.computeActions(start, end, end_actions, final_state);

  std::vector<ElementID> elements;

  for (const auto& action : actions)
    elements.push_back(callback_(action));

  return elements;
}

}  // namespace elements
}  // namespace sodf
