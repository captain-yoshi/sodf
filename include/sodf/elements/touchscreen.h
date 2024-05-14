#ifndef TOUCHSCREEN_H_
#define TOUCHSCREEN_H_

#include <functional>

#include <sodf/behavior/state.h>
#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

using ActionElementMap = std::map<behavior::Action, ElementID>;

using ActionsToElementIDCallback = std::function<std::string(behavior::Action)>;

class Touchscreen : public Element
{
public:
  Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z, int current_state,
              behavior::StateManager&& state_manager, ActionsToElementIDCallback callback);

  virtual ~Touchscreen(){};

  const geometry::Transform& centerTF() const;
  double pressForce() const;

  int currentState() const;
  void setCurrentState(int state);

  std::vector<ElementID> computeActions(behavior::State start, behavior::State end,
                                        const std::vector<behavior::Action>& end_actions, behavior::State& final_state);

  virtual const geometry::Transform* getTransform() const override
  {
    return &center_tf_;
  };

protected:
  const geometry::Transform center_tf_;  // +X points towards the center of the screen
                                         // +Y parallel to bottom/top screen and points towards the left screen
                                         // +Z parallel to left/right screen and points towards the top screen
  const double size_y_;
  const double size_z_;

  int current_state_;
  behavior::StateManager state_manager_;
  ActionsToElementIDCallback callback_;
};

}  // namespace elements
}  // namespace sodf

#endif  // TOUCHSCREEN_H_
