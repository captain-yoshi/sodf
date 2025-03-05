#ifndef TOUCHSCREEN_H_
#define TOUCHSCREEN_H_

#include <functional>

#include <sodf/behavior/state.h>
#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

using ActionToElementIDCallback = std::function<std::string(behavior::Action)>;

struct ElementSequence
{
  std::vector<ElementID> elements;
  behavior::State final_state;
};

class Touchscreen : public Element
{
public:
  Touchscreen(const geometry::Transform& center_tf, double size_y, double size_z, behavior::State current_state,
              behavior::StateManager&& state_manager, ActionToElementIDCallback callback);

  virtual ~Touchscreen(){};

  const geometry::Transform& centerTF() const;
  double pressForce() const;

  behavior::State currentState() const;
  void setCurrentState(behavior::State state);

  behavior::ActionSequence computeActions(behavior::State start, behavior::State end,
                                          const std::vector<behavior::Action>& end_actions);

  ElementSequence computeActions(const std::string& start, const std::string& end,
                                 const std::vector<std::string>& end_actions);

  ElementSequence computeActions(const std::string& end, const std::vector<std::string>& end_actions);

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

  behavior::State current_state_;
  behavior::StateManager state_manager_;
  ActionToElementIDCallback callback_;
};

}  // namespace elements
}  // namespace sodf

#endif  // TOUCHSCREEN_H_
