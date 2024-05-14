#include <sodf/behavior/state.h>

#include <gtest/gtest.h>

#include <enum.h>
#include <limits.h>

using namespace sodf;
using namespace sodf::behavior;

BETTER_ENUM(HMIStates,                           //
            int,                                 //
            home = 0,                            //
            incubate,                            //
            saved_protocols,                     //
            incubate_bloc_temperature_keyboard,  //
            incubate_hold_time_keyboard,         //
            incubate_lid_time_keyboard,          //
            run,                                 //
            run_cancelled,                       //
            popup_before_canceling_run           //
);

BETTER_ENUM(HMIActions,
            int,  //
            // home
            incubate = HMIStates::_size(),  //
            saved_protocols,                //
            run,                            //

            // Numeric Keyboard
            cancel,    //
            ok,        //
            zero,      //
            one,       //
            two,       //
            tree,      //
            four,      //
            five,      //
            six,       //
            seven,     //
            eight,     //
            nine,      //
            dot,       //
            infinity,  //

            // Incubate
            bloc_temperature,  //
            hold_time,         //
            lid_time);         //

Action actionFromStringCallback(const std::string& action_string)
{
  auto action = HMIActions::_from_string(action_string.c_str());

  return action._to_integral();
}

State stateFromStringCallback(const std::string& state_string)
{
  auto state = HMIStates::_from_string(state_string.c_str());

  return state._to_integral();
}

TEST(StateManager, computeActions)
{
  StateNodeMap node_map;

  // State HOME
  {
    StateNode node(HMIStates::home);
    node.addActionTransition(HMIActions::incubate, HMIStates::incubate);
    node.addActionTransition(HMIActions::saved_protocols, HMIStates::saved_protocols);

    node_map.emplace(node.state(), std::move(node));
  }

  // State Incubate
  {
    StateNode node(HMIStates::incubate);
    node.addActionTransition(HMIActions::bloc_temperature, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::hold_time, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::lid_time, HMIStates::incubate_lid_time_keyboard);

    node_map.emplace(node.state(), std::move(node));
  }

  // State Incubate Bloc Temperature Keyboard
  {
    StateNode node(HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::zero, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::one, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::two, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::tree, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::four, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::five, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::six, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::seven, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::eight, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::nine, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::dot, HMIStates::incubate_bloc_temperature_keyboard);
    node.addActionTransition(HMIActions::ok, HMIStates::incubate);

    node_map.emplace(node.state(), std::move(node));
  }

  // State Incubate Hold Time Keyboard
  {
    StateNode node(HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::zero, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::one, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::two, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::tree, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::four, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::five, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::six, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::seven, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::eight, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::nine, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::dot, HMIStates::incubate_hold_time_keyboard);
    node.addActionTransition(HMIActions::ok, HMIStates::incubate);

    node_map.emplace(node.state(), std::move(node));
  }

  StateManager sm(std::move(node_map), actionFromStringCallback, stateFromStringCallback);

  auto seq =
      sm.computeActions(HMIStates::home,                                //
                        HMIStates::incubate_bloc_temperature_keyboard,  //
                        std::vector<Action>{ HMIActions::zero, HMIActions::zero, HMIActions::seven, HMIActions::ok,
                                             HMIActions::hold_time, HMIActions::one, HMIActions::two, HMIActions::tree

                        });

  EXPECT_EQ(10, seq.actions.size());

  EXPECT_EQ(9, seq.actions[0]);
  EXPECT_EQ(26, seq.actions[1]);
  EXPECT_EQ(14, seq.actions[2]);
  EXPECT_EQ(14, seq.actions[3]);
  EXPECT_EQ(21, seq.actions[4]);
  EXPECT_EQ(13, seq.actions[5]);
  EXPECT_EQ(27, seq.actions[6]);
  EXPECT_EQ(15, seq.actions[7]);
  EXPECT_EQ(16, seq.actions[8]);
  EXPECT_EQ(17, seq.actions[9]);

  EXPECT_EQ(4, seq.final_state);

  EXPECT_STREQ("incubate", HMIActions::_from_integral(seq.actions[0])._to_string());
  EXPECT_STREQ("bloc_temperature", HMIActions::_from_integral(seq.actions[1])._to_string());
  EXPECT_STREQ("zero", HMIActions::_from_integral(seq.actions[2])._to_string());
  EXPECT_STREQ("zero", HMIActions::_from_integral(seq.actions[3])._to_string());
  EXPECT_STREQ("seven", HMIActions::_from_integral(seq.actions[4])._to_string());
  EXPECT_STREQ("ok", HMIActions::_from_integral(seq.actions[5])._to_string());
  EXPECT_STREQ("hold_time", HMIActions::_from_integral(seq.actions[6])._to_string());
  EXPECT_STREQ("one", HMIActions::_from_integral(seq.actions[7])._to_string());
  EXPECT_STREQ("two", HMIActions::_from_integral(seq.actions[8])._to_string());
  EXPECT_STREQ("tree", HMIActions::_from_integral(seq.actions[9])._to_string());

  EXPECT_STREQ("incubate_hold_time_keyboard", HMIStates::_from_integral(seq.final_state)._to_string());

  seq = sm.computeActions(HMIStates::_from_integral(HMIStates::home)._to_string(),                                //
                          HMIStates::_from_integral(HMIStates::incubate_bloc_temperature_keyboard)._to_string(),  //
                          std::vector<std::string>{
                              HMIActions::_from_integral(HMIActions::zero)._to_string(),       //
                              HMIActions::_from_integral(HMIActions::zero)._to_string(),       //
                              HMIActions::_from_integral(HMIActions::seven)._to_string(),      //
                              HMIActions::_from_integral(HMIActions::ok)._to_string(),         //
                              HMIActions::_from_integral(HMIActions::hold_time)._to_string(),  //
                              HMIActions::_from_integral(HMIActions::one)._to_string(),        //
                              HMIActions::_from_integral(HMIActions::two)._to_string(),        //
                              HMIActions::_from_integral(HMIActions::tree)._to_string()        //

                          });

  EXPECT_EQ(10, seq.actions.size());

  EXPECT_EQ(9, seq.actions[0]);
  EXPECT_EQ(26, seq.actions[1]);
  EXPECT_EQ(14, seq.actions[2]);
  EXPECT_EQ(14, seq.actions[3]);
  EXPECT_EQ(21, seq.actions[4]);
  EXPECT_EQ(13, seq.actions[5]);
  EXPECT_EQ(27, seq.actions[6]);
  EXPECT_EQ(15, seq.actions[7]);
  EXPECT_EQ(16, seq.actions[8]);
  EXPECT_EQ(17, seq.actions[9]);

  EXPECT_EQ(4, seq.final_state);

  EXPECT_STREQ("incubate", HMIActions::_from_integral(seq.actions[0])._to_string());
  EXPECT_STREQ("bloc_temperature", HMIActions::_from_integral(seq.actions[1])._to_string());
  EXPECT_STREQ("zero", HMIActions::_from_integral(seq.actions[2])._to_string());
  EXPECT_STREQ("zero", HMIActions::_from_integral(seq.actions[3])._to_string());
  EXPECT_STREQ("seven", HMIActions::_from_integral(seq.actions[4])._to_string());
  EXPECT_STREQ("ok", HMIActions::_from_integral(seq.actions[5])._to_string());
  EXPECT_STREQ("hold_time", HMIActions::_from_integral(seq.actions[6])._to_string());
  EXPECT_STREQ("one", HMIActions::_from_integral(seq.actions[7])._to_string());
  EXPECT_STREQ("two", HMIActions::_from_integral(seq.actions[8])._to_string());
  EXPECT_STREQ("tree", HMIActions::_from_integral(seq.actions[9])._to_string());

  EXPECT_STREQ("incubate_hold_time_keyboard", HMIStates::_from_integral(seq.final_state)._to_string());
  /*
  std::cout << "====== ACTIONS =======" << std::endl;
  for (const auto& action : actions)
  {
    HMIActions a = HMIActions::_from_integral(action);
    std::cout << action << " -> " << a._to_string() << std::endl;
  }

  std::cout << "final state = " << HMIStates::_from_integral(final)._to_string() << std::endl;
  */
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
