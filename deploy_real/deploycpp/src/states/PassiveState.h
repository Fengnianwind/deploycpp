#pragma once

#include "states/FSMState.h"

class PassiveState : public FSMState {
 public:
  FSMStateName name() const override;
  void enter(const StateAndCmd& state, PolicyOutput& output) override;
  void run(const StateAndCmd& state, PolicyOutput& output) override;
  void exit(const StateAndCmd& state, PolicyOutput& output) override;
  FSMStateName checkChange(const StateAndCmd& state) const override;
};
