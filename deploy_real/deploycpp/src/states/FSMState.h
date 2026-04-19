#pragma once

#include "types/PolicyOutput.h"
#include "types/StateAndCmd.h"

enum class FSMStateName {
  Passive,
  FixedPose,
  LocoMode,
  BeyondMimic,
  BeyondMimic2,
};

class FSMState {
 public:
  virtual ~FSMState() = default;
  virtual FSMStateName name() const = 0;
  virtual void enter(const StateAndCmd& state, PolicyOutput& output) = 0;
  virtual void run(const StateAndCmd& state, PolicyOutput& output) = 0;
  virtual void exit(const StateAndCmd& state, PolicyOutput& output) = 0;
  virtual FSMStateName checkChange(const StateAndCmd& state) const = 0;
};
