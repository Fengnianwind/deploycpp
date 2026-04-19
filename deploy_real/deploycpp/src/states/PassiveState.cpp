#include "states/PassiveState.h"

FSMStateName PassiveState::name() const {
  return FSMStateName::Passive;
}

void PassiveState::enter(const StateAndCmd&, PolicyOutput& output) {
  output.setDamping(8.0f);
}

void PassiveState::run(const StateAndCmd&, PolicyOutput& output) {
  output.setDamping(8.0f);
}

void PassiveState::exit(const StateAndCmd&, PolicyOutput& output) {
  output.setDamping(8.0f);
}

FSMStateName PassiveState::checkChange(const StateAndCmd& state) const {
  if (state.remote.start) {
    return FSMStateName::FixedPose;
  }
  return FSMStateName::Passive;
}
