#include "FSMController.h"

#include <stdexcept>

FSMController::FSMController(const Config& config, bool beyond_mimic_enabled)
    : fixed_pose_(config.fixed_pose_yaml_path),
      loco_mode_(config),
      beyond_mimic_(config),
      beyond_mimic2_(config),
      beyond_mimic_enabled_(beyond_mimic_enabled) {}

FSMStateName FSMController::currentState() const {
  return current_;
}

#ifdef LOCO_MODE_STATE_TEST_ONLY
void FSMController::setStateForTest(FSMStateName name) {
  current_ = name;
  entered_ = false;
}
#endif

void FSMController::run(const StateAndCmd& state, PolicyOutput& output) {
  FSMState& current_state = stateByName(current_);
  if (!entered_) {
    current_state.enter(state, output);
    entered_ = true;
  }

  current_state.run(state, output);
  const FSMStateName next = current_state.checkChange(state);
  if (next != current_) {
    if (next == FSMStateName::BeyondMimic && !beyond_mimic_enabled_) {
      return;
    }
    current_state.exit(state, output);
    current_ = next;
    FSMState& next_state = stateByName(current_);
    next_state.enter(state, output);
  }
}

const FixedPoseState& FSMController::fixedPoseState() const {
  return fixed_pose_;
}

const LocoModeState& FSMController::locoModeState() const {
  return loco_mode_;
}

const BeyondMimicState& FSMController::beyondMimicState() const {
  return beyond_mimic_;
}

const BeyondMimic2State& FSMController::beyondMimic2State() const {
  return beyond_mimic2_;
}

FSMState& FSMController::stateByName(FSMStateName name) {
  switch (name) {
    case FSMStateName::Passive:
      return passive_;
    case FSMStateName::FixedPose:
      return fixed_pose_;
    case FSMStateName::LocoMode:
      return loco_mode_;
    case FSMStateName::BeyondMimic:
      return beyond_mimic_;
    case FSMStateName::BeyondMimic2:
      return beyond_mimic2_;
  }
  throw std::runtime_error("Unknown FSM state");
}
