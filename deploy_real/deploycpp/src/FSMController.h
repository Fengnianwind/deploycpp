#pragma once

#include "config/Config.h"
#include "states/BeyondMimicState.h"
#include "states/BeyondMimic2State.h"
#include "states/FixedPoseState.h"
#include "states/LocoModeState.h"
#include "states/PassiveState.h"
#include "types/PolicyOutput.h"
#include "types/StateAndCmd.h"

class FSMController {
 public:
  explicit FSMController(const Config& config, bool beyond_mimic_enabled = true);

  FSMStateName currentState() const;
#ifdef LOCO_MODE_STATE_TEST_ONLY
  void setStateForTest(FSMStateName name);
#endif
  void run(const StateAndCmd& state, PolicyOutput& output);
  const FixedPoseState& fixedPoseState() const;
  const LocoModeState& locoModeState() const;
  const BeyondMimicState& beyondMimicState() const;
  const BeyondMimic2State& beyondMimic2State() const;

 private:
  FSMState& stateByName(FSMStateName name);

  PassiveState passive_;
  FixedPoseState fixed_pose_;
  LocoModeState loco_mode_;
  BeyondMimicState beyond_mimic_;
  BeyondMimic2State beyond_mimic2_;
  FSMStateName current_ = FSMStateName::Passive;
  bool entered_ = false;
  bool beyond_mimic_enabled_ = true;
};
