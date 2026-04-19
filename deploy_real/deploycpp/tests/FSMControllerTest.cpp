#include <iostream>

#include "FSMController.h"
#include "config/Config.h"

int main() {
  const Config config = loadConfig(ROBOMIMIC_DEPLOY_REPO_ROOT);
  FSMController fsm(config, true);
  StateAndCmd state;
  PolicyOutput output;

  fsm.run(state, output);
  state.remote.start = true;
  fsm.run(state, output);
  state.remote.start = false;

  for (int i = fsm.fixedPoseState().currentStep(); i < fsm.fixedPoseState().totalSteps(); ++i) {
    fsm.run(state, output);
  }

  state.remote.R1 = true;
  state.remote.A = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::LocoMode) {
    std::cerr << "expected FixedPose complete + R1+A -> LocoMode\n";
    return 1;
  }

  state.remote.A = false;
  state.remote.B = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::BeyondMimic2) {
    std::cerr << "expected LocoMode + R1+B -> BeyondMimic2\n";
    return 1;
  }

  state.remote.R1 = false;
  state.remote.B = false;
  state.remote.F1 = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::Passive) {
    std::cerr << "expected BeyondMimic2 + F1 -> Passive\n";
    return 1;
  }

  std::cout << "fsm_controller_test_pass=1\n";
  return 0;
}
