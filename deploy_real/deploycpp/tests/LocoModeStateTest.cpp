#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/Config.h"
#include "FSMController.h"
#include "states/LocoModeState.h"

namespace {

bool approxEqual(float lhs, float rhs, float tol = 1.0e-5f) {
  return std::fabs(lhs - rhs) <= tol;
}

float scaleToRange(float value, float min_value, float max_value) {
  return (value + 1.0f) * (max_value - min_value) * 0.5f + min_value;
}

}  // namespace

int main() {
  const Config config = loadConfig(ROBOMIMIC_DEPLOY_REPO_ROOT);
  LocoModeState state(config);
  StateAndCmd input;
  input.vel_cmd = {0.25f, -0.5f, 0.75f};
  PolicyOutput output;
  state.enter(input, output);

  const std::vector<float> obs = state.buildObsForTest(input);
  if (obs.size() != 96) {
    std::cerr << "expected 96-dim observation, got " << obs.size() << "\n";
    return 1;
  }

  const float expected_vel_x = scaleToRange(input.vel_cmd[0], -0.4f, 0.7f);
  const float expected_vel_y = scaleToRange(input.vel_cmd[1], -0.4f, 0.4f);
  const float expected_vel_z = scaleToRange(input.vel_cmd[2], -1.57f, 1.57f);
  if (!approxEqual(obs[6], expected_vel_x) || !approxEqual(obs[7], expected_vel_y) ||
      !approxEqual(obs[8], expected_vel_z)) {
    std::cerr << "unexpected vel_cmd observation mapping: [" << obs[6] << ", " << obs[7] << ", " << obs[8]
              << "]\n";
    return 1;
  }

  state.run(input, output);
  if (!output.isFinite()) {
    std::cerr << "loco output not finite\n";
    return 1;
  }

  StateAndCmd f1_input;
  f1_input.remote.F1 = true;
  if (state.checkChange(f1_input) != FSMStateName::Passive) {
    std::cerr << "expected F1 -> Passive\n";
    return 1;
  }

  StateAndCmd start_input;
  start_input.remote.start = true;
  if (state.checkChange(start_input) != FSMStateName::FixedPose) {
    std::cerr << "expected Start -> FixedPose\n";
    return 1;
  }

  StateAndCmd beyond_mimic_input;
  beyond_mimic_input.remote.L1 = true;
  beyond_mimic_input.remote.Y = true;
  if (state.checkChange(beyond_mimic_input) != FSMStateName::BeyondMimic) {
    std::cerr << "expected L1+Y -> BeyondMimic\n";
    return 1;
  }

  StateAndCmd beyond_mimic2_input;
  beyond_mimic2_input.remote.R1 = true;
  beyond_mimic2_input.remote.B = true;
  if (state.checkChange(beyond_mimic2_input) != FSMStateName::BeyondMimic2) {
    std::cerr << "expected R1+B -> BeyondMimic2\n";
    return 1;
  }

  FSMController controller(config);
  controller.setStateForTest(FSMStateName::LocoMode);
  controller.run(beyond_mimic2_input, output);
  if (controller.currentState() != FSMStateName::BeyondMimic2) {
    std::cerr << "expected controller to transition into BeyondMimic2\n";
    return 1;
  }

  StateAndCmd back_to_loco_input;
  back_to_loco_input.remote.R1 = true;
  back_to_loco_input.remote.A = true;
  controller.run(back_to_loco_input, output);
  if (controller.currentState() != FSMStateName::LocoMode) {
    std::cerr << "expected BeyondMimic2 to transition back into LocoMode\n";
    return 1;
  }

  std::cout << "loco_mode_state_test_pass=1\n";
  return 0;
}
