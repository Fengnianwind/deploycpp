#include <array>
#include <cmath>
#include <iostream>

#include "config/Config.h"
#include "states/BeyondMimicState.h"

namespace {

bool arraysEqual(const std::array<float, 29>& lhs, const std::array<float, 29>& rhs, float tol = 1.0e-6f) {
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (std::fabs(lhs[i] - rhs[i]) > tol) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  const Config config = loadConfig(ROBOMIMIC_DEPLOY_REPO_ROOT);
  BeyondMimicState state(config);
  StateAndCmd input;
  PolicyOutput output;
  for (std::size_t i = 0; i < output.actions.size(); ++i) {
    output.actions[i] = 100.0f + static_cast<float>(i);
    output.kps[i] = 200.0f + static_cast<float>(i);
    output.kds[i] = 300.0f + static_cast<float>(i);
  }
  const PolicyOutput seeded_output = output;

  state.enter(input, output);
  if (!output.isFinite()) {
    std::cerr << "enter should write finite hold output\n";
    return 1;
  }
  if (arraysEqual(output.actions, seeded_output.actions) && arraysEqual(output.kps, seeded_output.kps) &&
      arraysEqual(output.kds, seeded_output.kds)) {
    std::cerr << "enter should overwrite stale output during warmup\n";
    return 1;
  }

  state.run(input, output);
  if (!output.isFinite()) {
    std::cerr << "first warmup frame should keep finite hold output\n";
    return 1;
  }
  if (arraysEqual(output.actions, seeded_output.actions) && arraysEqual(output.kps, seeded_output.kps) &&
      arraysEqual(output.kds, seeded_output.kds)) {
    std::cerr << "first warmup frame should not preserve stale output\n";
    return 1;
  }

  state.run(input, output);
  if (!output.isFinite()) {
    std::cerr << "second warmup frame should keep finite hold output\n";
    return 1;
  }
  if (arraysEqual(output.actions, seeded_output.actions) && arraysEqual(output.kps, seeded_output.kps) &&
      arraysEqual(output.kds, seeded_output.kds)) {
    std::cerr << "second warmup frame should not preserve stale output\n";
    return 1;
  }

  state.run(input, output);
  if (!output.isFinite()) {
    std::cerr << "beyond mimic output not finite\n";
    return 1;
  }
  if (arraysEqual(output.actions, seeded_output.actions) && arraysEqual(output.kps, seeded_output.kps) &&
      arraysEqual(output.kds, seeded_output.kds)) {
    std::cerr << "third frame should write policy output after warmup\n";
    return 1;
  }

  std::cout << "beyond_mimic_state_test_pass=1\n";
  return 0;
}
