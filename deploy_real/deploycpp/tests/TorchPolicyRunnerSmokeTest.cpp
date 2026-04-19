#include <iostream>
#include <vector>

#include "config/Config.h"
#include "policy/TorchPolicyRunner.h"

int main() {
  const Config config = loadConfig(ROBOMIMIC_DEPLOY_REPO_ROOT);
  TorchPolicyRunner runner(config.loco_mode_model_path);
  const std::vector<float> obs(96, 0.0f);
  const std::vector<float> action = runner.run(obs);
  if (action.size() != 29) {
    std::cerr << "expected 29 actions, got " << action.size() << "\n";
    return 1;
  }
  std::cout << "torch_policy_runner_smoke_test_pass=1\n";
  return 0;
}
