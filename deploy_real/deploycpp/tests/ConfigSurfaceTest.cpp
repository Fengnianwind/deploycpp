#include <fstream>
#include <iostream>
#include <string>

#include "config/Config.h"
#include "types/StateAndCmd.h"

static_assert(static_cast<int>(FSMCommand::LocoMode) >= 0, "FSMCommand::LocoMode must exist");
static_assert(static_cast<int>(FSMCommand::BeyondMimic2) >= 0, "FSMCommand::BeyondMimic2 must exist");

namespace {

bool endsWith(const std::string& value, const std::string& suffix) {
  if (value.size() < suffix.size()) {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool contains(const std::string& value, const std::string& needle) {
  return value.find(needle) != std::string::npos;
}

}  // namespace

int main() {
  const Config config = loadConfig(ROBOMIMIC_DEPLOY_REPO_ROOT);
  const std::string cmake_lists_path = std::string(ROBOMIMIC_DEPLOY_REPO_ROOT) + "/deploy_real/deploycpp/CMakeLists.txt";

  if (!endsWith(config.loco_mode_yaml_path, "/policy/loco_mode/config/LocoMode.yaml")) {
    std::cerr << "loco_mode_yaml_path mismatch\n";
    return 1;
  }
  if (!contains(config.loco_mode_model_path, "/policy/loco_mode/model/")) {
    std::cerr << "loco_mode_model_path mismatch\n";
    return 1;
  }
  if (!endsWith(config.loco_mode_model_path, ".pt")) {
    std::cerr << "loco_mode_model_path suffix mismatch\n";
    return 1;
  }
  if (!endsWith(config.beyond_mimic2_yaml_path, "/policy/beyond_mimic2/config/BeyondMimic2.yaml")) {
    std::cerr << "beyond_mimic2_yaml_path mismatch\n";
    return 1;
  }
  if (!contains(config.beyond_mimic2_model_path, "/policy/beyond_mimic2/model/")) {
    std::cerr << "beyond_mimic2_model_path mismatch\n";
    return 1;
  }
  if (!endsWith(config.beyond_mimic2_model_path, ".onnx")) {
    std::cerr << "beyond_mimic2_model_path suffix mismatch\n";
    return 1;
  }
  if (!endsWith(config.beyond_mimic_yaml_path, "/policy/beyond_mimic/config/BeyondMimic.yaml")) {
    std::cerr << "beyond_mimic_yaml_path mismatch\n";
    return 1;
  }
  if (!contains(config.beyond_mimic_model_path, "/policy/beyond_mimic/model/")) {
    std::cerr << "beyond_mimic_model_path mismatch\n";
    return 1;
  }
  if (!endsWith(config.beyond_mimic_model_path, ".onnx")) {
    std::cerr << "beyond_mimic_model_path suffix mismatch\n";
    return 1;
  }

  StateAndCmd state;
  state.vel_cmd[0] = 1.0f;
  state.vel_cmd[1] = 2.0f;
  state.vel_cmd[2] = 3.0f;
  if (state.vel_cmd[0] != 1.0f || state.vel_cmd[1] != 2.0f || state.vel_cmd[2] != 3.0f) {
    std::cerr << "vel_cmd mismatch\n";
    return 1;
  }

  std::ifstream cmake_file(cmake_lists_path);
  if (!cmake_file) {
    std::cerr << "failed to open CMakeLists.txt\n";
    return 1;
  }
  std::string cmake_contents((std::istreambuf_iterator<char>(cmake_file)), std::istreambuf_iterator<char>());
  if (!contains(cmake_contents, "/home/unitree/onnxruntime-linux-aarch64-1.19.2")) {
    std::cerr << "expected robot onnxruntime default in CMakeLists.txt\n";
    return 1;
  }
  if (!contains(cmake_contents, "/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch")) {
    std::cerr << "expected robot libtorch default in CMakeLists.txt\n";
    return 1;
  }

  std::cout << "config_surface_test_pass=1\n";
  return 0;
}
