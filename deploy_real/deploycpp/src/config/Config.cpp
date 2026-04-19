#include "config/Config.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

std::string joinPath(const std::string& lhs, const std::string& rhs) {
  if (lhs.empty()) {
    return rhs;
  }
  if (lhs.back() == '/') {
    return lhs + rhs;
  }
  return lhs + "/" + rhs;
}

template <typename T>
T requiredValue(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing required config key: " + key);
  }
  return node[key].as<T>();
}

}  // namespace

Config loadConfig(const std::string& project_root) {
  Config config;
  config.project_root = project_root;
  config.real_yaml_path = joinPath(project_root, "deploy_real/config/real.yaml");
  config.fixed_pose_yaml_path = joinPath(project_root, "policy/fixedpose/config/FixedPose.yaml");
  config.passive_yaml_path = joinPath(project_root, "policy/passive/config/Passive.yaml");
  config.loco_mode_yaml_path = joinPath(project_root, "policy/loco_mode/config/LocoMode.yaml");
  config.beyond_mimic2_yaml_path = joinPath(project_root, "policy/beyond_mimic2/config/BeyondMimic2.yaml");
  config.beyond_mimic_yaml_path = joinPath(project_root, "policy/beyond_mimic/config/BeyondMimic.yaml");

  const YAML::Node root = YAML::LoadFile(config.real_yaml_path);
  config.net = requiredValue<std::string>(root, "net");
  config.num_joints = requiredValue<int>(root, "num_joints");
  config.lowcmd_topic = requiredValue<std::string>(root, "lowcmd_topic");
  config.lowstate_topic = requiredValue<std::string>(root, "lowstate_topic");
  config.control_dt = requiredValue<double>(root, "control_dt");
  config.error_over_time = requiredValue<int>(root, "error_over_time");

  if (config.num_joints != 29) {
    throw std::runtime_error("Expected num_joints=29 for G1, got " + std::to_string(config.num_joints));
  }

  const YAML::Node loco_mode = YAML::LoadFile(config.loco_mode_yaml_path);
  const std::string loco_policy_path = requiredValue<std::string>(loco_mode, "policy_path");
  config.loco_mode_model_path = joinPath(project_root, "policy/loco_mode/model/" + loco_policy_path);

  const YAML::Node beyond_mimic2 = YAML::LoadFile(config.beyond_mimic2_yaml_path);
  const std::string onnx_path = requiredValue<std::string>(beyond_mimic2, "onnx_path");
  config.beyond_mimic2_model_path = joinPath(project_root, "policy/beyond_mimic2/model/" + onnx_path);

  const YAML::Node beyond_mimic = YAML::LoadFile(config.beyond_mimic_yaml_path);
  const std::string beyond_mimic_onnx_path = requiredValue<std::string>(beyond_mimic, "onnx_path");
  config.beyond_mimic_model_path = joinPath(project_root, "policy/beyond_mimic/model/" + beyond_mimic_onnx_path);

  return config;
}
