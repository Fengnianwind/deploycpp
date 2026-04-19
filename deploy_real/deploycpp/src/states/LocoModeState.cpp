#include "states/LocoModeState.h"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
std::vector<T> requiredVector(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing LocoMode key: " + key);
  }
  return node[key].as<std::vector<T>>();
}

float requiredFloat(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing LocoMode key: " + key);
  }
  return node[key].as<float>();
}

int requiredInt(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing LocoMode key: " + key);
  }
  return node[key].as<int>();
}

float scaleToRange(float value, float min_value, float max_value) {
  return (value + 1.0f) * (max_value - min_value) * 0.5f + min_value;
}

std::array<float, 3> gravityOrientationFromQuat(const std::array<float, 4>& quaternion) {
  const float qw = quaternion[0];
  const float qx = quaternion[1];
  const float qy = quaternion[2];
  const float qz = quaternion[3];
  return {
      2.0f * (-qz * qx + qw * qy),
      -2.0f * (qz * qy + qw * qx),
      1.0f - 2.0f * (qw * qw + qz * qz),
  };
}

}  // namespace

LocoModeState::LocoModeState(const Config& config)
    : runner_([&config]() {
        const YAML::Node root = YAML::LoadFile(config.loco_mode_yaml_path);
        if (!root["policy_path"]) {
          throw std::runtime_error("Missing LocoMode key: policy_path");
        }
        std::filesystem::path policy_path = root["policy_path"].as<std::string>();
        if (policy_path.is_relative()) {
          policy_path = std::filesystem::path(config.loco_mode_yaml_path).parent_path().parent_path() / "model" / policy_path;
        }
        return policy_path.string();
      }()) {
  loadYaml(config.loco_mode_yaml_path);
  prev_action_.assign(static_cast<std::size_t>(num_actions_), 0.0f);
}

FSMStateName LocoModeState::name() const {
  return FSMStateName::LocoMode;
}

void LocoModeState::enter(const StateAndCmd&, PolicyOutput& output) {
  std::fill(prev_action_.begin(), prev_action_.end(), 0.0f);
  writeOutput(prev_action_, output);
}

void LocoModeState::run(const StateAndCmd& state, PolicyOutput& output) {
  const std::vector<float> obs = buildObs(state);
  const std::vector<float> action = runner_.run(obs);
  if (static_cast<int>(action.size()) != num_actions_) {
    throw std::runtime_error("Expected action size " + std::to_string(num_actions_) + ", got " +
                             std::to_string(action.size()));
  }
  writeOutput(action, output);
  prev_action_ = action;
}

void LocoModeState::exit(const StateAndCmd&, PolicyOutput&) {
  std::fill(prev_action_.begin(), prev_action_.end(), 0.0f);
}

FSMStateName LocoModeState::checkChange(const StateAndCmd& state) const {
  if (state.remote.F1) {
    return FSMStateName::Passive;
  }
  if (state.remote.start) {
    return FSMStateName::FixedPose;
  }
  if (state.remote.L1 && state.remote.Y) {
    return FSMStateName::BeyondMimic;
  }
  if (state.remote.R1 && state.remote.B) {
    return FSMStateName::BeyondMimic2;
  }
  return FSMStateName::LocoMode;
}

#ifdef LOCO_MODE_STATE_TEST_ONLY
std::vector<float> LocoModeState::buildObsForTest(const StateAndCmd& state) const {
  return buildObs(state);
}
#endif

void LocoModeState::loadYaml(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  kps_ = requiredVector<float>(root, "kps");
  kds_ = requiredVector<float>(root, "kds");
  default_angles_ = requiredVector<float>(root, "default_angles");
  joint2motor_idx_ = requiredVector<int>(root, "joint2motor_idx");
  cmd_scale_ = requiredVector<float>(root, "cmd_scale");
  ang_vel_scale_ = requiredFloat(root, "ang_vel_scale");
  dof_pos_scale_ = requiredFloat(root, "dof_pos_scale");
  dof_vel_scale_ = requiredFloat(root, "dof_vel_scale");
  action_scale_ = requiredFloat(root, "action_scale");
  num_obs_ = requiredInt(root, "num_obs");
  num_actions_ = requiredInt(root, "num_actions");

  if (!root["cmd_range"]) {
    throw std::runtime_error("Missing LocoMode key: cmd_range");
  }
  const YAML::Node cmd_range = root["cmd_range"];
  cmd_min_ = {
      cmd_range["lin_vel_x"][0].as<float>(),
      cmd_range["lin_vel_y"][0].as<float>(),
      cmd_range["ang_vel_z"][0].as<float>(),
  };
  cmd_max_ = {
      cmd_range["lin_vel_x"][1].as<float>(),
      cmd_range["lin_vel_y"][1].as<float>(),
      cmd_range["ang_vel_z"][1].as<float>(),
  };

  if (static_cast<int>(kps_.size()) != num_actions_ || static_cast<int>(kds_.size()) != num_actions_ ||
      static_cast<int>(default_angles_.size()) != num_actions_ ||
      static_cast<int>(joint2motor_idx_.size()) != num_actions_ || static_cast<int>(cmd_scale_.size()) != 3 ||
      static_cast<int>(cmd_min_.size()) != 3 || static_cast<int>(cmd_max_.size()) != 3 || num_obs_ != 96 ||
      num_actions_ != 29) {
    throw std::runtime_error("Invalid LocoMode config dimensions");
  }
}

std::vector<float> LocoModeState::buildObs(const StateAndCmd& state) const {
  std::vector<float> obs(static_cast<std::size_t>(num_obs_), 0.0f);
  const std::array<float, 3> gravity_orientation = gravityOrientationFromQuat(state.base_quat);

  for (int i = 0; i < 3; ++i) {
    obs[static_cast<std::size_t>(i)] = state.ang_vel[static_cast<std::size_t>(i)] * ang_vel_scale_;
    obs[static_cast<std::size_t>(3 + i)] = gravity_orientation[static_cast<std::size_t>(i)];
    obs[static_cast<std::size_t>(6 + i)] =
        scaleToRange(state.vel_cmd[static_cast<std::size_t>(i)], cmd_min_[static_cast<std::size_t>(i)],
                     cmd_max_[static_cast<std::size_t>(i)]) *
        cmd_scale_[static_cast<std::size_t>(i)];
  }

  for (int j = 0; j < num_actions_; ++j) {
    const int motor_idx = joint2motor_idx_[static_cast<std::size_t>(j)];
    if (motor_idx < 0 || motor_idx >= static_cast<int>(state.q.size())) {
      throw std::runtime_error("LocoMode joint2motor_idx out of range");
    }
    obs[static_cast<std::size_t>(9 + j)] =
        (state.q[static_cast<std::size_t>(motor_idx)] - default_angles_[static_cast<std::size_t>(j)]) * dof_pos_scale_;
    obs[static_cast<std::size_t>(9 + num_actions_ + j)] =
        state.dq[static_cast<std::size_t>(motor_idx)] * dof_vel_scale_;
    obs[static_cast<std::size_t>(9 + 2 * num_actions_ + j)] = prev_action_[static_cast<std::size_t>(j)];
  }

  return obs;
}

void LocoModeState::writeOutput(const std::vector<float>& action, PolicyOutput& output) const {
  output.actions.fill(0.0f);
  output.kps.fill(0.0f);
  output.kds.fill(0.0f);
  for (int j = 0; j < num_actions_; ++j) {
    const int motor_idx = joint2motor_idx_[static_cast<std::size_t>(j)];
    if (motor_idx < 0 || motor_idx >= static_cast<int>(output.actions.size())) {
      throw std::runtime_error("LocoMode joint2motor_idx out of range");
    }
    output.actions[static_cast<std::size_t>(motor_idx)] =
        action[static_cast<std::size_t>(j)] * action_scale_ + default_angles_[static_cast<std::size_t>(j)];
    output.kps[static_cast<std::size_t>(motor_idx)] = kps_[static_cast<std::size_t>(j)];
    output.kds[static_cast<std::size_t>(motor_idx)] = kds_[static_cast<std::size_t>(j)];
  }
}
